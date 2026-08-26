"""Shared MAVSDK vehicle gateway for the Wingbreaker UAV stack.

Owns one asyncio event loop on a background thread and a single MAVSDK
System connection. All MAVSDK calls must go through this loop; sync callers
use `submit()` (non-blocking) or `call()` (blocking).

Telemetry (position / battery / armed / in-air / heading) is streamed
continuously and exposed as plain attributes readable from any thread.
"""

import asyncio
import math
import threading

from mavsdk import System
from mavsdk.telemetry import LandedState


class GatewayNotConnected(RuntimeError):
    """Raised when an action is requested before the vehicle is connected."""


def distance_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in meters between two GPS points."""
    r = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return r * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def destination_point(lat_deg, lon_deg, bearing_deg, range_m):
    """Flat-earth destination point: bearing (deg from north), range (m)."""
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat_deg))
    br = math.radians(bearing_deg)
    dlat = (range_m * math.cos(br)) / meters_per_deg_lat
    dlon = (range_m * math.sin(br)) / meters_per_deg_lon
    return lat_deg + dlat, lon_deg + dlon


class DroneGateway:
    """One MAVSDK connection with a private asyncio loop on a daemon thread."""

    def __init__(self, system_address="udpin://0.0.0.0:14540", name="gateway",
                 connect_timeout=60.0, grpc_port=50051):
        self.system_address = system_address
        self.name = name
        self.connect_timeout = connect_timeout
        self.grpc_port = grpc_port

        self.connected = False
        self.armed = False
        self.in_air = False
        self._landed_in_air = False
        self.battery_pct = -1.0
        self.heading_deg = None
        self.ground_speed = None
        self.num_sats = -1
        # latest position: dict(lat, lon, abs_alt, rel_alt)
        self.position = None
        self.home_position = None

        self._loop = asyncio.new_event_loop()
        self._thread = None
        # dedicated gRPC port per gateway - otherwise MAVSDK reuses whatever
        # stale mavsdk_server is squatting on 50051 from previous runs
        self.drone = System(port=grpc_port)

    # ---------- lifecycle ----------
    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run_loop, name=f"gw-{self.name}", daemon=True)
        self._thread.start()
        asyncio.run_coroutine_threadsafe(self._connect(), self.loop)

    def stop(self):
        if self._loop is not None:
            self.loop.call_soon_threadsafe(self.loop.stop)

    @property
    def loop(self):
        if self._loop is None:
            raise RuntimeError("Gateway not started")
        return self._loop

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    # ---------- connection + telemetry ----------
    def _reset_telemetry(self):
        """Drop stale values so nothing looks alive while disconnected."""
        self.position = None
        self.battery_pct = -1.0
        self.ground_speed = None
        self.heading_deg = None
        self.num_sats = -1
        self.armed = False
        self._landed_in_air = False
        self._update_airborne()

    async def _connect(self):
        """Supervise the MAVSDK link forever.

        Every (re)connect builds a FRESH System - and therefore a fresh
        mavsdk_server on this gateway's dedicated gRPC port - then streams
        telemetry and watches connection_state. Any dropout or stream death
        tears everything down and reconnects, so callers never see frozen
        telemetry.
        """
        while True:
            try:
                self.connected = False
                self._reset_telemetry()
                self.drone = System(port=self.grpc_port)
                await self.drone.connect(system_address=self.system_address)
                async for state in self.drone.core.connection_state():
                    self.connected = state.is_connected
                    if state.is_connected:
                        break
                # optional extras - never fatal
                try:
                    async for home in self.drone.telemetry.home():
                        self.home_position = {
                            "lat": home.latitude_deg,
                            "lon": home.longitude_deg}
                        break
                except Exception:       # noqa: BLE001
                    pass
                self.get_logger_safe(
                    f'gateway connected to {self.system_address}')
                await self._stream_telemetry()   # returns when streams die
                self.get_logger_safe('telemetry lost - reconnecting')
            except Exception as e:      # noqa: BLE001
                self.get_logger_safe(f'connect retry: {e}')
            finally:
                self.connected = False
            await asyncio.sleep(2.0)

    def get_logger_safe(self, msg):
        print(f'[gw-{self.name}] {msg}', flush=True)

    async def _stream_telemetry(self):
        """Run all telemetry streams; returns when the first one dies."""

        async def _run(name, coro):
            try:
                await coro
            except Exception as e:      # noqa: BLE001
                self.get_logger_safe(f'{name} stream ended: {e}')

        streams = {
            'position': self.drone.telemetry.position(),
            'battery': self.drone.telemetry.battery(),
            'armed': self.drone.telemetry.armed(),
            'landed': self.drone.telemetry.landed_state(),
            'heading': self.drone.telemetry.heading(),
            'velocity': self.drone.telemetry.velocity_ned(),
            'gps': self.drone.telemetry.gps_info(),
        }
        handlers = {
            'position': self._on_position,
            'battery': self._on_battery,
            'armed': self._on_armed,
            'landed': self._on_landed,
            'heading': self._on_heading,
            'velocity': self._on_velocity,
            'gps': self._on_gps,
        }

        async def _pump(name):
            stream = streams[name]
            handler = handlers[name]
            async for msg in stream:
                handler(msg)

        tasks = [asyncio.ensure_future(_pump(n)) for n in streams]
        done, pending = await asyncio.wait(
            tasks, return_when=asyncio.FIRST_COMPLETED)
        for t in pending:
            t.cancel()

    # one small handler per stream keeps the pump generic
    def _on_position(self, p):
        self.position = {
            "lat": p.latitude_deg,
            "lon": p.longitude_deg,
            "abs_alt": p.absolute_altitude_m,
            "rel_alt": p.relative_altitude_m,
        }
        self._update_airborne()

    def _on_battery(self, b):
        pct = b.remaining_percent
        self.battery_pct = pct * 100.0 if pct <= 1.0 else float(pct)

    def _on_armed(self, a):
        self.armed = a

    def _on_landed(self, a):
        # PX4 fixed-wing keeps LandedState UNDEFINED in cruise - fall back
        # to an altitude heuristic so in_air is reliable for both types
        self._landed_in_air = a == LandedState.IN_AIR
        self._update_airborne()

    def _on_heading(self, h):
        self.heading_deg = h.heading_deg

    def _on_velocity(self, v):
        self.ground_speed = math.hypot(v.east_m_s, v.north_m_s)

    def _on_gps(self, i):
        self.num_sats = i.num_satellites

    # ---------- action plumbing ----------
    def submit(self, coro):
        """Schedule a coroutine on the gateway loop; returns a Future."""
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def call(self, coro, timeout=None):
        """Run a coroutine on the gateway loop and block for the result."""
        return self.submit(coro).result(timeout)

    def require_connection(self):
        if not self.connected:
            raise GatewayNotConnected(
                f"[{self.name}] not connected to {self.system_address}")

    def _update_airborne(self):
        """in_air = LandedState IN_AIR OR relative altitude above 3 m.

        PX4 fixed-wing keeps LandedState UNDEFINED during cruise, so the
        altitude fallback is what makes in_air reliable for the zam_uav.
        """
        alt_ok = bool(self.position and self.position["rel_alt"] > 3.0)
        self.in_air = self._landed_in_air or alt_ok

    # ---------- flight actions ----------
    async def arm_and_takeoff(self, altitude_m):
        """Idempotent + patient: retries arming through PX4 preflight warmup."""
        self.require_connection()
        deadline = asyncio.get_event_loop().time() + 60.0
        while not self.armed:
            try:
                await self.drone.action.set_takeoff_altitude(float(altitude_m))
                await self.drone.action.arm()
            except Exception as e:      # noqa: BLE001
                if asyncio.get_event_loop().time() > deadline:
                    raise
                self.get_logger_safe(f'arm retry ({e})')
                await asyncio.sleep(3.0)
        if not self.in_air:
            await self.drone.action.takeoff()
        # wait until airborne near target altitude
        while not self.in_air or (
                self.position and self.position["rel_alt"] < altitude_m * 0.9):
            await asyncio.sleep(0.5)

    async def goto(self, lat, lon, alt_m, yaw_deg=0.0):
        self.require_connection()
        await self.drone.action.goto_location(
            float(lat), float(lon), float(alt_m), float(yaw_deg))

    async def hold(self):
        self.require_connection()
        await self.drone.action.hold()

    async def land(self):
        self.require_connection()
        await self.drone.action.land()

    async def wait_until_within(self, lat, lon, radius_m, timeout_s=120.0):
        """Block until the vehicle is within radius_m of the target point."""
        deadline = asyncio.get_event_loop().time() + timeout_s
        while asyncio.get_event_loop().time() < deadline:
            pos = self.position
            if pos and distance_m(pos["lat"], pos["lon"], lat, lon) <= radius_m:
                return True
            await asyncio.sleep(0.5)
        return False
