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
                 connect_timeout=60.0):
        self.system_address = system_address
        self.name = name
        self.connect_timeout = connect_timeout

        self.connected = False
        self.armed = False
        self.in_air = False
        self.battery_pct = -1.0
        self.heading_deg = None
        self.ground_speed = None
        self.num_sats = -1
        # latest position: dict(lat, lon, abs_alt, rel_alt)
        self.position = None
        self.home_position = None

        self._loop = asyncio.new_event_loop()
        self._thread = None
        self.drone = System()

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
    async def _connect(self):
        await self.drone.connect(system_address=self.system_address)
        try:
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.connected = True
                    break
            async for health in self.drone.telemetry.health():
                if health.is_home_position_ok:
                    break
            async for home in self.drone.telemetry.home():
                self.home_position = {
                    "lat": home.latitude_deg, "lon": home.longitude_deg}
                break
            self._stream_telemetry()
        except Exception:
            self.connected = False
            raise

    def _stream_telemetry(self):
        async def _position():
            async for p in self.drone.telemetry.position():
                self.position = {
                    "lat": p.latitude_deg,
                    "lon": p.longitude_deg,
                    "abs_alt": p.absolute_altitude_m,
                    "rel_alt": p.relative_altitude_m,
                }

        async def _battery():
            async for b in self.drone.telemetry.battery():
                pct = b.remaining_percent
                self.battery_pct = pct * 100.0 if pct <= 1.0 else float(pct)

        async def _armed_in_air():
            async for a in self.drone.telemetry.armed():
                self.armed = a

        async def _in_air():
            async for a in self.drone.telemetry.landed_state():
                self.in_air = a == (
                    __import__("mavsdk").telemetry.LandedState.IN_AIR)

        async def _heading():
            async for h in self.drone.telemetry.heading():
                self.heading_deg = h.heading_deg

        async def _speed():
            async for v in self.drone.telemetry.velocity_ned():
                self.ground_speed = math.hypot(v.east_m_s, v.north_m_s)

        async def _gps():
            async for i in self.drone.telemetry.gps_info():
                self.num_sats = i.num_satellites

        for coro in (_position(), _battery(), _armed_in_air(), _in_air(),
                     _heading(), _speed(), _gps()):
            asyncio.ensure_future(coro)

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

    # ---------- flight actions ----------
    async def arm_and_takeoff(self, altitude_m):
        self.require_connection()
        await self.drone.action.set_takeoff_altitude(float(altitude_m))
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        # wait until airborne at target-ish altitude
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
