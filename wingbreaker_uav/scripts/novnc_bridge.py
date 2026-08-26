#!/usr/bin/env python3
"""Bridge that serves headless QGroundControl through a browser via VNC.

Plain Python script (not a ROS node). Orchestrates four processes:

    1. Xvfb       - virtual X11 display
    2. QGC        - QGroundControl AppImage rendered on the virtual display
    3. x11vnc     - VNC server bound to the virtual display
    4. websockify - HTTP/WebSocket proxy exposing noVNC in a browser

Examples:
    novnc_bridge.py                 # start missing pieces and supervise
    novnc_bridge.py --check         # JSON status of the noVNC endpoint
    novnc_bridge.py --stop          # stop the whole stack
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

LOG_DIR = Path.home() / ".uav_project_novnc" / "logs"

DEFAULT_DISPLAY = ":99"
DEFAULT_QGC_PATH = "/home/khalid/QGroundControl-x86_64.AppImage"
DEFAULT_VNC_PORT = 5901
DEFAULT_NOVNC_PORT = 6080

NOVNC_WEB_FALLBACKS = [
    Path("/home/khalid/UAV_Project/thirdparty/noVNC"),
    Path("/usr/share/novnc"),
]

SUPERVISOR_INTERVAL_S = 5.0
STARTUP_TIMEOUT_S = 60.0
STOP_TIMEOUT_S = 5.0
CHECK_TIMEOUT_S = 1.0
MAX_RESTARTS_PER_CHILD = 3

_shutdown_requested = False


class ProcessSpec:
    """Static description of one managed child process."""

    def __init__(
        self,
        name: str,
        pgrep_pattern: str,
        argv: list[str],
        env: dict[str, str] | None = None,
    ) -> None:
        self.name = name
        self.pgrep_pattern = pgrep_pattern
        self.argv = argv
        self.env = env


# ---------------------------------------------------------------------------
# Small utilities
# ---------------------------------------------------------------------------


def log_status(message: str) -> None:
    """Print a timestamped status line."""
    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {message}", flush=True)


def novnc_url(port: int) -> str:
    """Browser URL for the noVNC client page."""
    return f"http://localhost:{port}/vnc.html"


def tcp_port_open(port: int, host: str = "localhost", timeout: float = CHECK_TIMEOUT_S) -> bool:
    """Return True if a TCP connection to host:port succeeds."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def sleep_interruptible(total_seconds: float) -> None:
    """Sleep in short slices so shutdown signals are noticed quickly."""
    deadline = time.monotonic() + total_seconds
    while not _shutdown_requested:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(0.25, remaining))


def wait_for_port(port: int, timeout: float) -> bool:
    """Poll a TCP port until it opens, the deadline passes, or shutdown starts."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _shutdown_requested:
            return False
        if tcp_port_open(port):
            return True
        time.sleep(0.5)
    return False


# ---------------------------------------------------------------------------
# Process discovery (pgrep based)
# ---------------------------------------------------------------------------


def _cmdline_contains(pid: int, marker: str) -> bool:
    """Return True if /proc/<pid>/cmdline contains marker."""
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return False
    return any(marker.encode() in part for part in raw.split(b"\0"))


def find_pids(pgrep_pattern: str) -> list[int]:
    """Find PIDs matching pattern, excluding this script and its launchers.

    The QGC path appears in our own command line, so a naive pgrep -f would
    match this bridge (and the shell that launched it). Filter both out.
    """
    result = subprocess.run(
        ["pgrep", "-f", pgrep_pattern],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    marker = Path(sys.argv[0]).name
    own_pid = os.getpid()
    pids: list[int] = []
    for token in result.stdout.split():
        try:
            pid = int(token)
        except ValueError:
            continue
        if pid == own_pid:
            continue
        if _cmdline_contains(pid, marker):
            continue
        pids.append(pid)
    return pids


def is_running(pgrep_pattern: str) -> bool:
    """Return True if any foreign process matches the pgrep pattern."""
    return bool(find_pids(pgrep_pattern))


def pid_alive(pid: int) -> bool:
    """Return True if the PID still exists (signal 0 probe)."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


# ---------------------------------------------------------------------------
# Starting and stopping
# ---------------------------------------------------------------------------


def log_path_for(spec: ProcessSpec) -> Path:
    """Per-process log file under ~/.uav_project_novnc/logs/."""
    return LOG_DIR / f"{spec.name}.log"


def spawn_logged(spec: ProcessSpec) -> subprocess.Popen:
    """Start a child with stdout/stderr appended to its own log file."""
    log_path = log_path_for(spec)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    env = spec.env if spec.env is not None else os.environ.copy()
    with open(log_path, "ab") as log_file:
        banner = f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] starting: {' '.join(spec.argv)}\n"
        log_file.write(banner.encode())
        log_file.flush()
        return subprocess.Popen(
            spec.argv,
            stdin=subprocess.DEVNULL,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            env=env,
        )


def stop_popen(proc: subprocess.Popen, timeout: float = STOP_TIMEOUT_S) -> None:
    """SIGTERM a child, wait up to timeout, then SIGKILL."""
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def stop_by_pattern(pgrep_pattern: str) -> bool:
    """SIGTERM matched processes, wait up to STOP_TIMEOUT_S, SIGKILL survivors."""
    pids = find_pids(pgrep_pattern)
    if not pids:
        return False
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + STOP_TIMEOUT_S
    while time.monotonic() < deadline:
        if not any(pid_alive(pid) for pid in pids):
            break
        time.sleep(0.2)
    for pid in pids:
        if pid_alive(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    return True


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        prog="novnc_bridge.py",
        description="Serve headless QGroundControl in a browser via Xvfb, x11vnc and noVNC.",
    )
    parser.add_argument(
        "--display",
        default=DEFAULT_DISPLAY,
        help="X display used by Xvfb, x11vnc and QGC (default: %(default)s)",
    )
    parser.add_argument(
        "--qgc-path",
        default=DEFAULT_QGC_PATH,
        help="Path to the QGroundControl AppImage (default: %(default)s)",
    )
    parser.add_argument(
        "--vnc-port",
        type=int,
        default=DEFAULT_VNC_PORT,
        help="Port for x11vnc (default: %(default)s)",
    )
    parser.add_argument(
        "--novnc-port",
        type=int,
        default=DEFAULT_NOVNC_PORT,
        help="HTTP/WebSocket port for websockify/noVNC (default: %(default)s)",
    )
    parser.add_argument(
        "--novnc-web",
        default=None,
        help="Directory containing noVNC web assets (overrides $NOVNC_WEB and fallbacks)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Print JSON status of the noVNC endpoint and exit",
    )
    parser.add_argument(
        "--stop",
        action="store_true",
        help="Stop the managed processes in reverse order and exit",
    )
    return parser.parse_args(argv)


def resolve_novnc_web_dir(explicit: str | None) -> Path | None:
    """Resolve the noVNC web asset directory: flag, then $NOVNC_WEB, then fallbacks."""
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    env_value = os.environ.get("NOVNC_WEB")
    if env_value:
        candidates.append(Path(env_value))
    candidates.extend(NOVNC_WEB_FALLBACKS)
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    print("ERROR: no noVNC web directory found. Tried:", file=sys.stderr)
    for candidate in candidates:
        print(f"  - {candidate}", file=sys.stderr)
    return None


def build_specs(args: argparse.Namespace, novnc_web_dir: Path | None) -> list[ProcessSpec]:
    """Build the four managed process descriptions in startup order.

    The QGC pattern matches any casing because AppImages re-exec into a mount
    whose command line differs from the original AppImage path.
    """
    qgc_env = os.environ.copy()
    qgc_env["DISPLAY"] = args.display
    qgc_env["QT_QUICK_CONTROLS_STYLE"] = "Default"
    web_dir_arg = str(novnc_web_dir) if novnc_web_dir is not None else ""
    return [
        ProcessSpec(
            name="xvfb",
            pgrep_pattern=f"Xvfb {args.display}",
            argv=["Xvfb", args.display, "-screen", "0", "1920x1080x24"],
        ),
        ProcessSpec(
            name="qgroundcontrol",
            pgrep_pattern=r"[Qq][Gg]round[Cc]ontrol",
            argv=[str(args.qgc_path)],
            env=qgc_env,
        ),
        ProcessSpec(
            name="x11vnc",
            pgrep_pattern=f"x11vnc -display {args.display}",
            argv=[
                "x11vnc",
                "-display",
                args.display,
                "-forever",
                "-shared",
                "-rfbport",
                str(args.vnc_port),
                "-nopw",
            ],
        ),
        ProcessSpec(
            name="websockify",
            pgrep_pattern=rf"websockify .*\b{args.novnc_port}\b",
            argv=[
                "websockify",
                "--web",
                web_dir_arg,
                str(args.novnc_port),
                f"localhost:{args.vnc_port}",
            ],
        ),
    ]


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------


def run_check(args: argparse.Namespace) -> int:
    """Report whether the noVNC HTTP endpoint answers. Side-effect free."""
    running = tcp_port_open(args.novnc_port)
    print(json.dumps({"running": running, "novnc_url": novnc_url(args.novnc_port)}))
    return 0


def run_stop(specs: list[ProcessSpec]) -> int:
    """Terminate all managed processes in reverse startup order."""
    for spec in reversed(specs):
        if stop_by_pattern(spec.pgrep_pattern):
            log_status(f"{spec.name}: stopped")
        else:
            log_status(f"{spec.name}: not running")
    return 0


def child_is_alive(
    spec: ProcessSpec,
    proc: subprocess.Popen | None,
) -> bool:
    """Liveness of an owned child (poll) or an external process (pgrep)."""
    if proc is not None:
        return proc.poll() is None
    return is_running(spec.pgrep_pattern)


def shutdown_owned(specs: list[ProcessSpec], procs: dict[str, subprocess.Popen | None]) -> None:
    """Stop the children this bridge owns, in reverse order."""
    log_status("stopping owned children in reverse order")
    for spec in reversed(specs):
        proc = procs.get(spec.name)
        if proc is None:
            continue
        if proc.poll() is None:
            stop_popen(proc)
            log_status(f"{spec.name}: stopped")


def run_supervisor(args: argparse.Namespace, specs: list[ProcessSpec]) -> int:
    """Start missing processes, wait for noVNC, then supervise and restart."""
    procs: dict[str, subprocess.Popen | None] = {}
    restarts: dict[str, int] = {spec.name: 0 for spec in specs}

    for spec in specs:
        if is_running(spec.pgrep_pattern):
            procs[spec.name] = None
            log_status(f"{spec.name}: already running, leaving as-is")
        else:
            try:
                procs[spec.name] = spawn_logged(spec)
                log_status(f"{spec.name}: started (log: {log_path_for(spec)})")
            except (FileNotFoundError, PermissionError, OSError) as exc:
                procs[spec.name] = None
                log_status(f"{spec.name}: failed to start ({exc})")

    install_signal_handlers()

    log_status(f"waiting up to {STARTUP_TIMEOUT_S:.0f}s for noVNC on port {args.novnc_port}")
    if not wait_for_port(args.novnc_port, STARTUP_TIMEOUT_S) and not _shutdown_requested:
        log_status(f"WARNING: noVNC did not answer on port {args.novnc_port} within {STARTUP_TIMEOUT_S:.0f}s")
    print(novnc_url(args.novnc_port), flush=True)

    while not _shutdown_requested:
        sleep_interruptible(SUPERVISOR_INTERVAL_S)
        if _shutdown_requested:
            break
        statuses: list[str] = []
        for spec in specs:
            if child_is_alive(spec, procs[spec.name]):
                statuses.append(f"{spec.name}=up")
                continue
            if restarts[spec.name] >= MAX_RESTARTS_PER_CHILD:
                statuses.append(f"{spec.name}=dead(give-up)")
                continue
            restarts[spec.name] += 1
            try:
                procs[spec.name] = spawn_logged(spec)
                statuses.append(
                    f"{spec.name}=restarted({restarts[spec.name]}/{MAX_RESTARTS_PER_CHILD})"
                )
                log_status(f"{spec.name}: died, restarted "
                           f"({restarts[spec.name]}/{MAX_RESTARTS_PER_CHILD})")
            except (FileNotFoundError, PermissionError, OSError) as exc:
                statuses.append(f"{spec.name}=restart-failed")
                log_status(f"{spec.name}: restart failed ({exc})")
        log_status("status: " + ", ".join(statuses))

    shutdown_owned(specs, procs)
    log_status("bridge stopped")
    return 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def install_signal_handlers() -> None:
    """Install SIGINT/SIGTERM handlers that request a clean shutdown."""

    def handle_signal(signum: int, frame: object) -> None:
        del signum, frame  # unused
        global _shutdown_requested
        _shutdown_requested = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)


def main(argv: list[str] | None = None) -> int:
    """CLI entry point dispatching to check, stop or supervise modes."""
    args = parse_args(argv)

    if args.check:
        return run_check(args)

    if args.stop:
        # Stopping only needs the pgrep patterns, not the web asset directory.
        return run_stop(build_specs(args, None))

    novnc_web_dir = resolve_novnc_web_dir(args.novnc_web)
    if novnc_web_dir is None:
        return 1

    qgc_path = Path(args.qgc_path)
    if not qgc_path.is_file():
        print(f"ERROR: QGC AppImage not found: {qgc_path}", file=sys.stderr)
        return 1

    return run_supervisor(args, build_specs(args, novnc_web_dir))


if __name__ == "__main__":
    sys.exit(main())
