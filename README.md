# UAV Project - Drone Detector & Interceptor

Autonomous fixed-wing UAV that patrols an area, detects intruder drones with a
YOLO camera model, chases and locks onto them, and executes interception after
an approval step (human or LLM). Built on ROS 2 Humble + PX4 SITL + Gazebo.

Copy of `wing_braker_uav`, reworked following the engineering patterns of
`pothole_remasterd` (launch orchestration, config, dashboard).

---

## Architecture

```
camera (zam_uav_v2 gimbal) --gzbridge--> /camera/image_raw
        |
        v
detector  (YOLO, model_path from config; simulated mode until trained .pt lands)
        |  IntruderDetection (lat/lon/alt/confidence)
        v
brain     PATROL -> LOCK/CHASE (fly + orbit) -> DECISION -> ENGAGE -> REPORT -> PATROL
        |             |
        |             |-- approval_mode: human -> /request_interception (dashboard/CLI)
        |             |-- approval_mode: llm   -> intercept_llm.py (separate node, mock default)
        v             v
engagement_node (simulated missile launcher) -> InterceptReport -> web_dashboard :8080
```

## Packages

| Package | Contents |
|---|---|
| `wingbreaker_interfaces` | `FlyToGPS.action`, `IntruderDetection`, `VehicleStatus`, `MissionState`, `InterceptReport` msgs, `RequestInterception.srv` |
| `wingbreaker_uav` | brain, flight_node, safety_node, engagement_node, detector, intercept_llm, web_dashboard, wait_for_topic + scripts (`novnc_bridge.py`, `intruder_sim.py`) |

## Quick start

```bash
cd ~/UAV_Project
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# full stack: PX4 + Gazebo + camera bridge + all nodes + intruder drone
ros2 launch wingbreaker_uav wingbreaker.launch.py

# options
ros2 launch wingbreaker_uav wingbreaker.launch.py intruder_mode:=static   # parked target
ros2 launch wingbreaker_uav wingbreaker.launch.py approval_mode:=llm run_llm:=true

# optional: QGC in browser via noVNC (needs xvfb + x11vnc installed)
ros2 launch wingbreaker_uav qgc_novnc.launch.py
# then open http://localhost:6080/vnc.html or use the QGC tab on the dashboard
```

Dashboard: **http://localhost:8080** - mission state, telemetry, camera feed,
detections, intercept reports, Approve button (human mode), QGC tab (when
novnc_bridge is running).

## Detection model

`detector` reads `model_path` from `config/uav.yaml`. It ships with the stock
`yolov8n.pt` placeholder and `use_simulated: true` so the full pipeline runs
before your trained weights exist. To go live:

1. Copy your trained drone-detection `.pt` into this folder.
2. In `config/uav.yaml` set `detector.model_path` to its filename,
   `target_classes: ["drone"]` (adjust to your class name),
   `use_simulated: false`.

## Interception flow

1. Detector publishes an intruder above `detection_conf_threshold`.
2. Brain abandons patrol, flies to the target (LOCK), orbits it.
3. Approval:
   - `human`: click **Approve** on the dashboard or call the service from CLI
   - `llm`: the separate `intercept_llm` node decides (mock policy by default;
     wire a real provider in `_ask_llm`)
   - `auto`: engage immediately
4. Engagement fires (simulated), brain publishes `InterceptReport`
   (detected / intercepted flags, location, lat, lon, alt) shown on the
   dashboard.

## Sim assets (`sim/`)

- `models/zam_uav_v2` - Reaper-style fixed-wing (IMU, baro, mag, GPS,
  airspeed plugin, gimbal camera; lidar removed)
- `models/intruder_drone` - static-flagged quad target moved kinematically
- `airframes/4031_gz_zam_uav_v2` - PX4 airframe (installed into PX4 ROMFS)

## System dependencies (optional, QGC-in-browser only)

```bash
sudo apt install xvfb x11vnc          # not needed for the core sim
pip install --user websockify         # already done if ~/.local/bin/websockify exists
# noVNC app is vendored at thirdparty/noVNC
```

## Author

**xKhalid1**
