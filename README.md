# TEKNOFEST IDA Autonomy

ROS 2 Humble package for the TEKNOFEST Insansiz Deniz Araci autonomy stack.
The current codebase focuses on Parkur-1 GPS and heading based waypoint
tracking in simulation. Real Pixhawk, camera, depth, lidar, YKI, and kill-chain
integration is intentionally conservative and still requires hardware-specific
verification.

## Current Status

- `mission_manager_node` loads a waypoint JSON mission and publishes mission
  state. Real runs stay idle until `/mission/start` is received.
- `gps_guidance_node` computes distance and bearing to the active waypoint.
- `controller_node` publishes `/control/cmd_vel` and sends zero command when
  the mission has not started or is completed.
- `safety_node` gates `/control/cmd_vel` into `/control/cmd_vel_safe` using
  a latched `/safety/kill`.
- `sim_gps_node` provides a simple GPS and compass simulation for Parkur-1.
- `logger_node` writes competition telemetry CSV files to a configurable
  directory.
- `local_costmap_node` publishes `/local_costmap` and records a 1 Hz obstacle
  map CSV from lidar scans.
- `yki_bridge_node` sends telemetry over UDP and can optionally receive
  pre-start mission commands plus emergency kill.
- `mavros_bridge_node` is prepared as a guarded integration point, but real
  actuation is disabled by default.
- `rc_kill_node` reads MAVROS RC channel input and maps channel 7 to a latched
  software kill.
- `power_relay_node` is a disabled-by-default Jetson GPIO hook for the future
  SSR/contactor chain.

Not complete yet: RealSense RGB plus aligned depth, YOLO buoy/target detection,
RPLiDAR S3 local planning, Parkur-2 cost map and obstacle avoidance, Parkur-3
UAV target color handoff, production YKI UI, and verified hardware kill switch
integration.

## Build

From the workspace root:

```bash
colcon build --symlink-install --packages-select ida_otonom
source install/setup.bash
```

## Parkur-1 Simulation

```bash
ros2 launch ida_otonom parkur1_sim.launch.py
```

Useful launch arguments:

```bash
ros2 launch ida_otonom parkur1_sim.launch.py \
  mission_file:=/path/to/mission.json \
  arrival_radius_m:=3.0 \
  log_dir:=/tmp/ida_otonom_logs \
  enable_logger:=true \
  enable_costmap_logger:=false \
  enable_yki_bridge:=false
```

The installed default mission is resolved from the package share directory:
`share/ida_otonom/missions/mission.json`.

## MAVROS Safety Notes

`mavros_bridge_node` does not arm the vehicle, does not change flight mode, and
does not publish real actuator commands unless `enabled:=true` is explicitly set.
For your current hardware plan, assume two rear thrusters with separate ESCs.
Verify Pixhawk output mapping before enabling real commands, likely starting
with `MAIN OUT 1 = left ESC` and `MAIN OUT 2 = right ESC` if the electronics
team keeps that layout.
Before enabling it on the real boat, verify:

- Pixhawk Cube Orange+ is running the expected ArduRover version and mode.
- Jetson connects to Pixhawk over UART and the baudrate is confirmed.
- ESC PWM neutral, forward, and reverse values are measured.
- The physical emergency stop, RC kill, and YKI kill all remove real motor power.
- MAVROS topic choice is correct for the firmware configuration.
- Command axis mapping and scaling are tested with props/impellers disabled.
- Command timeout and `/safety/kill` behavior are observed on the real system.

Example dry status run:

```bash
ros2 run ida_otonom mavros_bridge_node
```

Real-boat stack dry launch. This still does not publish motor commands:

```bash
ros2 launch ida_otonom ida_real.launch.py
```

Example only after hardware verification:

```bash
ros2 run ida_otonom safety_node
ros2 run ida_otonom mavros_bridge_node --ros-args \
  -p enabled:=true \
  -p input_topic:=/control/cmd_vel_safe \
  -p output_mode:=cmd_vel \
  -p cmd_vel_output_topic:=/mavros/setpoint_velocity/cmd_vel_unstamped
```

## Competition Compliance Notes

The 2026 specification has a few software-critical rules baked into the current
architecture:

- YKI must not run autonomy, sensor processing, or image processing. Keep those
  nodes on Jetson; use YKI only for mission upload/start, telemetry display, and
  emergency kill.
- No image/video stream may be sent from IDA or UAV to ground. `yki_bridge_node`
  sends telemetry JSON only; processed camera video is recorded locally.
- After mission start, YKI/RC commands are forbidden except emergency motor
  power cut. `yki_bridge_node` ignores non-kill commands once `/mission/started`
  is true.
- The real motor path must go through `/control/cmd_vel_safe`; `safety_node`,
  `mavros_bridge_node`, Pixhawk failsafe, RC kill, YKI kill, and the physical
  contactor chain must all be verified before wet tests.
- Required delivery data is covered by local logs: processed camera MP4,
  telemetry CSV, and local costmap CSV. CSV/video artifacts are ignored by git.
- Parkur-2/3 perception is intentionally modular because the buoy/target dataset
  is not available yet. Model training can be added later without changing the
  mission, safety, logging, and MAVROS boundaries.

Example YKI command receiver for bench tests:

```bash
ros2 run ida_otonom yki_bridge_node --ros-args \
  -p enable_command_rx:=true \
  -p command_bind_port:=5006
```

Supported command JSON payloads are:

```json
{"command": "start_mission"}
{"command": "kill", "active": true}
{"command": "reset_kill"}
{"command": "set_target_color", "color": "red"}
```

Current hardware decisions captured in `config/ida_real.yaml`:

- Two rear thrusters, separate ESCs through Pixhawk.
- GPS and heading come from Pixhawk/MAVROS.
- RC receiver connects to Pixhawk.
- RC kill defaults to channel 7, active above 1800 PWM.
- Jetson-Pixhawk link is UART, but baudrate is still a hardware TODO.
- The 24 V contactor will cut motor current through SSR/relay; GPIO control is
  present but disabled until pin, logic level, and external safety circuit are
  verified.

## Repository Hygiene

Generated folders and artifacts such as `build/`, `install/`, `log/`,
`__pycache__/`, `*.pyc`, telemetry CSV files, videos, and `.DS_Store` files
should stay out of git.
