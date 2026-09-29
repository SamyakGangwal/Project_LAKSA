# Setup and deployment

[Index](README.md) · Previous: [Console and operation](07_console_and_operation.md) · Next: [Performance optimization](09_performance_optimization.md)

## Target machine

This work runs on a **bench Jetson**, not the original car computer: an Orin Nano Super developer kit with JetPack 6 (L4T R36.5) and Ubuntu 22.04, user `samyak`. It was set up to match the car's software stack. The ESP32, VESC, LiDAR and ZED are attached to it.

The micro-ROS agent that bridges the ESP32 (FastDDS, `ROS_DOMAIN_ID=0`, not localhost-only), and a health service, were already installed as boot services under another account. They are **reused and never restarted**, and the ESP32's serial port is never opened directly.

## One-time setup

Scripts are in `firmware/esp32-s3/jetson/setup/`.

| Step | Script | What it does |
|---|---|---|
| 1 | `sudo bash 01_system_setup.sh` | Adds the ROS 2 Humble apt source and installs the stack's packages (Nav2, RTAB-Map, robot_localization, SLAM Toolbox, laser_filters, joy, CycloneDDS…). Runs `rosdep init` and writes `/etc/laksa/`. **Doesn't** upgrade packages, install udev rules or services, change groups or open serial devices. |
| 2 | `bash 02_build_workspaces.sh` | Builds as the normal user, with no sudo. `~/third_party/third_party_ws` gets `sllidar_ros2` and `rf2o_laser_odometry` pinned to the car's commit plus `patches/rf2o-laser-odometry-laksa.patch`. `~/laksa_ws` gets `laksa_interfaces` and all Jetson packages, **symlinked** to the repo checkout at `~/src/Project_LAKSA`. |
| 3 | ZED wrapper | `zed-ros2-wrapper` built in `~/zed_ws` against the ZED SDK 5.5 already on the machine. The user must be in the `zed` group. |

Differences from the original car:
- The apt RTAB-Map is **0.23.7**; the car ran 0.22.1.
- `laksa_dense_map` hit an OpenCV clash between JetPack's build and ROS's. It was fixed with a linker alias inside `~/laksa_ws`, recorded in `02_build_workspaces.sh`. That package isn't launched.

## Launcher

`setup/dryrun_bringup.sh` starts every process from [System architecture](02_system_architecture.md#process-map):

| Command | Effect |
|---|---|
| `dryrun_bringup.sh start` | everything, supervisor actuation **disabled** (brake only); cap 1000 eRPM / 0.24 m/s |
| `dryrun_bringup.sh trial` | everything, actuation **enabled**; cap **620 eRPM / 0.15 m/s** |
| `dryrun_bringup.sh race` | actuation enabled, **no operator needed**; ARM in the console, the car starts on a green signal and stops on red; up to 3.0 m/s |
| `dryrun_bringup.sh auto` | trial or race, whichever was chosen on the console (default trial); used by `laksa-car.service` |
| `dryrun_bringup.sh console` | restart only the console, bound to the current network |
| `dryrun_bringup.sh stop` | stop every process it started |
| `dryrun_bringup.sh status` | show what's running |

In either mode the car stays braked until an operator starts driving.

The launcher also starts Nav2: `controller_server`, `planner_server`, `behavior_server`, `bt_navigator` and the lifecycle manager. They use `laksa_bringup/config/nav2_ackermann.yaml` plus `laksa_learned_driver/config/nav2_field_overrides.yaml`. The override switches route following to **Regulated Pure Pursuit** with reversing, on **Reeds-Shepp** plans (three-point turns). In an isolated simulation of an 8 x 6 m room it reached 4/4 goals with no collisions, where the production MPPI follower dithered (0/4) and forward-only planning could not turn the car around. Controller and BackUp output go to `/laksa/nav_cmd_vel`, the supervisor's navigation input. `LAKSA_NAV=0` skips Nav2. Stopping the whole stack takes 60–90 s.

## Boot services

Unit files are in `firmware/esp32-s3/jetson/systemd/`, installed to `/etc/systemd/system/`:

| Service | What it does |
|---|---|
| `laksa-car.service` | Runs as `samyak` after the network is up. Waits 25 s for USB devices and the micro-ROS agent to settle, then runs `dryrun_bringup.sh trial`. `stop` runs `dryrun_bringup.sh stop`. |
| `laksa-network-watch.service` | Runs as root. Starts and stops the hotspot and restarts `laksa-car` when the console's network changes (see [Console and operation](07_console_and_operation.md#networking-at-the-field)). |

Restart the stack:

```bash
sudo systemctl restart laksa-car
```

## Sessions and recording

Every `start` or `trial` creates `~/laksa_sessions/<YYYYMMDDTHHMMSS>/`. Nothing in it is ever overwritten:

| File | Content |
|---|---|
| `session.txt` | mode, actuation, caps and start time |
| `<process>.log` | each process's output |
| `decisions.csv` | the learned driver's per-scan decision log ([Runtime safety](05_runtime_safety.md#decision-log)) |
| `rtabmap.db` | that session's map database |
| `bag/` | rosbag of scans, commands, brake, driver status, supervisor state, perception JSON, odometry, VESC and ESP32 state, `/joy`, TF and `/map`. No camera video, to keep it small. |

`~/laksa_run/latest` points at the newest session. The ZED obstacle point cloud isn't recorded; `decisions.csv` already has the distances.

## Deploying code changes

The Jetson's `~/laksa_ws` symlinks into `~/src/Project_LAKSA`, a git checkout. To deploy from a Windows PC, copy the changed files over SSH, reset any file modes the copy changed, rebuild, test and restart:

```bash
tar -cf - <changed files> | ssh samyak@<jetson> 'cd ~/src/Project_LAKSA/firmware/esp32-s3/jetson && tar -xf -'
```

```bash
ssh samyak@<jetson> 'cd ~/src/Project_LAKSA && git diff --summary | awk "/mode change/{print substr(\$3,4), \$NF}" | while read m p; do chmod $m "$p"; done'
```

```bash
ssh samyak@<jetson> 'cd ~/laksa_ws && source /opt/ros/humble/setup.bash && source ~/zed_ws/install/setup.bash && colcon build --symlink-install --packages-select <packages>'
```

```bash
ssh samyak@<jetson> 'sudo systemctl restart laksa-car'
```

**Warning:** don't "undo mode changes" with `git checkout -- <file>`. That restores the file's **contents** too, and silently throws away the change you just deployed. This happened once with `zed_base_pose_adapter.py`; see [Performance](09_performance_optimization.md#deploy-regression-found-on-the-way). Use `chmod` as above.

After deploying, run each changed package's tests on the Jetson:

```bash
cd ~/src/Project_LAKSA/firmware/esp32-s3/jetson/<package> && python3 -m pytest -q test
```

Last run: learned driver 38, LiDAR 14, bringup 35, mapping 38, all passing.

## Bench tools

These talk to the ESP32 **directly, bypassing the supervisor**. They are for wheels-up bench tests only, with the supervisor stopped.

| Tool | Test | Aborts on |
|---|---|---|
| `setup/steering_bench.py` | sweeps steering about 9° left and 7° right, with traction held at 0 | any wheel motion, VESC fault, stale telemetry, another `/laksa/command` publisher |
| `setup/traction_bench.py` | slow speed steps (at most 0.30 m/s) with active braking between them; reports measured eRPM for calibration | more than 2500 eRPM, motor current above 8 A, sustained rotation against the command, VESC fault, stale telemetry, another publisher |

If either tool dies, the ESP32's 500 ms watchdog stops and centres the wheels.
