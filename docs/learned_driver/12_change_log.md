# Change log

[Index](README.md) · Previous: [Known issues and next steps](11_known_issues_and_next_steps.md)

Every file this branch adds or changes compared with `production/laksa-mainline` at `1f0db4f`. Paths are relative to `firmware/esp32-s3/` unless they start with `docs/`.

## New package: laksa_learned_driver

`jetson/laksa_learned_driver/`

| File | Purpose |
|---|---|
| `laksa_learned_driver/scan_features.py` | LiDAR input contract shared by the simulator and the car (120 min-range bins) |
| `laksa_learned_driver/scan_adapter.py` | Raw scan → vehicle-frame beams; mount yaw; own-body removal |
| `laksa_learned_driver/policy.py` | NumPy inference for the exported network; model-format checks |
| `laksa_learned_driver/safety.py` | Clearance governor (swept-arc free distance, stopping-distance speed cap) |
| `laksa_learned_driver/recovery.py` | Reverse-away recovery state machine |
| `laksa_learned_driver/smoothing.py` | Steering low-pass filter and rate limit |
| `laksa_learned_driver/perception.py` | Camera obstacle logic: cloud filtering, detection footprints, person rule, RANSAC ground plane |
| `laksa_learned_driver/driver_node.py` | The `learned_driver` ROS node that combines all of the above; decision log |
| `laksa_learned_driver/zed_perception_node.py` | The `zed_perception` ROS node |
| `laksa_learned_driver/operator_cli.py` | `laksa_operator` terminal deadman (run, stop, rearm, status) |
| `laksa_learned_driver/console_node.py`, `console_page.html` | LAKSA Console web app |
| `laksa_learned_driver/policy_probe.py` | Read-only probe: prints what the policy would command on live scans; publishes nothing |
| `config/learned_driver.yaml` | Driver parameters: caps, mount, governor, recovery |
| `config/zed_perception.yaml` | ZED profile: 15 fps, NEURAL_LIGHT, object detection |
| `config/ekf_field.yaml` | Field EKF: ZED base pose + VESC speed, with VIO-jump rejection |
| `config/rtabmap_field_overrides.yaml` | Lighter RTAB-Map for the Orin Nano |
| `launch/learned_cruise.launch.py` | Launch the driver alone, next to the production manual-control stack |
| `models/laksa_tinylidarnet_v2.npz`, `.report.json` | The shipped model and its training report |
| `training/*.py` | Simulator wrapper, tracks, expert, DART/DAgger training, evaluation, smoke test |
| `test/test_learned_driver.py` | 31 tests: features, adapter, governor, recovery, perception, smoothing, model contracts, topic authority |
| `README.md`, `package.xml`, `setup.py`, `setup.cfg`, `resource/` | Package metadata |

## Changes to existing packages

| File | Change | Why |
|---|---|---|
| `extra_ros_packages/laksa_interfaces/msg/DriveCommand.msg` | adds `bool brake` | match the ESP32's running firmware (autonomy-handoff fields) |
| `extra_ros_packages/laksa_interfaces/msg/VescState.msg` | adds `brake_active`, `telemetry_sequence`, `telemetry_age_ms` | same; the supervisor already read these fields |
| `jetson/laksa_description/urdf/laksa_visualization.urdf` | root at `base_footprint`; ZED joint inverted | `base_footprint` had two TF parents once the EKF published `odom → base_footprint` |
| `jetson/laksa_bringup/scripts/drive_supervisor_node.py` | LiDAR and ZED-cloud freshness subscriptions take raw bytes | CPU ([Performance](09_performance_optimization.md)) |
| `jetson/laksa_lidar/laksa_lidar/scan_quality.py` | scan statistics vectorised with NumPy | CPU |
| `jetson/laksa_lidar/laksa_lidar/lidar_guard.py` | TF from `/tf_static` only | CPU |
| `jetson/laksa_lidar/package.xml` | adds `python3-numpy` | new dependency of `scan_quality.py` |
| `jetson/laksa_mapping/laksa_mapping/zed_base_pose_adapter.py` | TF from `/tf_static` only; camera-to-base transform cached | CPU |

## Setup, services and patches

| File | Purpose |
|---|---|
| `jetson/setup/01_system_setup.sh` | One-time system install (sudo): ROS 2 Humble and dependencies |
| `jetson/setup/02_build_workspaces.sh` | Build the third-party and LAKSA workspaces (no sudo) |
| `jetson/setup/dryrun_bringup.sh` | Launcher: start, trial, console, stop, status; session folders; console token |
| `jetson/setup/laksa_network_watch.sh` | Hotspot and home Wi-Fi switching; console rebinding |
| `jetson/setup/steering_bench.py`, `traction_bench.py` | Wheels-up bench tests that talk to the ESP32 directly |
| `jetson/systemd/laksa-car.service` | Start the stack at boot (trial mode, braked until an operator acts) |
| `jetson/systemd/laksa-network-watch.service` | Run the network watcher at boot |
| `jetson/patches/rf2o-laser-odometry-laksa.patch` | RF2O fixes: correct twist sign for the π-yaw LiDAR mount, publish only valid estimates, realistic covariance instead of all-zero |

## Documentation

`docs/learned_driver/`: this documentation set.

## Not committed

The `scratch/` folder stays local. It holds the F1TENTH Gym clones, generated maps, training and evaluation logs, the field-test logs, the archived PPO attempt (`scratch/archive_ppo/`) and the v1 model (`scratch/archive_v1_model/`).
