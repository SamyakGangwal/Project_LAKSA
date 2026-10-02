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
| `laksa_learned_driver/safety.py` | Clearance governor (swept-arc free distance, stopping-distance speed cap) and the arc search that steers around obstacles |
| `laksa_learned_driver/recovery.py` | Reverse-away recovery state machine |
| `laksa_learned_driver/smoothing.py` | Steering low-pass filter and rate limit |
| `laksa_learned_driver/perception.py` | Camera obstacle logic: cloud filtering, detection footprints, person rule, RANSAC ground plane |
| `laksa_learned_driver/driver_node.py` | The `learned_driver` ROS node that combines all of the above; decision log |
| `laksa_learned_driver/zed_perception_node.py` | The `zed_perception` ROS node |
| `laksa_learned_driver/race.py`, `race_node.py`, `signals.py` | Race mode: arm (with the chosen profile), green start, red stop, Speed course / Obstacle course |
| `laksa_learned_driver/profiles.py` | Drive profile per mode (Obstacle, Speed, Trial & explore), clamped; applied live on `/laksa/drive_profile` |
| `laksa_learned_driver/hold_latch.py` | KarSha's deadman re-arm rule for HOLD/GO after a heartbeat loss |
| `laksa_learned_driver/operator_cli.py` | `laksa_operator` terminal deadman (run, stop, rearm, status) |
| `laksa_learned_driver/console_node.py`, `console_page.html` | LAKSA Console web app: drive-mode tabs with live settings, race and explore panels, telemetry, SAVE MAP, PLAN ROUTE and HOLD TO GO through Nav2; phone layout with STOP/REARM pinned |
| `laksa_learned_driver/policy_probe.py` | Read-only probe: prints what the policy would command on live scans; publishes nothing |
| `config/learned_driver.yaml` | Driver parameters: caps, mount, governor, recovery |
| `config/zed_perception.yaml` | ZED profile: 15 fps, NEURAL_LIGHT, object detection |
| `config/ekf_field.yaml` | Field EKF: ZED base pose + VESC speed, with VIO-jump rejection |
| `config/rtabmap_field_overrides.yaml` | Lighter RTAB-Map for the Orin Nano |
| `config/nav2_field_overrides.yaml` | Nav2 on top of the production config: Regulated Pure Pursuit at 0.22 m/s with reversing on Reeds-Shepp plans |
| `launch/learned_cruise.launch.py` | Launch the driver alone, next to the production manual-control stack |
| `models/laksa_tinylidarnet_v5.npz`, `.report.json` | The default model (replica-, course- and obstacle-trained) and its training report |
| `models/laksa_tinylidarnet_v2.npz`, `v3.npz`, `v4.npz` + reports | The first shipped model and the intermediate candidates |
| `training/*.py` | Simulator wrapper, tracks (plain, obstacles, 2026 course-style, Obstacle Course replica, field maps), expert, DAgger training, multi-model evaluation, smoke test |
| `training/field_maps/` | Maps saved from the console for training (not created until the first map is copied in) |
| `training/test_tracks.py` | Track geometry tests: 2-D clearance limits for field maps (`python -m unittest test_tracks` in `training/`) |
| `training/courses/` | Obstacle Course replica builder (from the PDF) and its map, route and features |
| `test/test_learned_driver.py` | 54 tests: features, adapter, governor, obstacle avoidance, recovery, perception, smoothing, profiles, race manager, signals, model contracts, topic authority |
| `test/test_hold_latch.py` | KarSha's 8 tests for the hold re-arm rule |
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
| `jetson/setup/dryrun_bringup.sh` | Launcher: start, trial, race, auto, console, stop, status; session folders; console token; KarSha's `LAKSA_*` overrides |
| `jetson/setup/laksa_network_watch.sh` | Hotspot and home Wi-Fi switching; console rebinding |
| `jetson/setup/steering_bench.py`, `traction_bench.py` | Wheels-up bench tests that talk to the ESP32 directly |
| `jetson/systemd/laksa-car.service` | Start the stack at boot from `~/laksa/current` (trial mode, braked until an operator acts), after `laksa-deploy` |
| `jetson/systemd/laksa-network-watch.service` | Run the network watcher at boot, from `~/laksa/current` |
| `jetson/systemd/laksa-deploy.service` | At boot, before the car stack: install the newest car package, move and prune logs |
| `jetson/release/laksa_release.py` | PC tool: build a car package from the committed HEAD, push it to the car, show its status |
| `jetson/release/laksa_deploy.sh` | Jetson deployer: `boot`, `status`, `rollback`, `bootstrap`; logs under `~/laksa_logs` |
| `jetson/release/install.sh` | Builds one release's own workspace and runs the unit tests before it is switched on |
| `jetson/setup/tonight/*`, `jetson/config/navigate_plan_once.xml`, `nav2_tonight_overrides.yaml` | KarSha's bench, route and battery tools, merged from `tonight/route-0929-full` on 1 Oct |
| `jetson/patches/rf2o-laser-odometry-laksa.patch` | RF2O fixes: correct twist sign for the π-yaw LiDAR mount, publish only valid estimates, realistic covariance instead of all-zero |

## 1 Oct changes

After the 1 Oct explore run ([Field tests](10_field_tests_and_findings.md#explore-run-1-oct)):

| File | Change |
|---|---|
| `laksa_learned_driver/safety.py` | `min_speed_mps` floor: forward speed ≥ 0.30 m/s or zero; `blocked_distance` (0.30 m with the car's settings) shared by the governor and avoidance |
| `laksa_learned_driver/driver_node.py`, `config/learned_driver.yaml` | `min_drive_speed_mps` 0.30, `stop_margin_m` 0.18, reverse 0.30 m/s for 1.2 s |
| `laksa_learned_driver/profiles.py`, `console_page.html` | explore avoid distance 1.0 m; speed minimum 0.3 m/s |
| `laksa_bringup/config/drive_supervisor.yaml`, `drive_supervisor_node.py` | A-hold 3 s → 1 s; the startup message shows the real value |
| `laksa_learned_driver/console_node.py`, `console_page.html` | HOLD presses A for 1.5 s (was 3.5 s); PLAN ROUTE waits up to 30 s for Nav2; **Clear start/end** button |
| `setup/dryrun_bringup.sh` | sessions in `~/laksa_logs/sessions/<time>_boot<id>`; uses a release's own workspace |
| `setup/laksa_network_watch.sh` | restarts the stack from `~/laksa/current` when installed |
| `laksa_learned_driver/test/test_learned_driver.py` | tests for the speed floor, the 1 ft stop and the 1 Oct wall case |

## Documentation

`docs/learned_driver/`: this documentation set.

## Scratch artifacts

`scratch/` holds working artifacts, kept for reference:

| Path | Content |
|---|---|
| `scratch/archive_ppo/` | the original PPO attempt and its TensorBoard runs |
| `scratch/archive_v1_model/`, `scratch/candidate_v2/` | the v1 model and the v2 training output |
| `scratch/learned_driver_maps/`, `scratch/learned_driver_runs/` | generated tracks and per-worker run data |
| `scratch/speed_course/`, `scratch/*_log.txt` | evaluation inputs and training and evaluation logs |
| `scratch/field_20260928/` | logs from the outdoor field test |

The two local F1TENTH Gym clones (`scratch/f1tenth_gym*`) are **not** committed and are listed in `.gitignore`. Clone `v1.0.0` from upstream instead ([Training pipeline](04_training_pipeline.md#how-to-retrain)).
