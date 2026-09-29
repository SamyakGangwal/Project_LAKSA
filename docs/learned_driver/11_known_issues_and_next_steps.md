# Known issues and next steps

[Index](README.md) · Previous: [Field tests and findings](10_field_tests_and_findings.md) · Next: [Change log](12_change_log.md)

## Reverse does not move the car

The driver commands reverse and the supervisor passes it on, but the wheels don't turn backwards. Reverse has never been bench-tested; the traction bench aborted on current before reaching it.

One wheels-up test tells the causes apart. Reverse at 0.12 m/s, then 0.20 m/s, braking in between, while recording what the ESP32 reports:

| ESP32 reports | Meaning |
|---|---|
| requested eRPM negative, active eRPM 0 | the ESP32 firmware refuses reverse; check its direction-change handling |
| active eRPM negative, motor current about 0 | the **VESC** is configured to refuse reverse (a common VESC setting) |
| active negative, current rising, wheel stuck | breakaway: too little speed or current to start moving |

Until this is fixed, the recovery ends in `BLOCKED`, which is safe but limited.

## Motor breakaway current

The drive needs 6–10 A to start from rest but only about 3 A to keep moving. At the low caps used so far, the first second of each start can stall or jerk. Check in VESC Tool:
- motor current limits;
- sensorless start and minimum-ERPM settings;
- whether hall sensors are fitted but not configured.

## Route planning from the console

START and END markers are published (`/laksa/console/start`, `/laksa/console/goal`), but **nothing plans or drives a route** between them yet.

The next step is to connect the markers to Nav2 (Smac Hybrid-A* planner and a controller) through the supervisor's existing navigation mode.

Things to check first:
- **CPU budget.** Nav2 adds load; see [Performance](09_performance_optimization.md).
- **Earlier planner forensics.** In this repo, SmacPlannerHybrid returned invalid paths in 3 of 5 test cases.
- **MPPI deadband vs. speed cap.** The MPPI config caps speed at 0.15 m/s, but its deadband critic penalises anything below 0.217 m/s.

## Model limits

| Limit | Next step |
|---|---|
| Fails at 3.0 m/s with randomised conditions on the competition course (0/3) | more DAgger rounds at high caps, or a stronger expert at speed |
| Trained only on obstacle-free corridors 0.9–2.2 m wide; rooms and outdoors are out of distribution | add open areas, obstacles and clutter to the procedural worlds, then retrain |
| Never learned to stop or reverse; that's the rule layers' job | fine, by design |
| Sim-only data | use the session bags plus the expert labels offline to fine-tune on real scans |

## Awaiting real-world confirmation

- **Slope handling.** Unit-tested only; it needs a drive on a real slope.
- **Live home-to-hotspot switch** with the new whole-stack restart. At the field this bug showed `UNKNOWN` in the console; it needs a re-test.
- **Low-light behaviour** with the stricter depth confidence and the near-field camera cutoff.

## Calibration and interfaces

- **eRPM constants disagree.** The supervisor converts eRPM to m/s with 2 pole pairs, a 11.82 gear ratio and a 0.109 m wheel. The ESP32's Kconfig defaults are 7, 1.0 and 0.100 m, and its real `sdkconfig` is gitignored. `traction_bench.py` measured 0.10 m/s → about 476 eRPM on this car. Use it to calibrate both sides together.
- **Firmware source vs. flashed firmware.** The ESP32 runs firmware with the handoff brake and telemetry fields, which this branch's messages now match. But the ESP32 **source** on this branch is still the older production version, which never sets those fields. If anyone flashes it, the supervisor would see telemetry as permanently stale and hold the brake: safe, but the car wouldn't drive. Bring the matching firmware source into the repo.
- **Micro-ROS publisher limit.** `app-colcon.meta` allows 4 publishers, but the production firmware creates 5.

## Security

- The ESP32 accepts commands that **bypass `drive_supervisor`**: the `/laksa/set_drive_command` service and an unauthenticated Wi-Fi HTTP motor API (`/api/motor`, `/api/steering`). Disable them for field use.
- Wi-Fi credentials are committed in `firmware/esp32-s3/src/esp32_config.h` and are in the repository history. Rotate them, and move them out of source.
- On the Jetson:
  - remove the setup-period passwordless sudo rule (`/etc/sudoers.d/90-samyak-laksa`);
  - change the account password, which was shared during setup;
  - treat the console link as a key: whoever has it on the hotspot can drive.
- The console binds only to localhost or the hotspot. Keep it that way; don't bind it to `0.0.0.0`.

## Pre-existing repository issues

Found in the initial review; not caused by this work:

- `laksa_mapping` declares a dependency on `laksa_lab`, which doesn't exist.
- Several systemd units reference scripts that aren't on this branch.
- macOS Finder duplicate files (`app 2.js`, `style 2.css`, …) are installed.
- The `esp32_BNO08x` submodule has no `.gitmodules` entry.
- `lidar_guard` validates the raw scan, not the self-filtered `/scan`. The learned driver removes the car's own body itself, but other consumers (costmaps, RTAB-Map) may see it.

## Performance headroom

- The ZED SDK uses about 87% CPU. It could turn object detection off while the car isn't driving, or use a lower resolution.
- `sllidar_node` at about 26% hasn't been looked at.
- `drive_supervisor` at about 34% is mostly rclpy overhead. Moving it to C++ would cut this.
