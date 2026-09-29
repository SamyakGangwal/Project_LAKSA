# Known issues and next steps

[Index](README.md) · Previous: [Field tests and findings](10_field_tests_and_findings.md) · Next: [Change log](12_change_log.md)

## Unreliable start from standstill (and reverse on the floor)

**Update after the stage-1 bench:** reverse **does** work at the motor (−0.25 m/s ran at −1,052 eRPM on 1.9 A). The real problem is starting from standstill: sometimes the motor starts within 0.3–0.9 s, sometimes it sits stuck at 10–13 A. See [Stage-1 bench](10_field_tests_and_findings.md#stage-1-bench). The fix belongs in the VESC's configuration: enable hall sensors if the motor has them, otherwise tune the sensorless start and open-loop settings. The VESC is only reachable through the ESP32, which doesn't pass configuration through, so connect the VESC's own USB to the Jetson for a read-only check first.

## Reverse does not move the car (original report)

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

The console now has **PLAN ROUTE** and **HOLD TO GO**, and the launcher runs Nav2 (see [Console and operation](07_console_and_operation.md#laksa-console)). The console-to-planner path was verified end to end on the car. But the current bench spot is too cramped to produce a route: the car sits 3 cm from a wall, its start cell is "inscribed" in the costmap, only about 180 cells of the small map are low-cost, and 62% is still unknown. Next: test PLAN, then HOLD TO GO in dry-run, in a larger mapped area.

Remaining checks for route driving:
- **CPU budget.** Nav2 adds load; see [Performance](09_performance_optimization.md).
- **Earlier planner forensics.** In this repo, SmacPlannerHybrid returned invalid paths in 3 of 5 test cases.
- **Speed cap vs. what the motor can do.** MPPI's `vx_max` and the supervisor's navigation cap (620 eRPM) are about 0.15 m/s. But the bench shows the motor runs reliably only from about 0.20 m/s (850 eRPM), which matches MPPI's 0.217 m/s deadband. Route following needs the navigation cap raised to about 0.22 m/s, together with a reliable start, before it can move the car smoothly.

## Model limits

| Limit | Next step |
|---|---|
| Fails at 3.0 m/s with randomised conditions on the competition course (0/3) | more DAgger rounds at high caps, or a stronger expert at speed |
| Trained only on obstacle-free corridors 0.9–2.2 m wide; rooms and outdoors are out of distribution | add open areas, obstacles and clutter to the procedural worlds, then retrain |
| Never learned to stop or reverse; that's the rule layers' job | fine, by design |
| Sim-only data | use the session bags plus the expert labels offline to fine-tune on real scans |
| Obstacle Course replica: the expert fails about 3 of 8 layouts at hoop 3 on the right loop | the right-steering limit (0.288 rad, 1.09 m radius, against 0.523 rad left) makes some legal hoop positions undrivable; more right steering travel on the car would fix it. In training, either keep hoop 3 near the natural line or give the expert a tracking controller that doesn't cut corners |
| v3/v4 not deployed | evaluate on the held-out Speed Course with `evaluate.py`, then deploy |

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
