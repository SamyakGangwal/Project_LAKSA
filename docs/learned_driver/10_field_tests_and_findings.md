# Field tests and findings

[Index](README.md) · Previous: [Performance optimization](09_performance_optimization.md) · Next: [Known issues and next steps](11_known_issues_and_next_steps.md)

All tests were on 28 Sep 2026, with a person next to the car throughout.

## Read-only hardware check

- Listened only to the ESP32's micro-ROS traffic through the existing agent. The serial port was never opened.
- All four message definitions on the ESP32 match this branch, so **no reflash was needed**.
- IMU data arrived at 39 Hz and state at 9.5 Hz.
- At first the VESC reported nothing (0 V). It was unpowered. Once the car ran from its battery it reported 16.4 V with no faults.

## Steering bench

Car on a stand, traction held at 0, run twice:

| Step | Commanded | Measured |
|---|---|---|
| Left | +0.150 rad | +0.147 rad (about 8.4°) |
| Centre | 0 | 0.000 |
| Right | −0.120 rad | −0.123 rad (about 7.0°) |
| Centre | 0 | 0.000 |

- Each target was reached within about 0.25 s; the small offsets are the servo's 1° steps.
- The wheels turned left first, then right, as intended, so **the steering sign is correct**.
- The motor stayed within ±3 eRPM of zero.

## Traction bench

Drive wheels off the ground:

| Command | Result |
|---|---|
| Forward 0.10 m/s | Stuck at about 0 eRPM for about 1 s while current rose to about **6 A**. Then it broke free at about 476 eRPM (**0.115 m/s**, 15% over target) on about 3 A. |
| Forward 0.20 m/s | Current reached **9.7 A** before breakaway, so the 8 A bench limit aborted the run |
| Reverse | not reached (the run had already aborted) |

**Finding:** the drivetrain needs **6–10 A to start** but only about 3 A to keep turning. The cause is either high static friction or the VESC's sensorless start struggling at low speed. Speed tracking is fine once the wheels are moving.

## Stage-1 bench

Wheels up, on battery, 28 Sep evening, with the updated `traction_bench.py`: battery guard at 14.4 V, a 12 A start-up allowance for 2.5 s then 8 A, and a CSV of every sample.

| Command | Result |
|---|---|
| 0.10 m/s (414 eRPM) | Stalls and restarts repeatedly while current winds up to 10 A. Unusable. |
| 0.20 m/s (828 eRPM) | Wheels spun: 849 eRPM, reported 0.205 m/s, on 1.8 A. Started after 0.87 s. Spin-down after braking 0.32 s. In an earlier run it first stuck at 10.6 A for 1.1 s. |
| 0.25 m/s (1,036 eRPM) | Once: 972 eRPM (0.235 m/s) on 2.1 A, started after 0.53 s. Next run: stuck at start and aborted at 12.5 A. |
| −0.25 m/s | **Reverse works**: −1,052 eRPM (−0.254 m/s) on 1.9 A, started after 0.32 s, spin-down 0.44 s. |

Findings:
- The ESP32 converts 0.10 m/s → 414 eRPM (4,140 eRPM per m/s).
- VESC telemetry updates about every 200 ms, which adds to the reaction delay.
- **Starting from standstill is unreliable:** sometimes 0.3–0.9 s, sometimes stuck at 10–13 A. Once turning, the drive needs only about 2 A. This points to the VESC's sensorless start, not the drivetrain, and it's the likely reason reverse "did nothing" on the floor.
- A whistle during stuck starts is the VESC's switching noise with the rotor not turning.

## First autonomous run

Indoors, on carpet, trial mode (0.15 m/s cap), started with `laksa_operator run`:

- At 3.0 s the operator's A-hold engaged `LIDAR_CRUISE`, and the learned driver drove for **7.4 s**.
- At 10.4 s the clearance governor found a white box **0.25 m ahead of the bumper** in the chosen path and stopped the car. The supervisor dropped autonomy and held the brake.
- A second run, still pointed at the box, was blocked immediately (0.21 m).

This was the first successful end-to-end run on the real car: the network steered, the governor stopped the car, the supervisor handed control back, and the operator's stop worked.

## LiDAR rear visibility

This was checked when reverse didn't move the car. The LiDAR sits above the camera, and the car's body doesn't block its rear view:

| Sector | Valid returns | Nearest |
|---|---|---|
| Front | 292 / 300 | 0.35 m |
| Left / right | 294 / 299 | 0.43 / 0.60 m |
| Rear-left / rear-right | 134 / 150 | 1.30 / 1.09 m |
| Reverse path behind the bumper | 120 points | 0.79 m, clear |

The driver saw the obstacle, found the rear clear, and **commanded reverse twice for 2.5 s**. The supervisor passed those commands through, yet the car didn't move. The problem therefore lies between the ESP32 and the wheels. See [Known issues](11_known_issues_and_next_steps.md#unreliable-start-from-standstill-and-reverse-on-the-floor).

## Outdoor field test

Evening, outdoors, getting dark, operated from a phone over the car's hotspot.

**What was reported on the spot:**
- the steering stuttered;
- the car couldn't get up a slope (the run was blocked);
- it went "fast" and not straight;
- it stopped as blocked with nothing in front;
- the console showed Mode and Autonomy as `UNKNOWN` until a power cycle;
- set START and set END did nothing.

**What the logs showed afterwards:**

| Finding | Evidence | Cause |
|---|---|---|
| Mapping worked | about 100 RTAB-Map nodes | the "mapping not running" message was the network-switch discovery bug |
| Two good long drives | 38 s and 86 s of `LEARNED_DRIVING`, ended by the operator or the 2-minute limit | — |
| Phantom blocks | two runs ended "Path blocked 0.00 m ahead of the bumper" after block → reverse → block cycles | the **camera**: noisy near-range depth in the dark, or sloped ground read as obstacle height. The LiDAR ignores that zone. |
| Camera depth failures | ZED "point cloud retrieve error" | darkness and overload |
| Odometry jumps | ZED VIO jumped 0.25–0.92 m in about 0.1 s; the supervisor rejected them | visual tracking in dark, featureless areas |
| Overload | RTAB-Map 1.5–3.6 s per update against a 0.7 s budget | too much running at once |

**Fixes deployed that night:**

| Issue | Fix | Verified |
|---|---|---|
| Phantom camera blocks | ignore camera points within 13 cm of the bumper; depth confidence 95 → 50 | tested on the bench |
| Slope read as a wall | RANSAC ground plane; heights measured against it; LiDAR hits on rising ground dropped | unit tests only; **needs a real slope** |
| Stutter | steering low-pass filter plus rate limit | tested |
| Overload | ZED 15 fps, lighter RTAB-Map | GPU 97% → 46%, RTAB-Map 0.07 s per update |
| VIO jumps | EKF Mahalanobis rejection threshold 3.0 on the ZED pose | deployed |
| "Why did it stop?" | per-scan `decisions.csv` | working |
| Lost logs and maps | a session folder per start; bag recording | working |
| Network-switch bug | the network watcher restarts the whole stack on a change | **untested** in the field |

## Explore run (1 Oct)

Trial & explore at 0.6 m/s, about 10 minutes of driving (567 s of `decisions.csv`), early on 1 Oct. The run ended by pulling the battery.

**What was reported:**
- the car slowed and stopped far from obstacles instead of driving up to them;
- after HOLD / EXPLORE it took a long time to start moving;
- near a wall it stopped and didn't back up;
- PLAN ROUTE said the Nav2 planner was down.

**What the logs showed:**

| Finding | Evidence | Cause |
|---|---|---|
| Commands the motor can't run | 985 of 2,107 commands were between 0 and 0.25 m/s, all while `AVOIDING` | the governor slowed smoothly toward its stop point, but the drive stalls below about 0.2 m/s, so the car stopped well before it |
| Parked at a wall for 3 minutes | 329–505 s: best arc 0.257 m free, command 0.025 m/s | 0.257 m was just above the 0.25 m stop margin, so the path never counted as blocked and the recovery never started |
| Reverse didn't move the car | `RECOVERY_REVERSE` at −0.12 m/s | about 500 eRPM, far below the stall speed |
| Camera sees the wall closer than the LiDAR | camera 0.26 m vs LiDAR 0.62 m free at the wall | low or angled parts of the wall below the LiDAR plane, or camera depth error; not resolved |
| Slow start | the console held A for 3.5 s, the supervisor needed 3 s | deliberate engage delay, longer than needed |
| "Planner down" | no Nav2 crash in any log (every `IS DOWN` was at shutdown); planning works on the car | the console failed at once when the planner wasn't answering yet, for example in the ~15 s after a (re)start |
| Logs from the run overwritten | the next boot used the same session folder | no RTC battery: every boot starts at the same clock time |

The run's rosbag was cut by the power loss and couldn't be recovered; the findings come from `decisions.csv`, which survived.

**Fixes (1 Oct, not yet driven):** see [Runtime safety](05_runtime_safety.md#clearance-governor) and [Console](07_console_and_operation.md#controls).

| Issue | Fix |
|---|---|
| Crawling and early stops | forward speed is ≥ 0.30 m/s or zero; the car stops only when the path has 0.30 m (1 ft) or less |
| No recovery at the wall | blocked at 0.30 m, for the governor and for avoidance alike, so the recovery starts |
| Reverse | 0.30 m/s for 1.2 s |
| Detect far, keep moving | explore steers around obstacles from 1.0 m (was 0.6 m) |
| Slow start | console 1.5 s, supervisor 1 s |
| Planner not ready | PLAN ROUTE waits up to 30 s for Nav2 |
| Lost logs | session folders carry the boot id, under `~/laksa_logs` |

### Second session (1 Oct, evening)

With the fixes above installed, on the bench floor at 0.6 m/s:

- **First hold drove well:** no command below 0.3 m/s, stops at about 0.3 m, and the car **reversed** for the first time (−1,150 eRPM measured). It gave up (`BLOCKED`) after four recoveries in one corner, where the camera read a constant 0.154 m.
- **Later holds at 0.8 m/s didn't move.** The supervisor sent 0.8 m/s, but the ESP32 rejected 94% of those commands and held the brake: its firmware has a speed limit between 0.6 and 0.8 m/s ([Known issues](11_known_issues_and_next_steps.md#esp32-firmware-speed-limit)). Explore is now capped at 0.6 m/s and the console shows whether the ESP32 accepts commands.
- **The map froze** a few seconds after start because `rgbd_sync` stopped receiving camera frames; routes then failed ("no drivable route"). Restarting it fixed the map; a watchdog now does that automatically ([Known issues](11_known_issues_and_next_steps.md#mapping-input-stalls)).
