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

The driver saw the obstacle, found the rear clear, and **commanded reverse twice for 2.5 s**. The supervisor passed those commands through, yet the car didn't move. The problem therefore lies between the ESP32 and the wheels. See [Known issues](11_known_issues_and_next_steps.md#reverse-does-not-move-the-car).

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
