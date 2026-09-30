# LAKSA learned driver

An imitation-learned LiDAR driving policy for the LAKSA car. A TinyLidarNet-style 1-D CNN
maps one LiDAR scan and the operator speed cap to a steering angle and a target speed. It
needs no map, localization or TF, and it runs in NumPy on the Jetson (no PyTorch/ONNX).

## Why imitation learning instead of PPO

End-to-end RL (the earlier PPO attempt) needs millions of steps, heavy reward shaping and
transfers poorly to a real car. Here a privileged simulator expert (smoothed raceline +
curvature speed profile + pure pursuit, which knows the true pose and the track) demonstrates
driving and the network copies it from LiDAR alone:

1. **DART** round: the expert drives with injected steering noise, so the data contains
   recoveries from off-line states.
2. **DAgger** rounds: the current student drives (mixed with the expert at a decaying rate)
   while the expert labels every visited state.

Training takes minutes on a laptop CPU. Sim-to-real randomization covers friction, LiDAR
noise and dropouts, steering bias, speed gain and one extra control step of latency.

## Models

The default is `models/laksa_tinylidarnet_v5.npz`: trained with obstacles, 2026 course-style
sections and the Obstacle Course replica. On identical held-out episodes (0.5-3.0 m/s) it
completes 84% overall, against 70% for v4 and 42% for v2, and it completes the Speed Course
12/12. The full table is in `docs/learned_driver/04_training_pipeline.md`.

## Results of the first model (simulation, `models/laksa_tinylidarnet_v2.npz`)

Trained in ~24 min on a laptop CPU (5 rounds, 168k samples, 12 workers). Held-out evaluation,
2 laps per episode; "randomized" varies friction, LiDAR noise/dropout, latency, steering bias and
speed gain, and starts from random poses on the random tracks.

| Speed cap | Competition course (never trained on), clean + 3 randomized | 10 unseen random tracks, 40 runs |
|---|---|---|
| 0.24-2.0 m/s | 4/4 at every cap, lap time within ~1% of the expert | 40/40 at every cap |
| 3.0 m/s | clean lap 37.2 s (expert 39.2 s); **randomized 0/3**, crashes ~10% into the lap | 40/40 |

The 3 m/s randomized competition-course failure was v2's known gap. v5 completes the Speed
Course at every cap up to 3 m/s.

## Contracts

| Item | Value |
|---|---|
| Input | 120 bins over the front 180 deg, min range per 1.5 deg bin, clipped to 10 m, measured from the LiDAR at `base_footprint` x = 0.31542 m |
| Conditioning | speed cap, trained on 0.25-3.0 m/s |
| Output | road-wheel steering (left <= 0.523 rad, right <= 0.288 rad), speed <= cap |
| Rate | one command per scan (~12.5 Hz) |
| Vehicle | wheelbase 0.324 m, footprint front 0.419 / rear 0.149 / half-width 0.148 m (+0.02 padding) |

`base_footprint` is assumed to be the rear axle. The competition Speed Course is **never** used
for training (`mission.yaml: training_allowed: false`); it is only a held-out evaluation.

## Train and evaluate (development PC)

Requirements: Python 3.10+, `numpy scipy opencv-python torch pyyaml` and F1TENTH Gym
`v1.0.0` (`git clone -b v1.0.0 https://github.com/f1tenth/f1tenth_gym`, then
`pip install -e f1tenth_gym --no-deps` plus `gymnasium pygame requests shapely
"yamldataclassconfig<2"`).

```bash
cd firmware/esp32-s3/jetson/laksa_learned_driver/training
python smoke_test.py      # expert drives three random tracks
python train.py --output ../models/laksa_tinylidarnet_v6.npz   # new name: the default is v5's file
python evaluate.py --models v5=../models/laksa_tinylidarnet_v5.npz v6=../models/laksa_tinylidarnet_v6.npz
```

To train on places the real car explored, save a map from the console (Trial & explore,
SAVE MAP), copy `~/laksa_maps/<timestamp>/` from the Jetson into `training/field_maps/`,
and pass `--field-fraction 0.2` to `train.py`. Only saved loops are used.

## Run on the Jetson

```bash
cp -a firmware/esp32-s3/jetson/laksa_learned_driver ~/laksa_ws/src/
cd ~/laksa_ws && colcon build --packages-select laksa_learned_driver && source install/setup.bash
ros2 launch laksa_learned_driver learned_cruise.launch.py
```

Run it next to `laksa_bringup/manual_control.launch.py` (the production
`laksa-control-navigation` service). **Do not** run `laksa_system.launch.py enable_autonomy:=true`
at the same time: its rollout LiDAR Cruise node publishes on the same topic. If two publishers
are detected the learned driver stops and reports `CONTROL_ERROR`, which makes the supervisor
abort autonomy.

### Authority and safety

The node publishes only a candidate `Twist` on `/laksa/lidar_cruise_cmd_vel`. `drive_supervisor`
forwards it only after the operator holds **Xbox A for 3 s**, and keeps every existing gate:
fresh Xbox, ESP32/VESC telemetry, LiDAR, odometry and ZED cloud, the e-stop latch (B), immediate
manual override from either stick, X to return to manual, and the `exploration_max_erpm` cap.
The node itself never publishes `/laksa/command`, `/cmd_vel` or `/laksa/brake`.

Speed is capped twice: `speed_cap_mps` in `config/learned_driver.yaml` (default 0.24 m/s) and the
supervisor's `exploration_max_erpm` (1000 eRPM, about 0.24 m/s). Raise them only deliberately,
together, and in small steps after wheels-up and slow floor tests.

### Interfaces and calibration

This branch's `laksa_interfaces` carries the autonomy-handoff fields (`DriveCommand.brake`,
`VescState.telemetry_sequence/telemetry_age_ms`), matching the firmware running on the ESP32.
The ESP32 *source* on this branch predates those fields, so do not flash it as-is. The eRPM
conversion constants in `drive_supervisor.yaml` and the ESP32 `sdkconfig` must still be calibrated
together (`setup/traction_bench.py`).

## Full documentation

The complete write-up (architecture, models, training, safety layers, camera perception, console,
Jetson setup, performance work, field tests and open issues) is in
[`docs/learned_driver/`](../../../../docs/learned_driver/README.md).
