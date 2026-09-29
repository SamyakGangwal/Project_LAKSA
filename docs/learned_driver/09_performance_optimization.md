# Performance optimization

[Index](README.md) · Previous: [Setup and deployment](08_setup_and_deployment.md) · Next: [Field tests and findings](10_field_tests_and_findings.md)

The Orin Nano runs everything on 6 CPU cores and 8 GB of memory shared with the GPU. On the night of the field test it was near its limit: CPU about 85%, GPU about 97%, and RTAB-Map taking 1.5–3.6 s per update against a 0.7 s budget. That load likely contributed to the steering stutter.

## GPU and mapping pass

Done during the field-test night:

| Change | Effect |
|---|---|
| ZED capture 30 → **15 fps**, images published at half resolution | halves depth and detection work |
| RTAB-Map field overrides: detection rate 0.5 Hz, 300/400 features, no 3-D grid (`config/rtabmap_field_overrides.yaml`) | RTAB-Map update **1.5–3.6 s → 0.07 s** |
| Combined | GPU **97% → 46%** |

## CPU pass

Measured per process with `top` on the running stack, before and after:

| Process | Before | After | What changed |
|---|---|---|---|
| `drive_supervisor` | 55% | **34%** | Its full TF listener decodes all of `/tf`, which fell from 267 to 65 Hz (below). Scan and ZED-cloud subscriptions now take raw bytes (`raw=True`), since they're only used as freshness signals. |
| `laksa_console` | 54% | **< 4%** | Subscribes to odometry, scan and camera only while a browser is connected; uses the ZED's compressed image stream instead of re-encoding raw frames |
| `zed_perception` | 44% | **16%** | Point clouds read via NumPy views on the message buffer, not per-point Python; TF from `/tf_static` only |
| `lidar_guard` | 37% | **6%** | Scan statistics vectorised in NumPy (`scan_quality.py`); TF from `/tf_static` only |
| `zed_base_pose_adapter` | 34% | **5%** | TF from `/tf_static` only, and the camera-to-base transform looked up once and cached |
| **Sum of these five** | **~224%** | **~63%** | |
| Session recorder | 17% | about 16% | dropped the camera obstacle cloud from the bag |

After both passes the whole system was **about 49% idle**.

## The /tf flood

`/tf` was running at **267 Hz**, of which **201 Hz** was one transform: the ZED's `zed_left_camera_frame → zed_imu_link`, published on every IMU sample.

- Every node with a Python TF listener deserialises every one of those messages. That includes the supervisor, the console, rtabmap and the recorder.
- Nothing uses that frame. The ZED fuses its IMU internally, and the field EKF fuses the ZED pose and VESC speed.
- The fix was `publish_imu_tf:=false` in the launcher.

`/tf` is now about 65 Hz: `map → odom` at 20 Hz, `odom → base_footprint` at 15 Hz, and the two ZED camera frames at 15 Hz each.

Nodes that only need fixed sensor transforms (`lidar_guard`, `zed_base_pose_adapter`, `zed_perception`) now subscribe to **`/tf_static` only**, through a small `_static_tf_feed` helper, not a full `TransformListener`. The supervisor keeps a full listener because its navigation readiness check needs the moving `map → base_footprint` transform.

## Deploy regression found on the way

After the first CPU pass, `zed_base_pose_adapter` was still at 37%. A 12-second `cProfile` of a scratch copy (publishing to a scratch topic, so the live stack wasn't affected) showed `Buffer.set_transform` running about 237 times a second. The adapter was still processing all of `/tf`.

The deployed file was the **old** version. The deploy step had run `git checkout --` on files whose permission bits the Windows tarball changed, meant to undo the mode change. It also restored the file's contents and silently reverted the fix. Only this file was affected, because its git mode is 644. The deploy procedure now resets modes with `chmod` (see [Setup and deployment](08_setup_and_deployment.md#deploying-code-changes)). All other deployed files were checked by checksum against the local copies.

## What remains

Current top consumers:

| Process | CPU | Notes |
|---|---|---|
| ZED SDK (`zed_node`) | about 87% | depth, detection and tracking. Reducible only by trading quality: resolution, depth mode, or turning detection off while not driving |
| `sllidar_node` | about 26% | the vendor LiDAR driver; not investigated yet |
| `drive_supervisor` | about 34% | mostly rclpy's fixed per-callback overhead (about 1 ms per wake-up on this CPU), at roughly 130 wake-ups a second. A C++ port or fewer subscriptions would cut it further. |
| `zed_perception` | about 16% | |

## How to measure

Per-process CPU, second sample of `top`:

```bash
top -b -n 2 -d 3 -o %CPU | awk '/^top -/{n++} n==2' | head -30
```

Rate of a topic:

```bash
ros2 topic hz /tf
```

Which transforms flood `/tf`: subscribe to `TFMessage` for a few seconds and count `(frame_id, child_frame_id)` pairs.

To profile a Python node without touching the live one, run a second copy under `python3 -m cProfile -s tottime` with a different node name and output topic, stopped with `timeout -s INT`. Never do this for `drive_supervisor`: a second copy would publish real drive commands.
