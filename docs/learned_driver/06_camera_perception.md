# Camera perception

[Index](README.md) · Previous: [Runtime safety](05_runtime_safety.md) · Next: [Console and operation](07_console_and_operation.md)

## Why the camera is needed

The LiDAR sees one horizontal slice about 13.5 cm above the ground. Anything **lower** (a shoe, a cable, a low box edge) or **overhanging** (a table top, a chair seat) is invisible to it. The ZED 2i fills that gap and adds object recognition.

The camera layer **adds to** the LiDAR and never replaces it. If camera data goes stale (more than 0.6 s old), the driver logs a warning and continues on LiDAR alone.

## ZED features used

Profile: `laksa_learned_driver/config/zed_perception.yaml`.

| ZED feature | Setting | Used for |
|---|---|---|
| Capture | HD720 at **15 fps** (was 30), images published at half resolution | halves GPU work for depth and detection |
| Neural depth | `NEURAL_LIGHT`, confidence threshold **50** (default 95), texture confidence 90, 0.3–8 m | point cloud for obstacles; RGB-D for mapping |
| Point cloud | 5 Hz, reduced resolution, 5 cm voxels | obstacle heights, ground plane |
| Object detection | `MULTI_CLASS_BOX_FAST`, tracking on, NMS3D, up to 6 m, reduced-precision inference | 3-D boxes: people, vehicles, bags, animals, electronics and so on |
| Positional tracking | `GEN_3`, IMU fusion on, `publish_tf` off | camera pose, fed to the EKF through `zed_base_pose_adapter` |
| IMU | published; its TF is **off** (`publish_imu_tf:=false`) | used by the ZED's own tracking only |
| Spatial mapping | off | RTAB-Map does the mapping |

The lower confidence threshold and the 15 fps rate both came from the field test. Low-light depth noise produced phantom obstacles, and the GPU was at 97%.

## zed_perception node

`laksa_learned_driver/zed_perception_node.py`, with the pure NumPy logic in `perception.py`. It takes the ZED's point cloud and detections, transforms them into `base_footprint`, and publishes:

| Topic | Content |
|---|---|
| `/laksa/perception/obstacles` | `PointCloud2`, z = 0: obstacle points on the floor plane |
| `/laksa/perception/person` | JSON `{factor, stop, nearest_m}`: the person speed rule |
| `/laksa/perception/detections` | JSON list of detected objects, for the console |
| `/laksa/perception/ground` | JSON `{plane: [a, b, c]}` or `null`: fitted ground `z = a·x + b·y + c` |

The node reads point clouds through NumPy views on the raw message buffer, not per-point Python, and gets its camera transform from `/tf_static` only. Both matter for CPU load; see [Performance](09_performance_optimization.md).

## Obstacles from the depth cloud

`filter_cloud` keeps a point as an obstacle when:

- its height **above the fitted ground plane** is 5–45 cm (flat ground is assumed when no plane is fitted);
- it is at least **0.55 m ahead of `base_footprint`**, that is, 13 cm beyond the bumper. Closer ZED depth is unreliable, especially in low light, and the LiDAR covers that zone. This rule removed the phantom "blocked 0.00 m ahead of the bumper" stops from the field test;
- it is within 4 m.

The points are then thinned to a 5 cm grid.

## Obstacles from detections

Each detected box's floor footprint (the convex hull of its eight corners) is sampled every 5 cm and added as obstacle points. People's footprints are enlarged by 0.25 m. Detections beyond 4 m are ignored.

## Ground plane and slopes

The problem: a slope rising ahead crosses the LiDAR's flat slice, so it looks like a wall. At the field the car stopped in front of a slope for this reason.

The fix (`fit_ground_plane`, `lidar_ground_mask`):

1. **RANSAC** fits a plane to cloud points 0.5–3 m ahead and within ±0.30 m of the car's ground height (40 iterations, 3 cm inlier band, at least 120 inliers). A least-squares refit on the inliers follows.
2. The plane is accepted only if it is no steeper than **20°** and passes within 15 cm of the car's own ground.
3. Camera obstacle heights are measured **against that plane**, so the slope itself isn't an obstacle but a box sitting on it is.
4. A LiDAR return where the plane rises to the LiDAR's height (within 5 cm) is treated as **ground, not wall**, and dropped from the governor's points.

**Status:** unit-tested, but not yet confirmed on real sloped ground. The only fit at the bench placed the floor 14 cm below the wheels while the car stood on a stand, and it was correctly rejected.

## Person rule

`person_speed_rule` looks at people whose centre lies within a ±60° cone ahead. It uses the nearest person's distance beyond the bumper: at 1 m or less, stop and hold (no reversing); at 2 m or less, half speed. Details in [Runtime safety](05_runtime_safety.md#person-rule).
