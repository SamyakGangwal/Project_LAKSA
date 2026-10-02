# LAKSA learned driver: documentation index

This folder documents the `feature/learned-driver` branch:
- an imitation-learned LiDAR driving network for the LAKSA car;
- the camera perception and safety layers around it;
- the operator console, boot services and field networking;
- the performance work;
- everything learned from bench and field tests.

Start with the [Overview](01_overview.md). Otherwise, jump to what you need below.

## Quick paths

| I want to… | Go to |
|---|---|
| understand what was built and where it stands | [Overview: status at a glance](01_overview.md#status-at-a-glance) |
| see how data flows from the sensors to the motor | [Architecture: data flow](02_system_architecture.md#data-flow) |
| know which ML models are used | [Models at a glance](03_models.md#models-at-a-glance) |
| retrain the network | [Training: how to retrain](04_training_pipeline.md#how-to-retrain) |
| know why the car stopped | [Safety: decision log](05_runtime_safety.md#decision-log) and [Ways the car stops](05_runtime_safety.md#ways-the-car-stops) |
| drive the car at the field | [Console: field checklist](07_console_and_operation.md#field-checklist) |
| pick a drive mode (Obstacle, Speed, Trial & explore) | [Console: drive modes](07_console_and_operation.md#drive-modes) |
| train on a place the car explored | [Console: saving a map](07_console_and_operation.md#saving-a-map-for-training), then [Training: maps the car explored](04_training_pipeline.md#training-on-maps-the-car-explored) |
| know what KarSha changed on the bench Jetson | [Known issues: work by KarSha](11_known_issues_and_next_steps.md#work-by-karsha-on-the-bench-jetson) |
| set up a Jetson from scratch | [Setup: one-time setup](08_setup_and_deployment.md#one-time-setup) |
| deploy a code change safely (one package, installed at boot) | [Setup: deploying code changes](08_setup_and_deployment.md#deploying-code-changes) |
| find a run's logs on the car | [Setup: sessions and recording](08_setup_and_deployment.md#sessions-and-recording) (`~/laksa_logs`) |
| know why the car stopped far from a wall or didn't back up (1 Oct) | [Field tests: explore run](10_field_tests_and_findings.md#explore-run-1-oct) |
| see what's broken or next | [Known issues and next steps](11_known_issues_and_next_steps.md) |
| review every changed file | [Change log](12_change_log.md) |

## Contents

### 1. [Overview](01_overview.md)
- [What this work is](01_overview.md#what-this-work-is)
- [Status at a glance](01_overview.md#status-at-a-glance)
- [How the pieces fit](01_overview.md#how-the-pieces-fit)
- [Timeline](01_overview.md#timeline)
- [What is not done](01_overview.md#what-is-not-done)

### 2. [System architecture](02_system_architecture.md)
- [Hardware](02_system_architecture.md#hardware)
- [Process map](02_system_architecture.md#process-map)
- [Data flow](02_system_architecture.md#data-flow)
- [Key topics](02_system_architecture.md#key-topics)
- [TF tree](02_system_architecture.md#tf-tree)
- [Control authority](02_system_architecture.md#control-authority)
- [Message interface](02_system_architecture.md#message-interface)

### 3. [Models](03_models.md)
- [Models at a glance](03_models.md#models-at-a-glance)
- [Why imitation learning and not PPO](03_models.md#why-imitation-learning-and-not-ppo)
- [Network architecture](03_models.md#network-architecture)
- [Input contract](03_models.md#input-contract)
- [Output contract](03_models.md#output-contract)
- [Model file](03_models.md#model-file)
- [What the network does and does not do](03_models.md#what-the-network-does-and-does-not-do)

### 4. [Training pipeline](04_training_pipeline.md)
- [Simulator](04_training_pipeline.md#simulator)
- [Tracks](04_training_pipeline.md#tracks)
- [Expert](04_training_pipeline.md#expert)
- [DART and DAgger](04_training_pipeline.md#dart-and-dagger)
- [Domain randomisation](04_training_pipeline.md#domain-randomisation)
- [Parallel workers](04_training_pipeline.md#parallel-workers)
- [Results](04_training_pipeline.md#results)
- [Obstacles and the 2026 courses (in progress)](04_training_pipeline.md#obstacles-and-the-2026-courses-in-progress)
- [Training on maps the car explored](04_training_pipeline.md#training-on-maps-the-car-explored)
- [How to retrain](04_training_pipeline.md#how-to-retrain)

### 5. [Runtime driving and safety](05_runtime_safety.md)
- [Per-scan pipeline](05_runtime_safety.md#per-scan-pipeline)
- [Clearance governor](05_runtime_safety.md#clearance-governor)
- [Steering around obstacles](05_runtime_safety.md#steering-around-obstacles)
- [Reverse recovery](05_runtime_safety.md#reverse-recovery)
- [Steering smoothing](05_runtime_safety.md#steering-smoothing)
- [Person rule](05_runtime_safety.md#person-rule)
- [Driver status values](05_runtime_safety.md#driver-status-values)
- [Supervisor gates](05_runtime_safety.md#supervisor-gates)
- [Race mode gates](05_runtime_safety.md#race-mode-gates)
- [Ways the car stops](05_runtime_safety.md#ways-the-car-stops)
- [Decision log](05_runtime_safety.md#decision-log)

### 6. [Camera perception](06_camera_perception.md)
- [Why the camera is needed](06_camera_perception.md#why-the-camera-is-needed)
- [ZED features used](06_camera_perception.md#zed-features-used)
- [zed_perception node](06_camera_perception.md#zed_perception-node)
- [Obstacles from the depth cloud](06_camera_perception.md#obstacles-from-the-depth-cloud)
- [Obstacles from detections](06_camera_perception.md#obstacles-from-detections)
- [Ground plane and slopes](06_camera_perception.md#ground-plane-and-slopes)
- [Person rule](06_camera_perception.md#person-rule)

### 7. [Console and operation](07_console_and_operation.md)
- [LAKSA Console](07_console_and_operation.md#laksa-console)
- [Drive modes](07_console_and_operation.md#drive-modes)
- [Saving a map for training](07_console_and_operation.md#saving-a-map-for-training)
- [Terminal operator](07_console_and_operation.md#terminal-operator)
- [Networking at the field](07_console_and_operation.md#networking-at-the-field)
- [Field checklist](07_console_and_operation.md#field-checklist)
- [Race mode](07_console_and_operation.md#race-mode)

### 8. [Setup and deployment](08_setup_and_deployment.md)
- [Target machine](08_setup_and_deployment.md#target-machine)
- [One-time setup](08_setup_and_deployment.md#one-time-setup)
- [Launcher](08_setup_and_deployment.md#launcher)
- [Boot services](08_setup_and_deployment.md#boot-services)
- [Sessions and recording](08_setup_and_deployment.md#sessions-and-recording)
- [Deploying code changes](08_setup_and_deployment.md#deploying-code-changes)
- [Bench tools](08_setup_and_deployment.md#bench-tools)

### 9. [Performance optimization](09_performance_optimization.md)
- [GPU and mapping pass](09_performance_optimization.md#gpu-and-mapping-pass)
- [CPU pass](09_performance_optimization.md#cpu-pass)
- [The /tf flood](09_performance_optimization.md#the-tf-flood)
- [Deploy regression found on the way](09_performance_optimization.md#deploy-regression-found-on-the-way)
- [What remains](09_performance_optimization.md#what-remains)
- [How to measure](09_performance_optimization.md#how-to-measure)

### 10. [Field tests and findings](10_field_tests_and_findings.md)
- [Read-only hardware check](10_field_tests_and_findings.md#read-only-hardware-check)
- [Steering bench](10_field_tests_and_findings.md#steering-bench)
- [Traction bench](10_field_tests_and_findings.md#traction-bench)
- [Stage-1 bench](10_field_tests_and_findings.md#stage-1-bench)
- [First autonomous run](10_field_tests_and_findings.md#first-autonomous-run)
- [LiDAR rear visibility](10_field_tests_and_findings.md#lidar-rear-visibility)
- [Outdoor field test](10_field_tests_and_findings.md#outdoor-field-test)
- [Explore run (1 Oct)](10_field_tests_and_findings.md#explore-run-1-oct)

### 11. [Known issues and next steps](11_known_issues_and_next_steps.md)
- [Unreliable start from standstill (and reverse on the floor)](11_known_issues_and_next_steps.md#unreliable-start-from-standstill-and-reverse-on-the-floor)
- [Motor breakaway current](11_known_issues_and_next_steps.md#motor-breakaway-current)
- [Route planning from the console](11_known_issues_and_next_steps.md#route-planning-from-the-console)
- [Work by KarSha on the bench Jetson](11_known_issues_and_next_steps.md#work-by-karsha-on-the-bench-jetson)
- [Model limits](11_known_issues_and_next_steps.md#model-limits)
- [Awaiting real-world confirmation](11_known_issues_and_next_steps.md#awaiting-real-world-confirmation)
- [Clock and networking](11_known_issues_and_next_steps.md#clock-and-networking)
- [Calibration and interfaces](11_known_issues_and_next_steps.md#calibration-and-interfaces)
- [Security](11_known_issues_and_next_steps.md#security)
- [Pre-existing repository issues](11_known_issues_and_next_steps.md#pre-existing-repository-issues)
- [Performance headroom](11_known_issues_and_next_steps.md#performance-headroom)
- [Race mode](11_known_issues_and_next_steps.md#race-mode)

### 12. [Change log](12_change_log.md)
- [New package: laksa_learned_driver](12_change_log.md#new-package-laksa_learned_driver)
- [Changes to existing packages](12_change_log.md#changes-to-existing-packages)
- [Setup, services and patches](12_change_log.md#setup-services-and-patches)
- [Documentation](12_change_log.md#documentation)
- [Scratch artifacts](12_change_log.md#scratch-artifacts)

## Where the code is

| What | Path |
|---|---|
| Learned driver package | `firmware/esp32-s3/jetson/laksa_learned_driver/` |
| Training code | `firmware/esp32-s3/jetson/laksa_learned_driver/training/` |
| Default model | `firmware/esp32-s3/jetson/laksa_learned_driver/models/laksa_tinylidarnet_v5.npz` |
| Drive profiles (mode defaults) | `firmware/esp32-s3/jetson/laksa_learned_driver/laksa_learned_driver/profiles.py` |
| Saved field maps for training | `firmware/esp32-s3/jetson/laksa_learned_driver/training/field_maps/` (copied from `~/laksa_maps/` on the Jetson) |
| Launcher, setup, bench tools | `firmware/esp32-s3/jetson/setup/` |
| Boot services | `firmware/esp32-s3/jetson/systemd/laksa-car.service`, `laksa-network-watch.service` |
