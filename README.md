# xsim

xsim is a lightweight multi-robot simulator with one process per world. It manages entities, dynamics, entity enablement and the simulation clock in one place. Control and telemetry go through an optional ROS 1 interface; world management is HTTP/JSON RPC on a Unix socket.

[Full configuration and interface reference](docs/reference.md)

## Supported robots

| `kind` | Model | Control input |
| --- | --- | --- |
| `fs150` | PX4 v1.12.3 cascaded control → four-motor response → 6-DoF rigid body (RK4) → planar ground contact | PVA, attitude or body-rate targets; MAVROS Arm / Mode |
| `scout` | velocity limits → pure delay and first-order response → differential-drive planar kinematics | body-frame forward speed and yaw rate |
| `mecanum` | velocity limits → omnidirectional planar kinematics | body-frame forward and lateral speed and yaw rate |

FS150 uses a lightweight FCU interface and the fixed PX4 control sources; Scout and Mecanum use planar models. Sensor scenes are for observation only; they do not take part in the collision dynamics of robots or obstacles.

## Start

```sh
source /opt/ros/noetic/setup.bash
xsim --bootstrap-input /private/bootstrap.json --config /private/world.json
```

The supervisor first creates the private runtime directory and the shared XRPC BootstrapInput; the binding names `xgc2.simulation`, `v1`, `http.v1`, the Unix endpoint and `local_private`. The native host generates the instance ID of this ServiceRef itself; the fixed input does not pre-create an incarnation. The runtime directory must already exist, be owned by the current user and have mode `0700`. The ROS build also requires existing `ROS_HOME` and `ROS_LOG_DIR` directories and the matching `rosHomeGrant` and `rosLogGrant` in the application section.

`--config` takes a native configuration like [config/example.json](config/example.json); `--manifest` takes a native world and entity assets with an explicit epoch. Workflows use `--experiment-file /private/experiment.json`; the three inputs are mutually exclusive, and `--scene-file /private/scene.yaml` may be added. A frozen Experiment is still `{instanceId, epochNs, robots, context, settings}` and keeps the exact time, the public robot IDs, the raw `authoredSimulationSensors` and the full context; models, sensors and timing are interpreted in C++. `GET /v1/entities/<robotId>` returns the actual EntityRef, whose generation an enable request must carry.

The world schedules at 500 Hz (2 ms) by default. The whole fleet shares the measured wall-clock dt and the time is `epoch_ns + Σ actual dt`. A short lag is caught up in bounded batches, a lasting overload smoothly increases the period (10 ms at most by default), and an idle world blocks. The snapshot period is 8 ms; public localization defaults to 125 Hz and IMU/local to 30 Hz, the telemetry rates are set in the native configuration, and point clouds default to 10 Hz and are throttled independently. Pause keeps the management interface available, Step advances a paused world by the given number of nominal steps, and Reset keeps the session clock and the entities' enabled state.

## Interfaces

The paths below are defaults; `<name>` is the entity name. ROS configuration can override the interface names.

| Direction / scope | Topic or service |
| --- | --- |
| Input · FS150 | `/<name>/mavros/setpoint_raw/{local,attitude}` |
| Input · Scout / Mecanum | `/<name>/cmd_vel` |
| Output · all robots | `/<name>/{pose,twist}`, public localization in the world frame; pose supports configurable position noise |
| Output · FS150 | `/<name>/mavros/local_position/{pose,velocity_local,odom}`, `imu/{data,data_raw}`, `state`, `extended_state`, `setpoint_raw/target_attitude` (all in the same MAVROS namespace) |
| Output · Scout / Mecanum | Scout `/<name>/imu/data_raw`, Mecanum `/<name>/imu`; body-frame IMU, 30 Hz by default |
| Output · sensor / world | optional `/<name>/cloud`, optional CPU `/<name>/simple_lidar/beams`; `/clock` when `publish_clock=true` |
| ROS service · FS150 | `/<name>/mavros/{cmd/arming,set_mode,cmd/command}` |
| RPC queries | `GET /v1/describe`, `/v1/health`, `/v1/world`, `/v1/entities`, `/v1/operations/<id>` |
| RPC management | entity create and delete, `POST /v1/entities/<id>/state`, `/reset`, world `/v1/world/{pause,resume,step,reset}`, operations `/v1/operations/<id>/{wait,cancel}` |

RPC uses the request ID, timeout and instance headers of the shared SDK; entity operations also carry the actual generation. `202` only means accepted: the terminal state of the operation must be awaited to judge the domain result. Entity enablement is independent of FS150 Arm; disabling does not reinitialize the model, and reset keeps the enabled state and the world clock.

## Sources and build

| Directory (relative to `src/xsim/`) | Responsibility |
| --- | --- |
| `core/` | the single entity table, stable ID / generation, components, commands and execution at the world boundary |
| `systems/` | batched stepping of the three robot types; CPU / GPU sensor tasks and workers |
| `models/` | numerical robot models and the fixed PX4 control sources |
| `io/` | configuration, Unix RPC, ROS input and output |
| `main.cpp` | composes World, Sensors, IO and Server |

The physical components of World are arrays by robot type (SoA), and controller and filter state sits in compact model arrays; ROS and RPC commands run at the same world boundary. Immutable snapshots are shared by the output, sensor and publication threads, and point-cloud data is shared and reused; ROS publication uses two fixed shard threads by default, with four initial send buffers per topic that are topped up by connection usage and then reused.

Build, install and packaging are in the [reference](docs/reference.md#build-install-and-packaging). `XSIM_ROS=OFF` builds the same world and RPC service without ROS; GPU observation is enabled by an explicit build option.

The test entry is `src/xsim/test.sh`; `tests/validate.sh` provides an isolated build and the ROS integration checks.
