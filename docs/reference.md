# xsim configuration and interface reference

`xsim` is a simulation server with one process per world. It manages entity identity, physical state, entity generations and the simulation clock. ROS1 is an optional input and output boundary; management uses HTTP/JSON on a Unix socket. See [example.json](../config/example.json) for a configuration example and the [README](../README.md) for an overview.

## Build, install and packaging

Requires C++17, CMake 3.16, Eigen3, nlohmann-json, yaml-cpp, Python3, the xgc2-math headers and the FS150 SITL assets. The default `XSIM_ROS=ON` also needs the ROS Noetic packages `roscpp`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `rosgraph_msgs` and `mavros_msgs`. Install the sensor library `scene/sensors/world_lidar/library` into the chosen prefix first.

The following commands run in the xsim repository root; replace `/private` and the source placeholder paths with real directories.

```sh
cmake -S ../../common/scene/sensors/world_lidar/library -B /private/lidar-build \
  -DCMAKE_INSTALL_PREFIX=/private/install -DCMAKE_BUILD_TYPE=Release
cmake --build /private/lidar-build -j1
cmake --install /private/lidar-build
cmake -S src/xsim -B /private/xsim-build \
  -DCMAKE_PREFIX_PATH='/private/install;/opt/ros/noetic' \
  -DCMAKE_INSTALL_PREFIX=/private/install \
  -DXGC2_MATH_INCLUDE=/path/to/math/include \
  -DFS150_ASSET_SOURCE_ROOT=/path/to/gazebo-sim/fs150-sitl \
  -DCMAKE_BUILD_TYPE=Release -DXSIM_TESTS=ON
cmake --build /private/xsim-build -j1
(cd /private/xsim-build && ctest --output-on-failure)
cmake --install /private/xsim-build
cpack --config /private/xsim-build/CPackConfig.cmake
```

`XSIM_ROS=OFF` builds the same world, robot and sensor systems and the native Unix server without finding or linking ROS. `XSIM_TESTS=ON` enables the model and world checks; the build without ROS also runs `xsim_native_headless`. `XSIM_ROS=OFF ./src/xsim/test.sh` selects the build without ROS.

The installed program is `bin/xsim`; the configuration, the record of the FS150 asset origin and the documentation are in `share/xsim`. The archive package contains xsim; the external ROS and geometry dependencies must be installed separately.

The GPU backend requires the sensor library built with `XGC_WORLD_LIDAR_GPU=ON`, xsim built with `XSIM_GPU=ON`, and PCL, OpenCV, OpenGL, GLFW, GLM, OpenMP and a real GPU/GL context that is usable at run time. When the backend is unavailable it fails explicitly and does not fall back to the CPU. The license of the GPL GPU renderer is installed with the shader assets.

## Process and world configuration

```sh
source /opt/ros/noetic/setup.bash
ROS_MASTER_URI=http://127.0.0.1:PRIVATE_PORT \
  /private/install/bin/xsim --bootstrap-input /private/bootstrap.json --config /private/world.json
```

`--bootstrap-input` is required; exactly one of `--config`, `--experiment-file` and `--manifest` must be given. The Unix endpoint comes from the shared BootstrapInput; an explicit `--socket` must be identical to it. The node name of the ROS build is `/xsim`. The supervisor is responsible for the frozen input, the choice of the ROS master and of the time domain, process start, restart and crash recovery.

`--experiment-file` uses the public frozen input `{instanceId:string, epochNs:string, robots:array, context:object, settings:object}`. `robots` comes unchanged from `asset.experiment-robots@4`, and the zero values and the field presence of `authoredSimulationSensors` are preserved; the existing `simulationSensors` Runtime defaults are not used for the native world projection. `context.openingRunId` and `context.openingAcceptedAtEpochNs` must equal `instanceId` and `epochNs` respectively, the nanoseconds being a positive int64 decimal string. `containerizedDeployment:false` and Core placement must be stated explicitly; where a historical context lacks these facts they are not guessed. `context.visualizationTopics` keeps its declared arrays in the original preset order, and the native product interprets the reference-cloud semantics; it carries no other Action Inputs. From the frozen scene parameters, the original roster and `settings.autoStartGazeboServer` the product resolves models, timing, FCU, the FNV noise seed and sensors, and rejects combinations it does not implement. It does not read back mutable assets and does not use the process start clock in place of the Session time. Both input paths share a bounded file reader that rejects duplicate keys and symbolic links, and both accept a nullable `--scene-file`.

| World key | Default / requirement | Meaning |
|---|---|---|
| `instance_id` | required, non-empty, unique within the session | isolates the management requests of different world instances |
| `epoch_ns` | required, positive integer | start of the formal session time domain, in ns |
| `model_step_ns` | `2000000` | nominal scheduling period of the world, 500 Hz; the duration of one step of a paused Step |
| `max_model_step_ns` | `10000000` | upper bound of the adaptive period and of the actual integration step; nominal period ≤ upper bound ≤ 20 ms |
| `output_period_ns` | `8000000` | telemetry snapshot period, 8 ms, theoretical limit 125 Hz |
| `catchup_batch` | `8` | maximum number of model steps caught up per batch, must be positive |
| `paused` | `false` | initial paused state of the world |
| `publish_clock` | `false` | whether ROS IO publishes `/clock` |
| `input_poll_ns` | `1000000` | polling interval of the ROS input queue |
| `sensor_workers` | `2` | number of CPU sensor worker threads, started on demand |
| `publish_workers` | `2`, integer `1..8` | number of fixed ROS publication shard threads; each robot is handled by exactly one thread |
| `scene` | empty object | immutable world geometry and sampling configuration |
| `scene_file` | optional | YAML scene file, read in and used as `scene.document` |
| `reference_cloud_topic` | empty string | optional one-shot latched topic of the scene reference cloud |
| `telemetry_rates_hz` | table below | ROS telemetry publication rates shared by the whole world; keys given in the configuration override the corresponding defaults |
| `entities` | empty array | list of the initial robot configurations |

The main clock of the world takes the elapsed wall time of `steady_clock` as its target; `epoch_ns + Σdt` is committed only after all robots have completed the shared actual dt. The step count is only a counter; timestamps and receipts are determined by the accumulated integration interval. The session epoch is still given explicitly by the supervisor; the example value is only an example. dt is accumulated in integer ns, and the whole world converts it to seconds only once per integration for the models; a remainder shorter than 1 µs is carried over to later steps, and the nominal period must not be smaller than 1 µs.

Small jitter of a normal wake-up is included directly in the actual dt; one step is at most 1.25 times the current adaptive period and is bounded by `max_model_step_ns`. A short lag is caught up within `catchup_batch` and a budget of about 4 ms of computation per batch, and the world yields for 100 µs between batches. The computational load is evaluated every 100 ms: only after two consecutive windows above 80% does the period grow by 25%, and after five consecutive windows below 50% it shrinks by 5% per window back to the nominal period. An idle world waits for an absolute deadline and a paused world waits for management events, without busy polling. When even the maximum step is not enough to run in real time, the real lag is kept; physical time is not dropped and wall-clock timestamps are not faked.

Pause freezes the clock; Resume resets the wall-clock anchor only on the transition from paused to running and does not catch up the time spent paused. Resume of a world that is already running is idempotent and keeps the current mapping from wall clock to simulation time. A paused Step N still deterministically advances N nominal steps. Resume is rejected while a Step is unfinished. Real-time continuous inputs carry a lower bound on their arrival time and take effect at the next world boundary; they do not act on past debt, and inputs with a future timestamp likewise take effect only at the next boundary. Inputs do not split the integration of the whole fleet at the microsecond arrival time of each robot.

With `publish_clock=true` the supervisor must ensure that there is exactly one `/clock` publisher in this ROS graph and must set `/use_sim_time` consistently for the consumers. `/clock` belongs to ROS IO and can be published before any robot is enabled; the build without ROS still keeps the integer world clock.

The Unix socket has mode `0600`; a missing parent directory is created by the server with mode `0700`, and the mode of an existing parent directory is not rewritten. An existing socket path makes the startup fail and is not removed implicitly. SIGINT/SIGTERM make every thread exit and be joined, close ROS and the client connections, and remove only the socket that the server created itself. The supervisor is responsible for confirming and cleaning up a socket left behind by a crash.

## Robot configuration and lifecycle

| Entity key | Default / requirement | Meaning |
|---|---|---|
| `name` | required, non-empty and unique | ROS namespace segment, only letters, digits and underscores |
| `kind` | required | `fs150`, `scout` or `mecanum` |
| `position` | `[0,0,0]` | initial position of the robot base origin in world ENU |
| `yaw` | `0` | initial yaw, rad |
| `local_origin` | `[0,0,0]` | translation of the FS150 local origin along the world axes |
| `ground_z` | `0` | ground height |
| `fcu_parameters` | optional, FS150 only | object of validated PX4 parameter names and values |
| `ros` | empty object | ROS names, frames and public-localization noise configuration |
| `sensor` | empty object | optional sensor configuration; an empty object creates no sensor |

`local_origin` only translates along the world axes: the MAVROS local pose is the world position minus this vector, and a frame-1 local PVA position adds it back. It does not rotate the ENU axes and does not transform body setpoints again.

A new entity is initially disabled, and the public EntityRef uses a positive generation. A frozen Experiment establishes the correspondence of entities by public robot ID and does not guess the robot identity from the namespace. `GET /v1/entities/<id>` reads the actual ref and state; a state change must carry the generation of this ref. `state.enabled` only switches the run state; only reset resets the model, and it does not roll the clock back. The public generation changes after a delete and re-create; the old ref is rejected.


## ROS topics and services

The paths in the tables are the complete default names; `<name>` is the robot name. A configuration key under the `ros` of an entity can replace the corresponding name, and each item creates exactly one actual topic or service. The `frame` of FS150 defaults to `map` and that of UGVs to `world`; `body_frame` defaults to `base_link`. The public pose/twist always use world coordinates, with frame `world`.

### Control inputs

| Robot | `ros` key | Default topic | Type / meaning |
|---|---|---|---|
| FS150 | `setpoint_topic` | `/<name>/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget`, PVA, mask and frame |
| FS150 | `attitude_topic` | `/<name>/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget`, attitude, body rate, thrust and mask |
| Scout / Mecanum | `cmd_vel_topic` | `/<name>/cmd_vel` | `geometry_msgs/Twist`, body forward/left/yaw rate; Scout ignores left |

### State and measurement outputs

| Robot | `ros` key | Default topic | Type / meaning |
|---|---|---|---|
| all | `localization_pose_topic` | `/<name>/pose` | `geometry_msgs/PoseStamped`, public localization in the world frame |
| all | `localization_twist_topic` | `/<name>/twist` | `geometry_msgs/TwistStamped`, velocity in the world frame |
| FS150 | `pose_topic` | `/<name>/mavros/local_position/pose` | `geometry_msgs/PoseStamped`, pose after the local-origin translation |
| FS150 | `velocity_topic` | `/<name>/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped`, velocity on the world axes |
| FS150 | `odometry_topic` | `/<name>/mavros/local_position/odom` | `nav_msgs/Odometry`, twist rotated into the body child frame |
| FS150 | `imu_topic` | `/<name>/mavros/imu/data` | `sensor_msgs/Imu`, attitude, body specific force and gyro |
| FS150 | `raw_imu_topic` | `/<name>/mavros/imu/data_raw` | `sensor_msgs/Imu`, specific force/gyro; orientation covariance is -1 |
| Scout | `raw_imu_topic` | `/<name>/imu/data_raw` | `sensor_msgs/Imu`, body specific force/gyro; orientation covariance is -1 |
| Mecanum | `imu_topic` | `/<name>/imu` | `sensor_msgs/Imu`, actual yaw attitude, body specific force/gyro |
| FS150 | `state_topic` | `/<name>/mavros/state` | `mavros_msgs/State`, provider/FCU state |
| FS150 | `extended_state_topic` | `/<name>/mavros/extended_state` | `mavros_msgs/ExtendedState`, landed state |
| FS150 | `target_attitude_topic` | `/<name>/mavros/setpoint_raw/target_attitude` | `mavros_msgs/AttitudeTarget`, the actual output of the cascaded controller |

State outputs of the same model step use the same integer clock stamp. `ros.mocap_noise: [sx,sy,sz]` configures Gaussian position noise at the publication boundary of the public pose; each axis standard deviation must be a finite non-negative number, and `ros.mocap_seed` defaults to `1`. The noise is independent of the physical truth and of the local FCU feedback; MAVROS local, IMU and twist stay free of measurement noise. `/<name>/{pose,twist}` are published directly by xsim, and the xsim profile of the Adapter subscribes to these localization topics directly.

The IMU frame is `body_frame`, the axes are forward/left/up, gyro is in rad/s, and linear_acceleration is the specific force including gravity, `Rᵀ(a_world − g_world)`, in m/s². Both vehicles use a planar attitude, so z is `+9.8066` at rest; the planar acceleration is computed from the actual integration dt, the actual velocity change of the model and the turning term. The direct velocity model of the Mecanum shows an acceleration over a finite step at the step where the velocity changes; tyre or suspension impacts are not simulated. Raw IMU provides no attitude estimate, and the first entry of its orientation covariance is `-1`.

### Telemetry publication rates

`telemetry_rates_hz` is configured per topic group and all robots of the fleet use the same settings; it does not change the model integration period, the topic names or the sensor `rate_hz`.

| Key | Default Hz | Published content |
|---|---|---|
| `localization` | `125` | public pose and twist of all robots |
| `local` | `30` | FS150 local pose, velocity and odometry |
| `imu` | `30` | FS150 `/mavros/imu/data`, Mecanum `/imu` |
| `imu_raw` | `30` | FS150 `/mavros/imu/data_raw`, Scout `/imu/data_raw` |
| `state` | `1` | FS150 `/mavros/state` |
| `extended_state` | `1` | FS150 `/mavros/extended_state` |
| `target` | `10` | FS150 `/mavros/setpoint_raw/target_attitude` |

Values must be finite and within `0..1000` Hz, and the period of a positive rate must be representable as an integer number of ns; `0` turns the periodic publication of that group off while the topic is kept. Publication uses integer deadlines with the phase of the common world epoch and does not resend missed old samples. The actual periodic rate of each group is limited by the output snapshot rate; the default `output_period_ns=8000000` corresponds to a theoretical limit of `125` Hz. When the world or the output thread lags, the usable snapshot rate drops further; the requested rate and the theoretical cap do not represent the rate a receiver actually gets.

While paused, frozen numerical measurements are not published repeatedly. A new generation or a timestamp going backwards resets the publication deadlines; a change of the provider/FCU state or of the landed state immediately publishes the corresponding state group, and the change notification is also kept when `state` and `extended_state` are set to `0`. The rates are given by `telemetry_rates_hz` in the native configuration.

### FS150 services

| `ros` key | Default service | Type / meaning |
|---|---|---|
| `arming_service` | `/<name>/mavros/cmd/arming` | `mavros_msgs/CommandBool`, arm/disarm |
| `mode_service` | `/<name>/mavros/set_mode` | `mavros_msgs/SetMode`, custom mode |
| `command_service` | `/<name>/mavros/cmd/command` | `mavros_msgs/CommandLong`, only command 400 (arm/disarm) |

Arm returns the actual result of the model; disarming in the air is rejected and forced operations are not supported. `CommandLong` requires `param1` to be 0 or 1 and accepts no broadcast, confirmation or unrelated parameters.

Mode requires `base_mode=0` and a valid non-empty `custom_mode`. The model supports `OFFBOARD`, `POSCTL`, `ALTCTL`, `AUTO.LOITER` and `AUTO.LAND`. MAVROS `mode_sent` means that the request was transmitted: a valid request may return true even if the model rejects it because the OFFBOARD stream is insufficient or the mode is not supported; the actual mode is given by `state`.

Service responses wait for the world execution boundary. A request that has not been claimed by the timeout is atomically cancelled; a claimed request is awaited and its actual completion result is returned.

Subscriptions and FCU service callbacks are bound to the entity ID/generation. Reset invalidates old callbacks, queued commands and old sensor samples. A Twist without a stamp does not carry the generation of its sender; an external sender must stop the old stream before it restarts the provider. Data that an external subscriber has already queued in TCPROS cannot be withdrawn.

Provider and reset use the Unix management interface; the configuration keys `ros.provider_service`, `ros.truth_topic`, `ros.reset_service`, `ros.mocap_topic` and `ros.mocap_velocity_topic` are rejected. `pose_topic`, `velocity_topic` and `odometry_topic` accept FS150 configuration only.

### Sensors and world outputs

| Configuration | Default topic / enabled when | Type |
|---|---|---|
| `sensor.topic` | `/<name>/cloud`, the entity has a sensor | `sensor_msgs/PointCloud2` |
| `sensor.publish_beams` | `/<name>/simple_lidar/beams`, explicitly enabled and supported by the model | `sensor_msgs/PointCloud2` |
| `publish_clock` | `/clock`, when true and ROS IO is compiled in | `rosgraph_msgs/Clock` |
| `reference_cloud_topic` | the configured name is non-empty | latched `sensor_msgs/PointCloud2`, the scene reference cloud is published only once |

## Unix HTTP/JSON management interface

The host uses the shared XRPC HTTP and BootstrapInput. `GET /v1/describe` returns the real ServiceRef; later requests are bound to this process by the instance, request ID and timeout headers of the shared SDK. The incarnation becomes invalid when the process exits.

| Path | Purpose |
| --- | --- |
| `GET /v1/health`, `POST /v1/health/observe` | readiness of the native components and waiting for a revision |
| `GET /v1/world` | world time, step count, entities and native diagnostics |
| `GET /v1/entities`, `GET /v1/entities/<id>` | public entity identity and state |
| `POST /v1/entities`, `DELETE /v1/entities/<id>` | create and delete entities |
| `POST /v1/entities/<id>/state` | `{generation, state:{enabled}}` |
| `POST /v1/entities/<id>/reset` | reset the entity, carrying its generation |
| `POST /v1/world/{pause,resume,step,reset}` | world management; reset does not roll time back |
| `GET /v1/operations/<id>`, `POST /v1/operations/<id>/{wait,cancel}` | exact operation receipt, waiting and cancellation |

The accepted/running result of a mutation does not mean completion. The caller waits for the terminal state and checks the actual result; enabling is not arming, and cancellation does not erase physical effects that were already executed. Apart from the two fixed cold-preparation workers, requests create no per-robot threads; the runtime directory and the HTTP endpoint lease are released after the native work has stopped.

## ECS data and execution flow

`src/xsim/core/` manages the single entity roster, identity, components, commands and the scheduling of the world boundary; `systems/robots.*` prepares and steps FS150, Scout and Mecanum; `systems/sensors.*` manages sensor resources and workers; `models/` holds the numerical models and the PX4 sources; `io/config.*`, `io/native_rpc/` and `io/ros/` manage configuration and transports; `main.cpp` composes these concrete objects.

Dense arrays grouped by robot kind and the authoritative body/planar SoA columns share a stable ID mapping. Controller and filter state use a contiguous AoS, and name, config and ROS handles are low-frequency data. Remove moves the last dense element of that kind into the vacated slot and rebinds its index. An Entity owns its IO and optional sensor resources; snapshots refer to the Entity only weakly, so a removed resource is not kept alive by a snapshot.

ROS and HTTP modify the world through the same boundary command executor. Add prepares the model, the ROS endpoints and the sensor resources off the world thread; Reset prepares the initial model off the world thread and then submits it to the world boundary. GPU initialization and the upload of the static map are completed on the GPU owner before the add is submitted.

Dynamics, controllers and filters receive only the shared actual dt and never read the operating system clock. Flight control updates once per call, and the rigid body keeps safe RK4 substeps of ≤2 ms; planar ground contact is still constrained at the end of the whole call interval. A larger world step reduces the number of control updates but cannot remove the numerical substeps of the rigid body; this model gives no guarantees for complex contact or collisions.

Threads: 1 world thread, 1 input/HTTP thread (main), 2 service threads and 1 output thread; ROS publication defaults to 2 fixed shard threads, assigned by stable entity ID, with exactly one publisher per robot. CPU sensor workers (2 by default) and/or one GPU context worker are started on demand. Native resource preparation and ROS services share the two service workers. The own network threads of ROS handle the transport. World does not perform ROS publication, service waits, socket IO or sensor scans.

At each step boundary World executes the discrete commands in order, then walks the robots by type and commits the actual time. The enable flags of flight control use a compact array, so the hot loop does not look up the entity hash table; the read-only control parameters and the inverse inertia matrices are cached at construction. Continuous inputs can be merged as long as they do not cross a discrete operation, and `input_coalesced` is recorded. The 16 pre-built Frame buffers are rewritten only when nobody holds them, and the array capacity grows with the number of entities; the output, query, sensor and publication threads share the same immutable snapshot, and all body and sensor poses share one stamp. When the snapshot pool is full or the exchange lock is busy the output is skipped while the physical integration continues, and `output_misses` is recorded. A publication shard keeps only the latest pending snapshot, and a replacement counts into `coalesced_worker_frames`; it handles the telemetry of its shard first, then rotates through the completed clouds and checks for newer telemetry after each cloud it sends.

Noetic ROS IO caches non-latched Publications; serialization runs on the shard thread and the data is then handed to the original publication queue on the ROS Poll thread; sending is not executed on World. Each topic starts with 4 wire buffers, and a buffer can be reused only after ROS has released its reference. When the slots are exhausted the number is topped up to N+4 for the current N connections while the existing capacity is kept; data capacity grows only when the peak grows. If there is still no free slot at that capacity, the handover is skipped without blocking; buffer occupancy does not change the sampling rate. The ROS send queue of each TCP connection is 1: when it is full the older pending message is dropped, and the message in flight is still completed by ROS/TCP. Message objects and frame strings are reused. The point-cloud payload is serialized directly into TCPROS bytes without first being copied into the data of a temporary PointCloud2. ROS queues, connections and the kernel transport can still allocate and copy; this reuse does not mean zero allocation or zero copy end to end for the whole ROS process.

The `bytes_estimate` of point clouds and beams counts the serialized bytes handed to ROS multiplied by the number of connections, without TCP/IP headers or retransmissions, and it does not mean that the subscribers have received the data. There is currently no point-cloud bandwidth quota or token bucket. When no wire buffer is available at publication time, the handover is dropped and `buffer_drops` and `backpressure_ns` are accumulated; these metrics do not feed back into the sampling period of the Sensor. The Sensor backs off by itself according to the scan time, the backlog of pending or completed samples and the pressure on the payload pool; data that was already handed to ROS cannot be withdrawn.

A Sensor is an optional component sampled from the output Frame. IO resources are retired with the Entity.

## Sensor configuration and limits

Sensors use the immutable `LidarScene`/geometry/index. The scene comes from `scene` or `scene_file`; the world accepts only the fixed scene given at configuration time, and there is no management endpoint for editing the scene dynamically.

| `sensor` key | Default / range | Meaning |
|---|---|---|
| `backend` | `cpu`; optional `gpu` | observation backend |
| `mode` | `raycast` | CPU: `raycast`, `penetrating`, `depth`; GPU: `lidar_scan` |
| `topic` | `/<name>/cloud` | point-cloud topic |
| `frame` | `world`; optional `map` | frame of the output world points |
| `rate_hz` | `10`, positive | sampling rate in simulation time |
| `range`, `min_range` | `20`, `0` | distance, m |
| `h_fov_deg`, `v_fov_deg` | `360`, `30` | horizontal/vertical FOV, deg |
| `h_res`, `v_res` | `360`, `32` | horizontal/vertical number of samples |
| `translation`, `rotation` | zero translation, unit quaternion | sensor mount; the order of rotation is `[w,x,y,z]` |
| `noise_std`, `seed` | `0`, `0` | CPU Gaussian range noise and random seed |
| `world_bodies` | `false`, CPU only | observe the robot bodies of the whole world in the snapshot of the same step |
| `publish_beams` | `false`, only CPU models that support beams | publish the beam point cloud |
| `surface_spacing`, `keep_buried` | inherit the scene defaults | penetrating sampling spacing and buried policy |
| `heading_crop`, `heading_cos_min`, `vertical_slab_tan` | `false`, `0`, `0.5773502691896258` | penetrating crop |
| `width`, `height` | `160`, `120` | depth image size |
| `fx`, `fy`, `cx`, `cy` | `0` | depth pinhole intrinsics |
| `point_cover_spacing` | inherit the scene spacing | GPU static-map point coverage spacing |

The CPU uses the raycast, penetrating or pinhole depth implementations of `WorldLidar`. The output is normally XYZ; with `world_bodies=true` an INT32 `vehicle_id` is added. The beam output contains `x,y,z,dx,dy,dz,range,hit`, with `vehicle_id` added when robot bodies are observed; the penetrating model does not support beams.

The GPU uses the spherical-nearest `lidar_scan`, outputs XYZ/intensity, shares one static-map upload/context, and switches the projection of each sensor inside the GPU owner thread. The GPU supports neither `world_bodies`, `publish_beams` nor a non-zero `noise_std`. CPU and GPU are explicitly different observation models and cannot be swapped automatically.

The two axes of the GPU kernel share `polar_res`, which requires `h_fov_deg / h_res == v_fov_deg / v_res` and the point-coverage constraint. For example 120° × 60° with 240 × 120 samples is valid; the same FOV with 240 × 30 is rejected. xsim does not modify the FOV or the resolution to accept a mismatching request and does not switch to the CPU.

The output thread delivers the immutable pose, entity ID/generation, stamp and scene version 1 at absolute sampling phases; the actual sampling rate is limited by the rate of the output Frames (125 Hz by default). Every sensor has at most one executing, one pending and one completed sample; the shared CPU worker / GPU owner executes the scan, and under overload the old pending/completed sample is replaced and counted. The computational overload of a sensor increases its sampling period while the requested Hz, FOV, resolution, fields and backend of the configuration are kept; the sampling period backs off in 25% increments up to the larger of 8 times the requested period and 1 s, and recovers step by step in 250 ms wall-clock windows once the pressure is gone. On Linux the observation workers try to use a lower scheduling priority (nice 5). This degradation does not block World.

Every sensor pre-builds 4 payload buffers; a computation writes only a buffer that nobody holds, and the cache, the completed sample and the publisher share the immutable data; when the pool is full the sample is dropped. A noise-free CPU observation reuses the cache only when the sensor pose and the geometry of the observed bodies are exactly the same and the generation has not changed; a hit copies no point-cloud/beam bytes, while noise or motion recomputes. The static scene index, the CPU buffers and the GPU map/context are reused. After a scan completes the body Frame is released, and the cache does not hold world snapshots for long. Publication keeps the sampling stamp and drops results of removed, disabled or expired generations. Memory grows with the configured entities, beam patterns, output and geometry cache buffers and the shared scene.

In the native diagnostics of `GET /v1/world`, `simulation_time_ns` is the time already integrated, `model_step_ns` is the nominal period, `scheduling_period_ns` is the current scheduling period and `last_dt_ns` is the most recent actual integration interval. `rtf` is the cumulative ratio since the process start (including pauses and manual Steps); `realtime_rtf` counts only the automatically run integrated time divided by the active wall-clock time. `frame_slots` and `frame_array_grows` report the snapshot pool and the growth of its arrays; each Sensor reports its requested/effective/source/observed rate, throttled samples, misses, cache hits, the number of actual computations, and payload slots, growth and drops.

`publication` reports the number of shards, snapshot coalescing, preparation time and errors, and the handover counts, estimated bytes, buffer allocation/capacity/drops, serialization and queue handover time of telemetry, cloud and clock. `accepted` means entering the ROS publication queue, not an acknowledgement by the subscriber; `backpressure_ns` is the accumulated age of the buffers observed when a handover was refused, not blocking time.
