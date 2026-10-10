# xsim 配置与接口参考

`xsim` 是一个世界一个进程的仿真服务器，管理实体身份、物理状态、实体 generation 和仿真时钟。ROS1 是可选的输入输出边界；管理使用 Unix socket 上的 HTTP/JSON。配置示例见 [example.json](../config/example.json)，总览见 [README](../README.md)。

## 构建、安装与打包

需要 C++17、CMake 3.16、Eigen3、nlohmann-json、yaml-cpp、Python3、xgc2-math headers 和 FS150 SITL 资产。默认 `XSIM_ROS=ON` 还需要 ROS Noetic 的 `roscpp`、`geometry_msgs`、`sensor_msgs`、`nav_msgs`、`rosgraph_msgs`、`mavros_msgs`。先将传感器库 `scene/sensors/world_lidar/library` 安装到选定前缀。

以下命令在 xsim 仓根执行；将 `/private` 和源码占位路径替换为实际目录。

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

`XSIM_ROS=OFF` 编译同一世界、机器人和传感器系统及 native Unix server，不查找或链接 ROS。`XSIM_TESTS=ON` 启用模型/世界检查；无 ROS 构建还运行 `xsim_native_headless`。`XSIM_ROS=OFF ./src/xsim/test.sh` 可选择无 ROS 构建。

安装后的程序为 `bin/xsim`，配置、FS150 资产来源记录和文档位于 `share/xsim`。归档包包含 xsim；外部 ROS 和 geometry 依赖需要单独安装。

GPU 后端要求传感器库启用 `XGC_WORLD_LIDAR_GPU=ON`，xsim 启用 `XSIM_GPU=ON`，并提供 PCL、OpenCV、OpenGL、GLFW、GLM、OpenMP 和运行时可用的真实 GPU/GL context。后端不可用时明确失败，不回退到 CPU。GPU renderer 的 GPL 许可证随 shader 资产安装。

## 进程与世界配置

```sh
source /opt/ros/noetic/setup.bash
ROS_MASTER_URI=http://127.0.0.1:PRIVATE_PORT \
  /private/install/bin/xsim --bootstrap-input /private/bootstrap.json --config /private/world.json
```

`--bootstrap-input` 必填；`--config`、`--experiment-file` 与 `--manifest` 必须且只能选择一个。Unix endpoint 来自共享 BootstrapInput；如显式传入 `--socket`，它必须与 binding 完全相同。ROS 构建的节点名为 `/xsim`。监督者负责冻结输入、ROS master 和时间域选择、进程启动、重启与崩溃恢复。

`--experiment-file` 使用公开冻结输入 `{instanceId:string, epochNs:string, robots:array, context:object, settings:object}`。`robots` 原样来自 `asset.experiment-robots@4`，同时保留 `authoredSimulationSensors` 的零值和字段存在性；既有 `simulationSensors` Runtime 默认值不会用于原生世界投影。`context.openingRunId` 和 `context.openingAcceptedAtEpochNs` 必须分别与 `instanceId`、`epochNs` 一致，纳秒为正的 int64 十进制字符串。`containerizedDeployment:false` 和 Core placement 必須明确存在；历史上下文缺少这些事实时不能补猜。`context.visualizationTopics` 按原 preset 顺序保存其声明数组，原生产品解释 reference-cloud 语义，不带其它 Action Inputs。产品据冻结场景参数、原始 roster 和 `settings.autoStartGazeboServer` 解析模型、时序、FCU、FNV 噪声 seed 与传感器，并拒绝未实现组合。它不回读可变资产，不用进程启动时钟替代 Session 时间。两种输入入口共用有界、拒绝重复键和符号链接的文件读取器，并同样接受可空的 `--scene-file`。

| 世界键 | 默认值 / 要求 | 含义 |
|---|---|---|
| `instance_id` | 必填、非空、会话内唯一 | 隔离不同世界实例的管理请求 |
| `epoch_ns` | 必填、正整数 | 正式会话时间域的起点，单位 ns |
| `model_step_ns` | `2000000` | 世界名义调度周期，500 Hz；暂停 Step 的单步时长 |
| `max_model_step_ns` | `10000000` | 自适应周期和实际积分步的上限；名义周期 ≤ 上限 ≤ 20 ms |
| `output_period_ns` | `8000000` | 遥测快照周期，8 ms，理论上限 125 Hz |
| `catchup_batch` | `8` | 每批最多追赶的模型步数，须为正 |
| `paused` | `false` | 初始世界暂停状态 |
| `publish_clock` | `false` | ROS IO 是否发布 `/clock` |
| `input_poll_ns` | `1000000` | ROS 输入队列轮询间隔 |
| `sensor_workers` | `2` | 按需启动的 CPU 传感器工作线程数 |
| `publish_workers` | `2`，整数 `1..8` | ROS 发布固定分片线程数；每个机器人由唯一线程处理 |
| `scene` | 空对象 | 不可变世界几何与采样配置 |
| `scene_file` | 可选 | YAML 场景文件，读入后作为 `scene.document` |
| `reference_cloud_topic` | 空字符串 | 可选的一次性 latched 场景参考点云主题 |
| `telemetry_rates_hz` | 下表 | 全世界共享的 ROS 遥测发布频率；配置提供的键覆盖对应默认值 |
| `entities` | 空数组 | 初始机器人配置列表 |

世界主时钟以 `steady_clock` 的墙钟经过时间作为目标，所有机器人完成共享实际 dt 后才提交 `epoch_ns + Σdt`；步数仅为计数，时间戳和回执由累计积分间隔确定。会话 epoch 仍由监督者显式指定，示例值仅为示例。dt 以整数 ns 累加，每次全世界积分仅转换一次秒数供模型使用；不足 1 µs 的余量保留到后续步，名义周期不得小于 1 µs。

正常唤醒的微小抖动直接包含在实际 dt 中；单步不超过当前自适应周期的 1.25 倍，并受 `max_model_step_ns` 限制。短时落后按 `catchup_batch` 和每批约 4 ms 运算预算追赶，批间阻塞让出 100 µs。持续运算负载每 100 ms 评估，连续两窗超过 80% 才将周期增加 25%；连续五窗低于 50% 后逐窗减少 5%，回到名义周期。空闲时等待绝对 deadline，暂停时等待管理事件，无忙轮询。达到步长上限仍不足以实时运行时，保持真实 lag，不丢物理时间、不伪造墙钟时间戳。

Pause 冻结时钟；Resume 仅在 paused 转为 running 时重置墙钟锚点，不追赶暂停期间的时间。已经运行时的 Resume 为幂等操作，保持当前墙钟到仿真时间的映射。暂停 Step N 仍确定地前进 N 个名义步。Step 尚未完成时 Resume 被拒绝。实时连续输入带到达时间下界，在下一世界边界生效，不提前作用于历史债务；未来时间戳同样允许到下一边界才生效。输入不按逐机器人的微秒到达时刻切分整群积分。

`publish_clock=true` 时，监督者必须保证该 ROS 图内只有一个 `/clock` 发布者，并为消费者一致设置 `/use_sim_time`。`/clock` 属于 ROS IO，在任何机器人 Provider 启用前即可发布；无 ROS 构建仍保留世界整数时钟。

Unix socket 权限为 `0600`；缺失父目录由服务器以 `0700` 创建，既有父目录权限不被改写。已有 socket 路径会使启动失败，不会被隐式删除。SIGINT/SIGTERM 触发各自线程退出并 join、关闭 ROS 和客户端连接，只删除服务器自身创建的 socket。崩溃后确认并清理残留 socket 由监督者负责。

## 机器人配置与生命周期

| 实体键 | 默认值 / 要求 | 含义 |
|---|---|---|
| `name` | 必填、非空且唯一 | ROS namespace segment，仅字母、数字、下划线 |
| `kind` | 必填 | `fs150`、`scout` 或 `mecanum` |
| `position` | `[0,0,0]` | 世界 ENU 中机器人 base origin 的初始位置 |
| `yaw` | `0` | 初始偏航角，rad |
| `local_origin` | `[0,0,0]` | FS150 本地坐标原点的世界轴平移 |
| `ground_z` | `0` | 地面高度 |
| `fcu_parameters` | 可选，仅 FS150 | 经校验的 PX4 参数名/值对象 |
| `ros` | 空对象 | ROS 名称、frame 和公共定位噪声配置 |
| `sensor` | 空对象 | 可选传感器配置；空对象不创建传感器 |

`local_origin` 仅平移世界轴：MAVROS local pose 为世界位置减该向量；frame-1 local PVA 位置加回该向量。它不旋转 ENU 轴，不重复变换 body setpoint。

新实体初始禁用，公开 EntityRef 使用正 generation。冻结实验按公开机器人 ID 建立实体对应关系，不从 namespace 猜测机器人身份。`GET /v1/entities/<id>` 读取实际 ref 和状态；状态修改必须携带此 ref 的 generation。`state.enabled` 只切换运行状态，reset 才重置模型，且不回退时钟。删除后重建的公开 generation 改变；旧 ref 被拒绝。


## ROS 话题与服务

表中的路径为完整默认名称；`<name>` 是机器人名称。实体 `ros` 下的配置键可替换相应名称，每项只创建一个实际 topic/service。FS150 的 `frame` 默认 `map`，UGV 默认 `world`；`body_frame` 默认 `base_link`。公共 pose/twist 始终使用世界坐标，frame 为 `world`。

### 控制输入

| 机器人 | `ros` 配置键 | 默认话题 | 类型 / 含义 |
|---|---|---|---|
| FS150 | `setpoint_topic` | `/<name>/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget`，PVA、mask 和 frame |
| FS150 | `attitude_topic` | `/<name>/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget`，姿态、角速度、推力和 mask |
| Scout / Mecanum | `cmd_vel_topic` | `/<name>/cmd_vel` | `geometry_msgs/Twist`，body forward/left/yaw rate；Scout 忽略 left |

### 状态与测量输出

| 机器人 | `ros` 配置键 | 默认话题 | 类型 / 含义 |
|---|---|---|---|
| 全部 | `localization_pose_topic` | `/<name>/pose` | `geometry_msgs/PoseStamped`，世界系公共定位 |
| 全部 | `localization_twist_topic` | `/<name>/twist` | `geometry_msgs/TwistStamped`，世界系速度 |
| FS150 | `pose_topic` | `/<name>/mavros/local_position/pose` | `geometry_msgs/PoseStamped`，local-origin 平移后的位姿 |
| FS150 | `velocity_topic` | `/<name>/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped`，世界轴速度 |
| FS150 | `odometry_topic` | `/<name>/mavros/local_position/odom` | `nav_msgs/Odometry`，twist 转入 body child frame |
| FS150 | `imu_topic` | `/<name>/mavros/imu/data` | `sensor_msgs/Imu`，姿态、body specific force 和 gyro |
| FS150 | `raw_imu_topic` | `/<name>/mavros/imu/data_raw` | `sensor_msgs/Imu`，specific force/gyro；orientation covariance 为 -1 |
| Scout | `raw_imu_topic` | `/<name>/imu/data_raw` | `sensor_msgs/Imu`，body specific force/gyro；orientation covariance 为 -1 |
| Mecanum | `imu_topic` | `/<name>/imu` | `sensor_msgs/Imu`，实际偏航姿态、body specific force/gyro |
| FS150 | `state_topic` | `/<name>/mavros/state` | `mavros_msgs/State`，Provider/FCU 状态 |
| FS150 | `extended_state_topic` | `/<name>/mavros/extended_state` | `mavros_msgs/ExtendedState`，落地状态 |
| FS150 | `target_attitude_topic` | `/<name>/mavros/setpoint_raw/target_attitude` | `mavros_msgs/AttitudeTarget`，实际级联控制器输出 |

同一模型步的状态输出使用同一整数时钟 stamp。`ros.mocap_noise: [sx,sy,sz]` 配置公共 pose 发布边界的高斯位置噪声，各轴标准差须为有限非负数；`ros.mocap_seed` 默认 `1`。噪声独立于物理真值和本地 FCU 反馈；MAVROS local、IMU 和 twist 保持无测量噪声。`/<name>/{pose,twist}` 由 xsim 直接发布，Adapter 的 xsim profile 直接订阅这些定位话题。

IMU 的 frame 为 `body_frame`，轴为前/左/上，gyro 单位 rad/s，linear_acceleration 为含重力的比力 `Rᵀ(a_world − g_world)`，单位 m/s²。两种车采用平面姿态，静止时 z 为 `+9.8066`；平面加速度由实际积分 dt、模型实际速度变化及转弯项计算。Mecanum 的直接速度模型在换速那一步表现为有限步长下的加速度，不模拟轮胎或悬架冲击。Raw IMU 不提供姿态估计，orientation covariance 首项为 `-1`。

### 遥测发布频率

`telemetry_rates_hz` 按话题组配置，全群机器人使用同一组设置；它不改变模型积分周期、话题名称或传感器 `rate_hz`。

| 键 | 默认 Hz | 发布内容 |
|---|---|---|
| `localization` | `125` | 全部机器人的公共 pose 和 twist |
| `local` | `30` | FS150 local pose、velocity 和 odometry |
| `imu` | `30` | FS150 `/mavros/imu/data`、Mecanum `/imu` |
| `imu_raw` | `30` | FS150 `/mavros/imu/data_raw`、Scout `/imu/data_raw` |
| `state` | `1` | FS150 `/mavros/state` |
| `extended_state` | `1` | FS150 `/mavros/extended_state` |
| `target` | `10` | FS150 `/mavros/setpoint_raw/target_attitude` |

值须为有限的 `0..1000` Hz，正频率的周期必须能表示为整数 ns；`0` 关闭该组的周期发布，话题仍保留。发布使用共同世界 epoch 相位的整数 deadline，不补发错过的旧样本。每组实际周期发布上限受输出快照频率约束；默认 `output_period_ns=8000000` 对应理论上限 `125` Hz。世界或输出线程落后时，实际可用快照频率还会降低；请求频率与理论 cap 不代表接收者实际收到的频率。

暂停时不重复发布冻结的数值测量。新 generation 或时间戳回退会重置发布 deadline；Provider/FCU 状态或落地状态变化会立即发布相应状态组，`state` 和 `extended_state` 设为 `0` 时也保留变化通知。频率由原生配置中的 `telemetry_rates_hz` 指定。

### FS150 服务

| `ros` 配置键 | 默认服务 | 类型 / 含义 |
|---|---|---|
| `arming_service` | `/<name>/mavros/cmd/arming` | `mavros_msgs/CommandBool`，arm/disarm |
| `mode_service` | `/<name>/mavros/set_mode` | `mavros_msgs/SetMode`，custom mode |
| `command_service` | `/<name>/mavros/cmd/command` | `mavros_msgs/CommandLong`，仅 command 400（arm/disarm） |

Arm 返回模型实际结果；空中 disarm 被拒绝，强制操作不支持。`CommandLong` 要求 `param1` 为 0 或 1，不接受 broadcast、confirmation 或无关参数。

Mode 要求 `base_mode=0` 和有效非空 `custom_mode`。模型支持 `OFFBOARD`、`POSCTL`、`ALTCTL`、`AUTO.LOITER`、`AUTO.LAND`。MAVROS `mode_sent` 表示请求已传送：有效请求即使因 OFFBOARD stream 不足或模式不受支持而被模型拒绝，也可能返回 true；实际模式以 `state` 为准。

服务响应等待 world 执行边界。超时仍未 claimed 的请求会被原子取消；已 claimed 的请求等待并返回实际完成结果。

订阅和 FCU service callback 绑定实体 ID/generation。Reset 使旧 callback、队列命令和旧传感器样本失效。无 stamp 的 Twist 不携带发送者 generation；外部发送者须先停旧 stream，再重启 Provider。外部订阅者已经排入 TCPROS 的数据无法撤回。

Provider 和 reset 使用 Unix 管理接口；`ros.provider_service`、`ros.truth_topic`、`ros.reset_service`、`ros.mocap_topic`、`ros.mocap_velocity_topic` 配置被拒绝。`pose_topic`、`velocity_topic`、`odometry_topic` 仅接受 FS150 配置。

### 传感器与世界输出

| 配置 | 默认话题 / 启用条件 | 类型 |
|---|---|---|
| `sensor.topic` | `/<name>/cloud`，实体有传感器 | `sensor_msgs/PointCloud2` |
| `sensor.publish_beams` | `/<name>/simple_lidar/beams`，显式启用且模型支持 | `sensor_msgs/PointCloud2` |
| `publish_clock` | `/clock`，为 true 且编译 ROS IO | `rosgraph_msgs/Clock` |
| `reference_cloud_topic` | 配置的名称非空 | latched `sensor_msgs/PointCloud2`，场景参考云只发布一次 |

## Unix HTTP/JSON 管理接口

宿主使用共享 XRPC HTTP 与 BootstrapInput。`GET /v1/describe` 返回真实 ServiceRef；后续请求通过共享 SDK 的 instance、request ID 和 timeout 头绑定本次进程。进程退出后该 incarnation 失效。

| 路径 | 用途 |
| --- | --- |
| `GET /v1/health`、`POST /v1/health/observe` | 原生组件 ready 与 revision 等待 |
| `GET /v1/world` | 世界时间、步数、实体与原生诊断 |
| `GET /v1/entities`、`GET /v1/entities/<id>` | 公开实体身份及状态 |
| `POST /v1/entities`、`DELETE /v1/entities/<id>` | 创建、删除实体 |
| `POST /v1/entities/<id>/state` | `{generation, state:{enabled}}` |
| `POST /v1/entities/<id>/reset` | 携带 generation 重置实体 |
| `POST /v1/world/{pause,resume,step,reset}` | 世界管理；reset 不回退时间 |
| `GET /v1/operations/<id>`、`POST /v1/operations/<id>/{wait,cancel}` | 精确操作回执、等待和取消 |

变更返回的 accepted/running 不等于完成。调用方等待终态并检查实际结果；启用不等于 Arm，取消也不抹掉已执行的物理效果。请求在固定两个冷准备 worker 之外不创建每机器人线程；运行目录和 HTTP 端点租约在原生工作停止后释放。

## ECS 数据与执行流

`src/xsim/core/` 管理唯一实体 roster、身份、组件、命令和世界边界调度；`systems/robots.*` 准备并步进 FS150、Scout、Mecanum；`systems/sensors.*` 管理传感器资源和 workers；`models/` 保存数值模型和 PX4 源；`io/config.*`、`io/native_rpc/`、`io/ros/` 管理配置与传输；`main.cpp` 组合这些具体对象。

按机器人种类分组的 dense array 和权威 body/planar SoA 列共享稳定 ID 映射。Controller/filter 状态采用连续 AoS，name/config/ROS handles 为低频数据。Remove 将该类最后一个 dense 元素移入空位并重新绑定索引。Entity 持有 IO 和可选 sensor 资源，快照仅弱引用 Entity，已移除资源不会被快照延长寿命。

ROS 和 HTTP 通过同一个边界命令执行器修改世界。Add 在 world thread 外准备模型、ROS endpoints 和 sensor 资源；Reset 在 world thread 外准备初始模型，再提交到 world boundary。GPU 初始化与静态地图上传在 GPU owner 上完成后才提交 add。

动力学、控制器与滤波器只接收共享的实际 dt，不读取操作系统时钟。Flight 控制每次更新一次，刚体保持 ≤2 ms 的安全 RK4 子步；平面接地仍在整个调用区间端点约束。增大世界步长减少控制更新，但不能消除刚体数值子步；本模型不提供复杂接触/碰撞保证。

线程分工：world 1 个，input/HTTP 1 个（main），service 2 个，output 1 个；ROS 发布默认 2 个固定分片线程，按稳定实体 ID 分配，每个机器人只有一个发布者。按需另启 CPU sensor workers（默认 2 个）和/或一个 GPU context worker。Native 资源准备和 ROS services 复用两个 service worker。ROS 自有网络线程负责传输。World 不执行 ROS publication、service wait、socket IO 或 sensor scan。

World 在每步边界按序执行离散命令，再按类型连续遍历机器人并提交实际时间。飞行控制启用标志使用紧凑数组，热循环不查 entity 散列表；只读控制参数与惯量逆矩阵在构造时缓存。连续输入可在不跨越离散操作的前提下合并，记录 `input_coalesced`。16 个预建 Frame 缓冲仅在无人持有时重写，数组容量随实体数量增长；输出、查询、传感器及发布线程共享同一不可变快照，所有 body 和 sensor pose 共用 stamp。快照池满或交换锁忙时跳过输出，物理积分继续，记录 `output_misses`。发布分片只保留最新待处理快照，替换计入 `coalesced_worker_frames`；先处理本分片遥测，再轮转完成的 cloud，每发一个 cloud 就检查更新遥测。

Noetic ROS IO 缓存非 latched Publication，序列化在分片线程执行，再交给 ROS Poll 线程的原发布队列；发送不在 World 上执行。每个话题初始 4 个 wire 缓冲，只有 ROS 释放引用后才能复用。槽位耗尽时按当前连接数 N 补至 N+4 个，保留已有容量；数据容量仅在峰值扩大时增长。达到这个容量仍无空槽时跳过本次交接，不阻塞；缓冲占用不改变采样频率。每个 TCP 连接的 ROS 待发送队列为 1，队满丢旧待发消息，在途消息仍由 ROS/TCP 完成。消息对象与 frame 字符串复用。点云 payload 直接序列化成 TCPROS 字节，不先复制到临时 PointCloud2 的 data。ROS 队列、连接和内核传输仍可分配、复制；这些复用不等于整个 ROS 进程零分配或端到端零拷贝。

点云和 beam 的 `bytes_estimate` 按交给 ROS 的序列化字节数乘连接数统计，不包含 TCP/IP 头或重传，也不代表订阅者已收到的数据。当前没有点云带宽限额或令牌桶。发布时没有可用 wire 缓冲会丢弃本次交接，并累计 `buffer_drops` 和 `backpressure_ns`，这些指标不反馈调整 Sensor 采样周期。Sensor 自身根据扫描耗时、待处理或完成样本积压和 payload 池压力独立退避；已经交给 ROS 的数据包不能撤回。

Sensor 使用可选组件及输出 Frame 采样。IO 资源随 Entity 退休。

## 传感器配置与约束

传感器使用不可变 `LidarScene`/geometry/index。场景来自 `scene` 或 `scene_file`；世界只接受配置时的固定场景，没有动态场景编辑管理端点。

| `sensor` 键 | 默认值 / 范围 | 含义 |
|---|---|---|
| `backend` | `cpu`；可选 `gpu` | 观察后端 |
| `mode` | `raycast` | CPU：`raycast`、`penetrating`、`depth`；GPU：`lidar_scan` |
| `topic` | `/<name>/cloud` | 点云主题 |
| `frame` | `world`；可选 `map` | 输出世界点坐标的 frame |
| `rate_hz` | `10`，正数 | 仿真时间上的采样频率 |
| `range`、`min_range` | `20`、`0` | 距离，m |
| `h_fov_deg`、`v_fov_deg` | `360`、`30` | 水平/垂直 FOV，deg |
| `h_res`、`v_res` | `360`、`32` | 水平/垂直采样数 |
| `translation`、`rotation` | 零平移、单位四元数 | sensor mount；rotation 顺序为 `[w,x,y,z]` |
| `noise_std`、`seed` | `0`、`0` | CPU Gaussian range noise 和随机 seed |
| `world_bodies` | `false`，仅 CPU | 观察同一步全世界机器人 body 快照 |
| `publish_beams` | `false`，仅支持 beams 的 CPU 模型 | 输出 beam 点云 |
| `surface_spacing`、`keep_buried` | 继承 scene 默认 | penetrating 采样间距与 buried 策略 |
| `heading_crop`、`heading_cos_min`、`vertical_slab_tan` | `false`、`0`、`0.5773502691896258` | penetrating 裁剪 |
| `width`、`height` | `160`、`120` | depth 图像尺寸 |
| `fx`、`fy`、`cx`、`cy` | `0` | depth pinhole 内参 |
| `point_cover_spacing` | 继承 scene spacing | GPU 静态地图点覆盖间距 |

CPU 使用 `WorldLidar` 的 raycast、penetrating 或 pinhole depth 实现。通常输出 XYZ；`world_bodies=true` 时增加 INT32 `vehicle_id`。Beam 输出包含 `x,y,z,dx,dy,dz,range,hit`，观察机器人 body 时增加 `vehicle_id`；penetrating 模型不支持 beams。

GPU 使用 spherical-nearest `lidar_scan`，输出 XYZ/intensity，共用一个静态地图上传/context，在 GPU owner thread 内切换各 sensor projection。GPU 不支持 `world_bodies`、`publish_beams` 或非零 `noise_std`。CPU/GPU 是明确不同的观察模型，不能自动互换。

GPU kernel 的两个轴共用 `polar_res`，要求 `h_fov_deg / h_res == v_fov_deg / v_res`，并满足点覆盖约束。例如 120° × 60°、240 × 120 samples 有效；相同 FOV 配 240 × 30 被拒绝。xsim 不修改 FOV/分辨率来接受不匹配请求，也不改用 CPU。

输出线程按绝对采样相位投递不可变 pose、实体 ID/generation、stamp 和 scene version 1；实际采样上限受输出 Frame 频率约束（默认 125 Hz）。每个 sensor 最多有一个 executing、一个 pending 和一个 completed 样本；共享 CPU worker / GPU owner 执行扫描，过载替换旧 pending/completed 并计数。传感器自身计算过载会增大采样周期，保持请求 Hz、FOV、分辨率、字段和后端的配置；采样周期按 25% 增量退避，上限为请求周期的 8 倍与 1 s 中的较大者；压力解除后按 250 ms 墙钟窗口逐步恢复。Linux 观察 worker 尽力使用较低的调度优先级（nice 5）。此降级不会阻塞 World。

每个 sensor 预建 4 个 payload 缓冲，计算时只写无人持有的缓冲，缓存、完成样本和发布者共享不可变数据；池满时丢弃采样。无噪声 CPU 观察仅在 sensor pose 和所观测 body 几何完全相同且 generation 未变时复用缓存，命中不复制点云/beam 字节；有噪声或运动会重新计算。静态场景索引、CPU 缓冲和 GPU 地图/context 复用。扫描完成后释放 body Frame，缓存不长期占用世界快照。发布保留采样 stamp，丢弃移除、disabled 或过期 generation 的结果。内存随配置实体、beam patterns、输出与几何缓存缓冲和共享场景增长。

`GET /v1/world` 的原生 diagnostics 中 `simulation_time_ns` 是已经积分的时间，`model_step_ns` 是名义周期，`scheduling_period_ns` 是当前调度周期，`last_dt_ns` 是最近实际积分间隔。`rtf` 为自进程起点的累计比值（包含暂停和手动 Step）；`realtime_rtf` 只计算自动运行的积分时间 / 活动墙钟时间。`frame_slots` 与 `frame_array_grows` 报告快照池及数组成长；Sensor 分别报告 requested/effective/source/observed rate、throttled samples、misses、cache hits、实际计算次数、payload 槽位/成长/丢弃。

`publication` 报告分片数、快照合并、准备耗时与错误，以及 telemetry/cloud/clock 的交接次数、估计字节、缓冲分配/容量/丢弃、序列化和队列交接耗时。`accepted` 是进入 ROS 发布队列，不是订阅者接收确认；`backpressure_ns` 是拒绝交接时所观察缓冲年龄的累计值，不是阻塞时间。
