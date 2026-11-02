# ros2_use_example

ROS 2 Humble 下 **topic / service / action** 三种通信机制的可运行示例，包含两组：

- `ros2_use_example*`：抽象的最小案例（消息 → 求和 → Fibonacci），用来理解机制本身。
- `gimbal_use_example*`：贴近业务的最小案例（云台 pose 广播、升级前检查、yaw 转动控制）。
- `auto_flight_example*`：完整工程，把 topic / service / action 串成一条自动飞行链路。

## 目录结构

```
ros2_use_example/
├── ros2_use_example_interfaces/        # 通用示例接口
│   ├── msg/ExampleMessage.msg          # header + content + count
│   ├── srv/ExampleService.srv          # a + b -> sum
│   └── action/ExampleAction.action     # order -> sequence / partial_sequence
├── ros2_use_example/                   # 通用示例节点 + launch
│   ├── src/{topic,service,action}_*.cpp
│   └── launch/{topic,service,action}_demo.launch.py
├── gimbal_use_example_interfaces/      # 云台示例接口
│   ├── msg/GimbalPose.msg              # yaw/pitch/roll + 角速度（度）
│   ├── srv/UpgradeCheck.srv            # 版本 -> 能否升级 + 原因
│   └── action/GimbalMove.action        # 相对 yaw 角度 -> 反馈/结果
├── gimbal_use_example/                 # 云台示例节点 + launch
│   ├── src/gimbal_pose_{publisher,subscriber}.cpp
│   ├── src/upgrade_check_{server,client}.cpp
│   ├── src/gimbal_move_{server,client}.cpp
│   └── launch/{gimbal_pose,upgrade_check,gimbal_move}_demo.launch.py
├── auto_flight_interfaces/             # 自动飞行接口
│   ├── srv/GetGimbalInfo.srv           # 云台型号/版本/是否需要升级
│   ├── action/AutoFlight.action        # 起飞 - 航点 - 降落
│   └── msg/FlightStatus.msg            # 飞行状态广播
└── auto_flight_example/                # 完整工程
    ├── src/drone_sim.cpp               # 发布 pose/imu/odom，接收速度指令
    ├── src/gimbal_manager.cpp          # /gimbal/get_info 服务
    ├── src/auto_flight_node.cpp        # action 服务端 + topic + service 客户端
    ├── src/auto_flight_client.cpp      # 任务 CLI
    └── launch/auto_flight_demo.launch.py
```

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/pp_proj/ros2_pro/ros2_use_example
colcon build --symlink-install
source install/setup.bash
```

接口包与节点包之间有依赖，所有包一起构建即可；之后新开终端也要先 source `/opt/ros/humble/setup.bash`，
再 source 本目录的 `install/setup.bash`。

## 三种机制对比

| 维度 | Topic | Service | Action |
| --- | --- | --- | --- |
| 通信语义 | 发布者 → 订阅者，单向 | 请求 → 应答 | 目标 → 反馈（多次）→ 结果 |
| 是否阻塞 | 否，即发即忘 | 是，客户端等待应答 | 否，三个阶段异步完成 |
| 拓扑 | 多发布者/多订阅者 | 一个服务名一个服务端 | 一个 action 名一个服务端 |
| 运行中反馈 | 无（靠消息流本身） | 无 | 有 |
| 取消 | 不适用 | 已发出的请求无法取消 | 支持 |
| 典型场景 | 传感器、状态广播 | 参数查询、一次性计算 | 导航、机械臂动作等长任务 |

---

## 1. Topic：单向数据流

`topic_publisher` 每秒向 `/example_topic` 发一条 `ExampleMessage`，`topic_subscriber` 打印收到的消息。

```bash
# 两个终端分别运行
ros2 run ros2_use_example topic_subscriber
ros2 run ros2_use_example topic_publisher

# 或者一条命令
ros2 launch ros2_use_example topic_demo.launch.py
```

预期输出（订阅者）：

```text
[topic_subscriber]: Received: count=2 frame_id='map' content='hello from topic_publisher'
[topic_publisher]: Published: count=3
[topic_subscriber]: Received: count=3 frame_id='map' content='hello from topic_publisher'
```

关键 API：

| 方向 | API |
| --- | --- |
| 发布 | `create_publisher<T>("example_topic", 10)` + `publish(msg)` |
| 订阅 | `create_subscription<T>("example_topic", 10, callback)` |

命令行观察：

```bash
ros2 topic list
ros2 topic echo /example_topic
ros2 topic hz /example_topic
```

要点：

- 发布者和订阅者互不感知：没有订阅者时发布不会报错，订阅者晚启动也收不到历史消息。
- 两端 QoS 必须兼容（示例都用默认的 reliable + depth 10），否则能发现对方但收不到数据。

---

## 2. Service：请求/应答

`service_server` 提供 `/example_service`，计算 `a + b`；`service_client` 发一次请求后退出。

```bash
# 终端 A：服务端常驻
ros2 run ros2_use_example service_server

# 终端 B：客户端（参数 a b，缺省 2 3）
ros2 run ros2_use_example service_client 7 5

# 或者一条命令
ros2 launch ros2_use_example service_demo.launch.py a:=7 b:=5
```

预期输出：

```text
[service_client]: Sending request: 7 + 5
[service_server]: Request: 7 + 5 = 12
[service_client]: Response: sum=12
```

不写客户端也能测，服务端保持运行时执行：

```bash
ros2 service call /example_service ros2_use_example_interfaces/srv/ExampleService "{a: 7, b: 5}"
```

关键 API：

| 角色 | API |
| --- | --- |
| 服务端 | `create_service<T>("example_service", callback)`，在回调里填 `response` |
| 客户端 | `create_client<T>()` → `wait_for_service()` → `async_send_request(req).future.share()` → `spin_until_future_complete()` |

要点：

- 一次请求恰好对应一次应答，没有“流”的概念。
- 服务端回调里做耗时操作会占住执行器（本例是立即返回；长任务应该用 action）。

---

## 3. Action：长任务

`action_server` 计算 Fibonacci 前 `order` 个数，执行过程中每 0.5 s 发布一次部分序列作为反馈；
`action_client` 发送目标、打印反馈、打印结果，并支持中途取消。

```bash
# 终端 A：action 服务端常驻
ros2 run ros2_use_example action_server

# 终端 B：正常执行
ros2 run ros2_use_example action_client 8

# 终端 B：1.5 秒后取消
ros2 run ros2_use_example action_client 30 1.5

# 或者一条命令
ros2 launch ros2_use_example action_demo.launch.py order:=8 cancel_after:=0.0
```

预期输出（正常完成）：

```text
[action_client]: Goal accepted: order=8
[action_client]: Feedback: 3 numbers, last=1
[action_client]: Feedback: 4 numbers, last=2
...
[action_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13]
```

预期输出（取消）：

```text
[action_client]: Requesting cancellation
[action_client]: Result: canceled, partial sequence [0, 1, 1, 2, 3]
```

不写客户端也能测，`--feedback` 会持续打印反馈：

```bash
ros2 action list
ros2 action info /example_action
ros2 action send_goal /example_action ros2_use_example_interfaces/action/ExampleAction "{order: 8}" --feedback
```

关键 API：

| 阶段 | 服务端 | 客户端 |
| --- | --- | --- |
| 目标 | `handle_goal` 返回 ACCEPT / REJECT | `async_send_goal(goal, options)` |
| 反馈 | `publish_feedback(feedback)` | `options.feedback_callback` |
| 结果 | `succeed(result)` / `canceled(result)` / `abort(result)` | `async_get_result(goal_handle)` |
| 取消 | `handle_cancel` 返回 ACCEPT / REJECT | `async_cancel_goal(goal_handle)` |

要点：

- action 由三部分组成：goal（一次）、feedback（零到多次）、result（一次）。
- 示例中 `execute()` 会 sleep，因此放进独立线程执行；否则会堵住执行器，取消请求也处理不了。
- 客户端要区分三种结局：`SUCCEEDED` / `CANCELED` / `ABORTED`，以及目标被 `REJECT`（`goal_handle` 为空）。

---

## 4. 云台实战

前面三节是抽象案例；下面三个用同一套机制对应真实云台业务。

### 4.1 Topic：云台 pose 发布/订阅（两个进程）

`gimbal_pose_publisher` 在 `/gimbal_pose` 上以 20 Hz 发布模拟的 yaw/pitch/roll 及角速度（单位：度），
`gimbal_pose_subscriber` 是另一个进程，只通过这个 topic 接收姿态数据。

```bash
# 终端 A：订阅进程
ros2 run gimbal_use_example gimbal_pose_subscriber

# 终端 B：发布进程
ros2 run gimbal_use_example gimbal_pose_publisher

# 或者一条命令
ros2 launch gimbal_use_example gimbal_pose_demo.launch.py
```

预期输出（订阅端，日志限流到 2 Hz）：

```text
[gimbal_pose_subscriber]: Listening on /gimbal_pose
[gimbal_pose_subscriber]: Pose: yaw=24.95 pitch=12.78 roll=0.77 deg (yaw_rate=8.33 deg/s)
[gimbal_pose_subscriber]: Pose: yaw=28.49 pitch=13.41 roll=0.96 deg (yaw_rate=4.70 deg/s)
[gimbal_pose_subscriber]: Pose: yaw=29.96 pitch=13.96 roll=1.14 deg (yaw_rate=0.73 deg/s)
```

命令行观察 / 调整：

```bash
ros2 topic echo /gimbal_pose
ros2 topic hz /gimbal_pose
ros2 param set /gimbal_pose_publisher yaw_amplitude_deg 60.0
```

要点：

- 发布进程和订阅进程完全解耦：任一进程重启，另一个都不受影响；没有订阅者时发布照常进行。
- 控制环通常只取“最新一帧”姿态，日志要限流，否则 20 Hz 打印会淹没终端。

### 4.2 Service：升级前 check（一问一答）

`upgrade_check_server` 提供 `/gimbal_upgrade_check`：客户端带上当前版本和目标版本，
服务端检查电池、温度、云台是否静止，一次性回答“能否升级 + 原因”。

| 方向 | 字段 |
| --- | --- |
| 请求 | `current_version`、`target_version`、`force`（跳过非安全检查） |
| 应答 | `can_upgrade`、`reason`、`estimated_duration_sec` |

```bash
# 终端 A：服务端常驻
ros2 run gimbal_use_example upgrade_check_server

# 终端 B：询问 1.0.0 -> 1.1.0
ros2 run gimbal_use_example upgrade_check_client 1.0.0 1.1.0

# 或者一条命令
ros2 launch gimbal_use_example upgrade_check_demo.launch.py current_version:=1.0.0 target_version:=1.1.0
```

预期输出：

```text
[upgrade_check_client]: Requesting check: 1.0.0 -> 1.1.0 (force=false)
[upgrade_check_server]: Check: 1.0.0 -> 1.1.0 (force=false, battery=80%, temp=45 C, idle=true)
[upgrade_check_client]: Upgrade approved: checks passed, ready to upgrade to 1.1.0 (~30 s)
```

服务端把模拟的健康状态暴露成参数，可以现场构造拒绝场景：

```bash
ros2 param set /upgrade_check_server battery_percent 20.0
ros2 run gimbal_use_example upgrade_check_client 1.0.0 1.1.0
# [WARN] Upgrade refused: battery too low: 20% < 30%

ros2 run gimbal_use_example upgrade_check_client 1.0.0 1.1.0 --force
# [INFO] Upgrade approved: checks passed, ready to upgrade to 1.1.0 (~30 s)

ros2 run gimbal_use_example upgrade_check_client 1.1.0 1.1.0
# [WARN] Upgrade refused: gimbal is already on 1.1.0
```

也可以直接命令行调用：

```bash
ros2 service call /gimbal_upgrade_check gimbal_use_example_interfaces/srv/UpgradeCheck \
  "{current_version: '1.0.0', target_version: '1.1.0', force: false}"
```

要点：

- check 是“一次请求、一次应答”，用 service 最直接；客户端退出码 0 表示允许升级、2 表示拒绝。
- 真正的升级过程耗时长且需要进度/取消，应该用 action 下发，而不是让 service 回调一直阻塞。

### 4.3 Action：yaw 转动控制

`gimbal_move_server` 提供 `/gimbal_move`：客户端请求“yaw 相对转动 +30°”，
服务端接受目标后按固定角速度转动，过程中持续反馈当前角度和进度，结束后回复成功；
也支持中途取消，超出软限位或正在运动时会拒绝目标。

| 阶段 | 字段 |
| --- | --- |
| goal | `delta_yaw_deg`、`speed_dps`（<=0 用服务端默认速度） |
| feedback | `current_yaw_deg`、`progress`（0~1） |
| result | `success`、`final_yaw_deg`、`message` |

```bash
# 终端 A：action 服务端常驻
ros2 run gimbal_use_example gimbal_move_server

# 终端 B：yaw +30°
ros2 run gimbal_use_example gimbal_move_client 30

# 以 60 deg/s 转 +30°
ros2 run gimbal_use_example gimbal_move_client 30 60

# 转 +90°，1 秒后取消
ros2 run gimbal_use_example gimbal_move_client 90 30 1.0

# 或者一条命令
ros2 launch gimbal_use_example gimbal_move_demo.launch.py delta_yaw_deg:=30.0
```

预期输出（+30°，默认 30 deg/s）：

```text
[gimbal_move_client]: Sending goal: delta_yaw=30.0 deg, speed=0.0 deg/s
[gimbal_move_client]: Goal accepted
[gimbal_move_client]: Feedback: yaw=0.0 deg progress=0%
[gimbal_move_client]: Feedback: yaw=3.0 deg progress=10%
...
[gimbal_move_client]: Feedback: yaw=30.0 deg progress=100%
[gimbal_move_client]: Result: success, final yaw=30.0 deg (yaw moved to 30 deg)
```

取消与拒绝：

```text
# 90°，1 秒后取消
[gimbal_move_client]: Requesting cancellation
[gimbal_move_client]: Result: canceled at yaw=30.0 deg (canceled while moving)

# 请求 +200°，超出 [-170, 170] 软限位
[gimbal_move_client]: Goal was rejected by the server
```

命令行也可以直接发目标：

```bash
ros2 action send_goal /gimbal_move gimbal_use_example_interfaces/action/GimbalMove \
  "{delta_yaw_deg: 30.0, speed_dps: 0.0}" --feedback
```

要点：

- 服务端维护当前 yaw 和软限位，同一时刻只执行一个动作，第二个目标直接 `REJECT`。
- 转动是耗时操作，反馈让上位机能看到进度，取消让操作员能随时停下。
- 实际项目中 action 服务端通常还会把姿态发到 pose topic，本示例把两者拆开以免互相干扰。

---

## 5. 完整工程：自动飞行

`auto_flight_example` 把三种机制串成一条自动飞行链路，自包含、不依赖 PX4 / Webots / 真机：

```text
gimbal_manager  ──/gimbal/get_info (service)──────────▶ auto_flight_node
drone_sim       ──/drone/{pose,imu,odom} (topic)─────▶ auto_flight_node
drone_sim       ◀─/drone/cmd_vel (topic)────────────── auto_flight_node
auto_flight_node──/auto_flight/status (topic)────────▶ 地面站 / 日志
auto_flight_client ──/auto_flight/execute (action)───▶ auto_flight_node
```

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/gimbal/get_info` | `GetGimbalInfo`（service） | 返回型号、序列号、固件版本、要求版本，以及 `upgrade_required` + 原因 |
| `/auto_flight/execute` | `AutoFlight`（action） | 目标：起飞高度 + 航点；反馈：状态/当前航点/剩余距离；结果：成功、航点数、里程、耗时 |
| `/auto_flight/preflight` | `std_srvs/Trigger`（service） | 起飞前自检：传感器、姿态、云台固件 |
| `/auto_flight/status` | `FlightStatus`（topic） | 2 Hz 广播：状态、云台版本、是否需升级、传感器健康、航点进度 |

| 话题 | 类型 | 方向 |
| --- | --- | --- |
| `/drone/pose` | `geometry_msgs/PoseStamped` | 仿真 → 规划器 |
| `/drone/imu` | `sensor_msgs/Imu` | 仿真 → 规划器 |
| `/drone/odom` | `nav_msgs/Odometry` | 仿真 → 规划器 |
| `/drone/cmd_vel` | `geometry_msgs/Twist` | 规划器 → 仿真 |

```bash
# 一键启动（仿真 + 云台 + 规划器 + 一个方形任务）
ros2 launch auto_flight_example auto_flight_demo.launch.py

# 或手动分进程
ros2 run auto_flight_example drone_sim
ros2 run auto_flight_example gimbal_manager
ros2 run auto_flight_example auto_flight_node
ros2 run auto_flight_example auto_flight_client 3 5   # 起飞高度 3 m，方形边长 5 m
```

预期输出：

```text
[auto_flight_node]: Preflight passed: gimbal Z6 Z6-0001, firmware 1.2.0
[auto_flight_client]: Mission accepted
[auto_flight_client]: Feedback: TAKEOFF wp 0/4 remaining 0.0 m
[auto_flight_client]: Feedback: CRUISE wp 1/4 remaining 19.1 m
...
[auto_flight_client]: Feedback: LAND wp 4/4 remaining 0.0 m
[auto_flight_client]: Mission succeeded: 4/4 waypoints, 18.8 m flown in 18.8 s
```

观察运行状态：

```bash
ros2 topic echo /auto_flight/status --once
ros2 topic hz /drone/pose
ros2 service call /gimbal/get_info auto_flight_interfaces/srv/GetGimbalInfo "{}"
ros2 service call /auto_flight/preflight std_srvs/srv/Trigger "{}"
ros2 action list
```

异常与边界场景（均已实测）：

| 场景 | 操作 | 结果 |
| --- | --- | --- |
| 云台固件落后 | `firmware_version:=1.0.0` 启动 | preflight 返回 NOT ready；mission 直接 abort，原因写明需升级 |
| 旧固件放行 | `ros2 param set /auto_flight_node require_gimbal_up_to_date false` | 任务正常执行 |
| 传感器缺失 | 不启动 `drone_sim` | preflight 报 pose/imu/odom stale，mission abort |
| 中途取消 | `auto_flight_client 3 5 6` | 原地降落，action 返回 CANCELED，退出码 2 |
| 目标非法 | 高度 100 m 或航点半径 60 m | action 直接 REJECT（超高度限制 / 超出地理围栏） |

设计要点：

- **升级判断在云台侧**：`GetGimbalInfo` 自己比较 `firmware_version` 与 `required_version`，飞行器只按 `upgrade_required` 决定放行策略。
- **服务轮询 + 缓存**：`auto_flight_node` 以 2 Hz 轮询云台信息并缓存，`preflight` 服务因此不会阻塞 executor；任务线程等待缓存变新鲜即可。
- **任务互斥**：同一时刻只允许一个 mission（`mission_active`），第二个目标直接 REJECT。
- **取消即降落**：取消后先原地降落，再返回 `CANCELED`，不会把飞机悬停在空中。
- **仿真与真机的边界**：仿真为运动学 ENU（z 向上），与 PX4 的 NED 不同；接真机时替换 `drone_sim`，并按 NED 调整速度符号和话题（`/fmu/in/...`）。

---

## 自定义接口

通用示例在 `ros2_use_example_interfaces`，云台示例在 `gimbal_use_example_interfaces`：

```text
msg/X.msg      # 单段：字段列表
srv/X.srv      # 两段：请求 --- 应答
action/X.action # 三段：goal --- result --- feedback
```

| 包 | 接口 |
| --- | --- |
| `ros2_use_example_interfaces` | `ExampleMessage` / `ExampleService` / `ExampleAction` |
| `gimbal_use_example_interfaces` | `GimbalPose` / `UpgradeCheck` / `GimbalMove` |
| `auto_flight_interfaces` | `FlightStatus` / `GetGimbalInfo` / `AutoFlight` |

查看生成后的完整定义：

```bash
ros2 interface show gimbal_use_example_interfaces/msg/GimbalPose
ros2 interface show gimbal_use_example_interfaces/srv/UpgradeCheck
ros2 interface show gimbal_use_example_interfaces/action/GimbalMove
```

两个接口包都用了 `std_msgs/Header`，通过 `DEPENDENCIES std_msgs` 声明该依赖。
修改 `.msg` / `.srv` / `.action` 后需要重新 `colcon build`，接口代码才会更新。

## 常见问题

- **`Package 'ros2_use_example' not found`**：没有 source `install/setup.bash`，或者 `/opt/ros/humble` 与本目录的 source 顺序反了。
- **`colcon build` 找不到包**：命令要在 `ros2_use_example/` 目录下执行（两个包都在其下一层）。
- **客户端跑完就退出**：`service_client` / `action_client` 只发一次请求，重复调用需要重新 `ros2 run`。
- **topic 收不到数据**：确认 QoS 一致；另外默认 QoS 下订阅者只能收到订阅之后发布的消息。
