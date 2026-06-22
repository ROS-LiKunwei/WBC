# min_snap

`min_snap` 是一个独立 ROS2 Humble C++ 功能包，用于把 FA 上肢左右臂关节目标转换成满足速度、加速度约束的七次 minimum snap 轨迹，并发布 FA 原生 16 维上肢位置命令。

本包不包含 IK、VR 手势解析或 beavr-bot 内部逻辑。它只处理：

```text
左右臂 7+7 关节目标 -> minimum snap 轨迹 -> 16 维 FA 上肢位置命令
```

## Topic

订阅：

- `/min_snap/target`
  - 类型：`min_snap/msg/MinSnapTarget`
  - 遥操作或测试程序发布左右臂目标关节角。
- `/joint_states`
  - 类型：`sensor_msgs/msg/JointState`
  - 用于获取起点状态、neck 位置和 CSV 中的实际轨迹。

发布：

- `/upper_position_controller/commands`
  - 类型：`std_msgs/msg/Float64MultiArray`
  - 长度固定 16，直接对接 FA 上肢位置控制器。
- `/min_snap/desired_joint_states`
  - 类型：`sensor_msgs/msg/JointState`
  - RViz 可订阅该 topic 查看 minimum snap 期望姿态。

## FA 16 维命令顺序

```text
data[0]  = left_shoulder_pitch_joint
data[1]  = left_shoulder_roll_joint
data[2]  = left_shoulder_yaw_joint
data[3]  = left_elbow_joint
data[4]  = left_wrist_yaw_joint
data[5]  = left_wrist_pitch_joint
data[6]  = left_wrist_roll_joint

data[7]  = right_shoulder_pitch_joint
data[8]  = right_shoulder_roll_joint
data[9]  = right_shoulder_yaw_joint
data[10] = right_elbow_joint
data[11] = right_wrist_yaw_joint
data[12] = right_wrist_pitch_joint
data[13] = right_wrist_roll_joint

data[14] = neck_yaw_joint
data[15] = neck_pitch_joint
```

minimum snap 只作用于前 14 个手臂关节。neck 两个关节保持 `/joint_states` 中的当前值；如果 `/joint_states` 不包含 neck，则使用 `neck_default_position`。

## 输入消息

`msg/MinSnapTarget.msg`：

```text
float64[] left_arm_target_rad
float64[] right_arm_target_rad
float64 expected_duration_s
float64 max_velocity_rad_s
float64 max_acceleration_rad_s2
```

要求：

- `left_arm_target_rad` 长度必须是 7。
- `right_arm_target_rad` 长度必须是 7。
- 目标数组不能包含 NaN 或 Inf。
- `max_velocity_rad_s > 0`。
- `max_acceleration_rad_s2 > 0`。
- `expected_duration_s <= 0` 时使用参数 `default_expected_duration_s`，并打印 warning。

## Minimum Snap 边界条件

每个关节独立求解七次多项式：

```text
q(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5 + a6 t^6 + a7 t^7
```

起点：

```text
q(0)       = 当前轨迹位置或 /joint_states 位置
q_dot(0)   = 当前轨迹速度或 /joint_states.velocity
q_ddot(0)  = 当前轨迹加速度，新轨迹首次规划时为 0
q_jerk(0)  = 0
```

终点：

```text
q(T)       = 目标关节角
q_dot(T)   = 0
q_ddot(T)  = 0
q_jerk(T)  = 0
```

实现中固定：

```text
a0 = q0
a1 = v0
a2 = 0.5 * a0_acc
a3 = 0
```

然后求解 `a4..a7`。

## 执行时间为什么会变长

输入的 `expected_duration_s` 是期望值，不是硬约束。节点会保证最终轨迹满足：

```text
max(|q_dot(t)|)  <= max_velocity_rad_s
max(|q_ddot(t)|) <= max_acceleration_rad_s2
```

初始 duration 会先套用经验约束：

```text
T >= abs(delta) * 2.1875 / max_velocity_rad_s
T >= sqrt(abs(delta) * 7.513188404399293 / max_acceleration_rad_s2)
```

之后再对轨迹做密集采样检查。如果超限，会按 `duration_scale_on_violation` 继续放大 duration，直到满足约束或达到最大迭代次数。

因此最终执行时间可能大于用户输入的 `expected_duration_s`。这是为了避免把过短、过激的轨迹直接下发给电机。

## 在线重规划

轨迹执行过程中收到新的 `/min_snap/target` 时，节点会立即重规划。

重规划起点来自当前正在执行的轨迹采样状态：

```text
start_position     = 当前轨迹 position
start_velocity     = 当前轨迹 velocity
start_acceleration = 当前轨迹 acceleration
start_jerk         = 0
```

这样可以保持位置、速度、加速度连续。jerk 不保证跨段连续。

如果当前没有正在执行的轨迹，则从 `/joint_states` 读取起点位置和速度；如果 velocity 字段缺失，则速度按 0 处理。

## 参数

默认参数文件：

```bash
/home/likunwei/humanoid_ws/src/min_snap/config/min_snap.yaml
```

关键参数：

```yaml
publish_hz: 200.0
target_topic: "/min_snap/target"
command_topic: "/upper_position_controller/commands"
desired_joint_states_topic: "/min_snap/desired_joint_states"
joint_states_topic: "/joint_states"

min_duration_s: 0.01
default_expected_duration_s: 0.5
default_max_velocity_rad_s: 0.25
default_max_acceleration_rad_s2: 0.25

duration_scale_on_violation: 1.1
max_duration_search_iterations: 50
constraint_sample_count: 500

replan_threshold_rad: 0.0005
require_joint_state_before_start: true
joint_state_timeout_s: 0.5

record_tracking: true
tracking_output_dir: "/home/likunwei/humanoid_ws/src/min_snap/logs"
neck_default_position: [0.0, 0.0]
```

## 编译

```bash
cd /home/likunwei/humanoid_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select min_snap --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 运行

```bash
ros2 run min_snap min_snap_node --ros-args \
  --params-file /home/likunwei/humanoid_ws/src/min_snap/config/min_snap.yaml
```

## 测试发布目标

先保证 `/joint_states` 中存在 14 个 FA 手臂关节。然后发布测试目标：

```bash
ros2 topic pub --once /min_snap/target min_snap/msg/MinSnapTarget "{
  left_arm_target_rad: [0.0, 0.2, 0.0, -0.5, 0.0, 0.0, 0.0],
  right_arm_target_rad: [0.0, -0.2, 0.0, -0.5, 0.0, 0.0, 0.0],
  expected_duration_s: 0.5,
  max_velocity_rad_s: 0.25,
  max_acceleration_rad_s2: 0.25
}"
```

检查输出：

```bash
ros2 topic echo /upper_position_controller/commands
ros2 topic echo /min_snap/desired_joint_states
```

## 与 VR 遥操作通信

VR 遥操作侧应继续负责手势输入、坐标变换和 IK。IK 产生左右臂 7 维关节目标后，发布到：

```bash
/min_snap/target
```

本节点接收该关节目标并输出 `/upper_position_controller/commands`。这样轨迹优化从遥操作主进程中解耦，VR 侧只关心目标关节角，轨迹节点负责 minimum snap、速度/加速度约束和实际下发。

## RViz

节点发布：

```bash
/min_snap/desired_joint_states
```

如果只想查看期望姿态，可以让 RViz/robot_state_publisher 使用该 topic，或用 topic remap 把它映射为 `/joint_states`。真实硬件运行时不要把期望状态覆盖真实 `/joint_states`。

## CSV 记录

开启参数：

```yaml
record_tracking: true
tracking_output_dir: "/home/likunwei/humanoid_ws/src/min_snap/logs"
```

CSV 文件名：

```text
min_snap_tracking_YYYYMMDD_HHMMSS.csv
```

字段：

```text
time_s
joint_states.position.<joint>
joint_states.velocity.<joint>
desired.position.<joint>
desired.velocity.<joint>
desired.acceleration.<joint>
desired.jerk.<joint>
error.position.<joint>
```

`error.position = joint_states.position - desired.position`。

## 绘图

```bash
python3 /home/likunwei/humanoid_ws/src/min_snap/scripts/plot_min_snap_tracking.py \
  /home/likunwei/humanoid_ws/src/min_snap/logs/min_snap_tracking_YYYYMMDD_HHMMSS.csv
```

只画左臂：

```bash
python3 scripts/plot_min_snap_tracking.py <csv_file> --groups left
```

只画右臂：

```bash
python3 scripts/plot_min_snap_tracking.py <csv_file> --groups right
```

输出：

```text
*_left_tracking.png
*_right_tracking.png
*_neck_tracking.png
```

## 常见问题

节点收到目标但不发布命令：

- 检查 `/joint_states` 是否存在 14 个 FA 手臂关节。
- 检查 `require_joint_state_before_start` 是否为 `true`。
- 检查 `/joint_states` 是否超过 `joint_state_timeout_s`。

目标被拒绝：

- 检查左右目标数组长度是否都是 7。
- 检查目标、速度、加速度中是否有 NaN/Inf。
- `max_velocity_rad_s` 和 `max_acceleration_rad_s2` 必须大于 0。

duration 比输入更长：

- 这是预期行为，说明输入 duration 太短，无法满足速度/加速度限制。

RViz 中看不到期望姿态：

- 检查 `/min_snap/desired_joint_states` 是否有数据。
- 确认 robot_state_publisher/RViz 使用同一套 URDF 关节名。
