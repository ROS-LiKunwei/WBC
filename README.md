# ik_7dof 包说明

## 项目概述

`ik_7dof` 是一个面向 FA 机器人上肢的 ROS 2 正逆运动学包，当前主要包含：

- 基于 Pinocchio 的左右臂正运动学与逆运动学求解
- 支持 `pelvis` / `arm_base` 两种参考坐标系
- 纯数值一致性验证节点
- 基于 MoveIt 2 和 RViz 的可视化验证节点
- 随机测试、性能统计和失败样本记录

当前仓库中与上肢 IK 直接相关的核心代码主要是：

- `include/ik_7dof/fa_ik_solver.hpp`
- `src/fa_ik_solver.cpp`
- `src/fa_arm_kinematic_node.cpp`
- `src/arm_ik_rviz_node.cpp`

## fa_ik_solver 说明

### 核心能力

`fa_ik_solver` 负责左右臂 7 自由度关节的正逆运动学求解。

- 正运动学：给定 7 维关节角，输出末端位姿
- 逆运动学：给定目标位姿，求解 7 维关节角
- 限位约束：求解时考虑 URDF 中的关节上下限
- 双阶段求解：先用 `LDLT` 快速求解，失败后再用 `SVD + 随机重启` 提高稳健性
- 双臂支持：统一支持左臂和右臂

### 参考坐标系

新版接口增加了参考坐标系选项：

- `ArmReferenceFrame::PELVIS`
- `ArmReferenceFrame::ARM_BASE`

对应配置结构体：

```cpp
fa_arm_kinematic::ArmKinematicsOptions options;
options.reference_frame = fa_arm_kinematic::ArmReferenceFrame::PELVIS;
```

说明：

- `PELVIS`：FK 输出和 IK 输入都以 `pelvis` 为参考
- `ARM_BASE`：FK 输出和 IK 输入都以手臂基坐标系为参考
- 当前实现中腰部关节不参与手臂 IK 求解，内部会使用预计算的 `pelvis -> arm_base` 齐次变换做坐标转换

## 对外接口

### 1. 实例化求解器

```cpp
#include "ik_7dof/fa_ik_solver.hpp"

std::string urdf_file = "/path/to/fa_robot.urdf";
std::string srdf_file = "/path/to/fa_robot.srdf";
fa_arm_kinematic::IKSolver solver(urdf_file, srdf_file);
```

### 2. 正运动学

```cpp
#include "ik_7dof/fa_ik_solver.hpp"

using namespace fa_arm_kinematic;

Eigen::VectorXd q = Eigen::VectorXd::Zero(7);

ArmKinematicsOptions options;
options.reference_frame = ArmReferenceFrame::PELVIS;

PoseSE3 fk_se3 = solver.computeArmFK_SE3(q, ArmSide::LEFT, options);
PoseRPY fk_rpy = solver.computeArmFK(q, ArmSide::LEFT, options);
```

### 3. 逆运动学

```cpp
#include "ik_7dof/fa_ik_solver.hpp"

using namespace fa_arm_kinematic;

ArmKinematicsOptions options;
options.reference_frame = ArmReferenceFrame::PELVIS;

pinocchio::SE3 T_target;
T_target.translation(Eigen::Vector3d(0.25, 0.30, 0.45));
T_target.rotation(Eigen::Matrix3d::Identity());

IKResult result = solver.solveArmIK(
    T_target,
    ArmSide::LEFT,
    Eigen::VectorXd(),
    options,
    100,
    1e-3
);
if (result.has_solution) {
    Eigen::VectorXd q_solved = result.q_solution;
}
```

### 4. 指定底层求解方法

```cpp
#include "ik_7dof/fa_ik_solver.hpp"

using namespace fa_arm_kinematic;

ArmKinematicsOptions options;
options.reference_frame = ArmReferenceFrame::PELVIS;

int iters = 0;
Eigen::VectorXd q_init = Eigen::VectorXd::Zero(7);

Eigen::VectorXd q_ldlt = solver.solveIK_Core(
    T_target,
    q_init,
    200,
    1e-3,
    iters,
    SolverMethod::LDLT,
    ArmSide::LEFT,
    options
);

Eigen::VectorXd q_svd = solver.solveIK_Core(
    T_target,
    q_init,
    200,
    1e-3,
    iters,
    SolverMethod::SVD,
    ArmSide::LEFT,
    options
);
```

### 5. 获取关节信息

```cpp
const auto& left_joint_names = solver.getArmJointNames(fa_arm_kinematic::ArmSide::LEFT);
auto left_limits = solver.getArmJointLimits(fa_arm_kinematic::ArmSide::LEFT);
```

## 测试节点

### 1. `fa_arm_kinematic_node`

纯数值验证节点，用于离线检查 FK/IK 一致性。

功能：

- 随机采样关节角
- 用采样关节角计算 FK
- 以 FK 结果作为目标位姿再做 IK
- 对 IK 结果再次做 FK，比较位置和姿态误差
- 输出成功率、平均耗时、平均迭代步数
- 将失败样本记录到日志文件

### 2. `arm_ik_rviz_node`

MoveIt 2 / RViz 可视化验证节点。

功能：

- 先移动到 `leftArmHome` 或 `rightArmHome`
- 随机采样关节角并计算 FK/IK
- 将 IK 求解结果发送给 MoveIt 执行
- 在 RViz 中发布末端 Marker 观察位姿
- 支持 `pelvis` / `arm_base` 两种参考坐标系

## 构建方式

```bash
cd /home/likunwei/humanoid_ws
colcon build --packages-select ik_7dof
source install/setup.bash
```

## 运行方式

### 1. 纯数值验证

左臂，`pelvis` 坐标系：

```bash
ros2 run ik_7dof fa_arm_kinematic_node --ros-args \
  -p urdf_file:=/home/likunwei/humanoid_ws/src/sysmo_description/urdf/fa_robot.urdf \
  -p srdf_file:=/home/likunwei/humanoid_ws/src/fa_moveit2_config/config/fa_robot.srdf \
  -p arm_side:=left \
  -p reference_frame:=pelvis \
  -p num_tests:=100 \
  -p max_iters:=500 \
  -p eps:=1e-3
```

左臂，`arm_base` 坐标系：

```bash
ros2 run ik_7dof fa_arm_kinematic_node --ros-args \
  -p urdf_file:=/home/likunwei/humanoid_ws/src/sysmo_description/urdf/fa_robot.urdf \
  -p srdf_file:=/home/likunwei/humanoid_ws/src/fa_moveit2_config/config/fa_robot.srdf \
  -p arm_side:=left \
  -p reference_frame:=arm_base \
  -p num_tests:=100 \
  -p max_iters:=1000 \
  -p eps:=1e-3
```

### 2. RViz / MoveIt 验证

```bash
ros2 run ik_7dof arm_ik_rviz_node --ros-args \
  -p urdf_file:=/home/likunwei/humanoid_ws/src/sysmo_description/urdf/fa_robot.urdf \
  -p srdf_file:=/home/likunwei/humanoid_ws/src/fa_moveit2_config/config/fa_robot.srdf \
  -p arm_side:=left \
  -p reference_frame:=pelvis \
  -p num_tests:=5 \
  -p max_iters:=1000 \
  -p eps:=1e-3 \
  -p move_delay:=1.0
```

如果你使用 launch 文件，也建议在 launch 中同步补充 `reference_frame` 参数透传。

## 参数说明

### `fa_arm_kinematic_node`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `urdf_file` | string | 空 | URDF 路径 |
| `srdf_file` | string | 空 | SRDF 路径 |
| `arm_side` | string | `left` | `left` 或 `right` |
| `reference_frame` | string | `pelvis` | `pelvis` 或 `arm_base` |
| `num_tests` | int | `10` | 随机测试次数 |
| `max_iters` | int | `200` | 单次 IK 最大迭代次数 |
| `eps` | double | `1e-3` | 收敛阈值 |

### `arm_ik_rviz_node`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `urdf_file` | string | 空 | URDF 路径 |
| `srdf_file` | string | 空 | SRDF 路径 |
| `arm_side` | string | `left` | `left` 或 `right` |
| `reference_frame` | string | `pelvis` | `pelvis` 或 `arm_base` |
| `num_tests` | int | `5` | 随机测试次数 |
| `max_iters` | int | `1000` | 单次 IK 最大迭代次数 |
| `eps` | double | `1e-3` | 收敛阈值 |
| `move_delay` | double | `1.0` | 每次执行后的观察间隔 |

## 代码结构

```text
ik_7dof/
├── CMakeLists.txt
├── package.xml
├── include/ik_7dof/
│   └── fa_ik_solver.hpp
├── src/
│   ├── fa_ik_solver.cpp
│   ├── fa_arm_kinematic_node.cpp
│   └── arm_ik_rviz_node.cpp
├── launch/
│   ├── fa_arm_kinematic_verify.launch.py
│   ├── arm_ik_rviz.launch.py
│   └── left_arm_ik_test.launch.py
└── scripts/
    ├── leg_ik_mujoco_verify.py
    ├── leg_ik_py.py
    └── leg_ik_usage_and_installation.md
```

## 依赖项

- ROS 2 Humble
- Pinocchio
- Eigen
- MoveIt 2
- moveit_msgs
- tf2_ros
- geometry_msgs
- sensor_msgs
- srdfdom
- rviz2

## 注意事项

- 请确保 `URDF` 和 `SRDF` 路径有效
- `arm_ik_rviz_node` 依赖 `move_group`、控制器和 RViz 环境正常启动
- `arm_base` 模式当前已经完成接口适配与测试链路接通，但在随机目标下 IK 仍可能出现收敛失败，这更适合继续从求解器数值稳定性角度排查
- Marker 发布坐标系与 `reference_frame` 保持一致：
  - `pelvis` 模式发布在 `pelvis`
  - `arm_base` 模式发布在 `waist_pitch_link`

## 示例输出

```text
[INFO] [fa_arm_kinematic_node]: ========== FA左臂正逆解验证开始 (pelvis) ==========
[INFO] [fa_arm_kinematic_node]: --- 测试 1/100 ---
[INFO] [fa_arm_kinematic_node]: 随机目标关节角:
[INFO] [fa_arm_kinematic_node]:   left_shoulder_pitch_joint: 2.523365 rad
...
[INFO] [fa_arm_kinematic_node]: FK结果(pelvis): position=[-0.216606, 0.867833, 0.576374]
[INFO] [fa_arm_kinematic_node]: IK求解结果:
...
[INFO] [fa_arm_kinematic_node]: 一致性验证: pos_error=0.00000410 m, rot_error=0.00052139 rad
[INFO] [fa_arm_kinematic_node]:   ✅ 正逆解一致，耗时 6.0089 ms，迭代 659 步
```

## 故障排除

- `Frame not found`：检查 `pelvis`、末端 frame 和 arm base frame 名称是否与 URDF 一致
- `逆运动学求解失败`：先确认目标位姿是否在工作空间内，再检查 `reference_frame` 是否与输入目标位姿一致
- `MoveIt 不执行`：检查规划组名称、控制器和 `move_group` 是否正常运行
- `命名状态不存在`：检查 SRDF 中是否定义了 `leftArmHome` / `rightArmHome`

## 许可证

本项目采用 MIT 许可证。
