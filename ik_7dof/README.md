# ik_7dof

`ik_7dof` 提供 FA 机器人上肢正逆运动学求解器，以及两个常用测试节点：

- `fa_arm_kinematic_node`：纯数值正逆解一致性验证
- `arm_ik_rviz_node`：结合 MoveIt/RViz 做可视化验证与执行

## 参考坐标系

新版 `fa_ik_solver` 支持通过 `ArmKinematicsOptions` 选择参考坐标系：

- `pelvis`：以 `pelvis` 为参考坐标系
- `arm_base`：以手臂基坐标系为参考坐标系

当前测试节点通过参数 `reference_frame` 选择：

```text
reference_frame:=pelvis
reference_frame:=arm_base
```

默认值为 `pelvis`。

## fa_ik_solver 用法

```cpp
#include "ik_7dof/fa_ik_solver.hpp"

using namespace fa_arm_kinematic;

IKSolver solver(urdf_file, srdf_file);

ArmKinematicsOptions options;
options.reference_frame = ArmReferenceFrame::PELVIS;

Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
PoseSE3 fk = solver.computeArmFK_SE3(q, ArmSide::LEFT, options);

pinocchio::SE3 target;
target.translation(fk.p);
target.rotation(fk.R);

IKResult result = solver.solveArmIK(
    target, ArmSide::LEFT, Eigen::VectorXd(), options, 1000, 1e-3);
```

## 纯数值测试

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

节点输出内容包括：

- 关节名称和限位
- 每次测试的随机目标关节角
- 指定参考坐标系下的 FK 目标位姿
- IK 求解结果
- 正逆解位置/姿态误差
- 平均耗时、平均迭代步数、成功率

失败样本会写入当前工作目录下的：

```text
fa_left_arm_ik_failed_cases.log
fa_right_arm_ik_failed_cases.log
```

## RViz/MoveIt 测试

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

说明：

- 节点会先移动到 `leftArmHome` 或 `rightArmHome`
- 然后随机采样关节角，基于指定参考坐标系计算 FK/IK
- IK 成功后使用 MoveIt 执行到求解出的关节角
- 同时发布 `end_effector_markers` 方便在 RViz 里观察末端位姿

当 `reference_frame:=pelvis` 时，Marker 发布在 `pelvis` 坐标系下；
当 `reference_frame:=arm_base` 时，Marker 发布在 `waist_pitch_link` 坐标系下。
