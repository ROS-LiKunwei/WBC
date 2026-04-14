# IK 7DOF 机械臂逆运动学求解器

## 项目概述

本项目实现了一个 7 自由度机械臂的逆运动学求解器（fa_ik_solver），以及一个可视化验证节点（arm_ik_rviz_node），用于验证正逆运动学的正确性并在 RViz 中展示机械臂的运动。

## fa_ik_solver 原理

### 1. 核心原理

fa_ik_solver 基于以下原理实现：

- **正运动学**：使用 Pinocchio 库计算机械臂末端的位姿。Pinocchio 是一个高效的机器人动力学库，能够快速计算机器人的正向运动学。

- **逆运动学**：使用 Levenberg-Marquardt 算法求解逆运动学。这是一种非线性最小二乘优化算法，能够在存在多个解的情况下找到最优解。

- **关节角度限位**：在求解过程中考虑关节的角度限位，确保求解结果在机械臂的物理范围内。

### 2. 主要功能

- **正运动学计算**：给定关节角度，计算末端执行器的位姿。
- **逆运动学求解**：给定末端执行器的位姿，求解关节角度。
- **关节角度限位**：确保求解结果在关节的物理范围内。
- **多解支持**：由于 7 自由度机械臂存在多个逆运动学解，求解器会返回一个最优解。

### 3. 技术实现

- **Pinocchio 库**：用于计算正运动学。
- **Eigen 库**：用于矩阵运算。
- **Levenberg-Marquardt 算法**：用于求解逆运动学。
- **ROS 2**：用于与其他组件通信。

## arm_ik_rviz_node 运行效果

### 1. 功能描述

arm_ik_rviz_node 是一个 ROS 2 节点，用于验证 fa_ik_solver 的正确性并在 RViz 中展示机械臂的运动。

- **正逆运动学验证**：生成随机关节角度，计算正运动学，然后使用逆运动学求解，验证正逆解的一致性。
- **机械臂运动**：使用 MoveIt 2 规划并执行机械臂的运动，使机械臂移动到求解的关节角度。
- **末端位姿可视化**：使用 MarkerArray 在 RViz 中显示末端执行器的位姿。

### 2. 运行效果

当运行 arm_ik_rviz_node 时，您将看到：

1. 机械臂从 SRDF 中定义的 `leftArmHome` 状态开始。
2. 机械臂会生成随机关节角度，计算正运动学，然后使用逆运动学求解。
3. 机械臂会平滑地移动到求解的关节角度。
4. 在 RViz 中，您将看到末端执行器的位姿通过箭头标记显示。
5. 控制台会输出正逆运动学验证的结果，包括成功率和误差。

## 启动方式

### 1. 构建项目

```bash
colcon build --packages-select ik_7dof
```

### 2. 源安装目录

```bash
source install/setup.bash
```

### 3. 运行 launch 文件

```bash
ros2 launch ik_7dof arm_ik_rviz.launch.py
```

### 4. 在 RViz 中添加 MarkerArray 显示器

- 点击 "Add" 按钮
- 选择 "By topic"
- 选择 "/end_effector_markers" 话题

## 配置参数

arm_ik_rviz_node 支持以下配置参数：

- `urdf_file`：URDF 文件路径
- `srdf_file`：SRDF 文件路径
- `num_tests`：测试次数（默认：10）
- `max_iters`：最大迭代次数（默认：1000）
- `eps`：收敛阈值（默认：1e-6）
- `move_delay`：移动延迟（默认：1.0 秒）
- `trajectory_duration`：轨迹持续时间（默认：2.0 秒）

## 依赖项

- **ROS 2 Humble**：基础框架
- **MoveIt 2**：机械臂规划和控制
- **Pinocchio**：正运动学计算
- **Eigen**：矩阵运算
- **rviz2**：可视化

## 代码结构

```
ik_7dof/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── ik_solver.cpp     # IK 求解器实现
│   ├── ik_solver.hpp     # IK 求解器头文件
│   └── arm_ik_rviz_node.cpp  # 可视化验证节点
└── launch/
    └── arm_ik_rviz.launch.py  # 启动文件
```

## 注意事项

- 确保您的机械臂模型（URDF 和 SRDF 文件）正确配置。
- 确保 MoveIt 2 环境正确设置。
- 如果遇到控制器启动失败的问题，请检查控制器配置文件。

## 故障排除

- **控制器启动失败**：检查控制器配置文件，确保控制器名称和关节名称正确。
- **机械臂不运动**：检查 MoveIt 2 配置，确保规划组名称正确。
- **逆运动学求解失败**：检查末端执行器的位姿是否在机械臂的工作空间内。

## 贡献

欢迎提交问题和建议，帮助我们改进这个项目。

## 许可证

本项目采用 MIT 许可证。