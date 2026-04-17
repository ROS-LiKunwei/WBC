# ik_7dof 包说明

## 项目概述

`ik_7dof` 是一个用于 7 自由度机械臂正逆运动学求解的 ROS 2 包，主要功能包括：

- **正逆运动学求解**：支持 FA 机器人的左右手正逆运动学计算
- **离线验证**：通过随机生成关节角度，验证正逆解的一致性
- **RViz 可视化**：在 RViz 中实时显示机械臂运动和末端位姿
- **关节限位检查**：确保求解结果在关节角度限位范围内
- **失败案例记录**：将求解失败的案例记录到日志文件

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
- **双手支持**：同时支持 FA 机器人的左手和右手。
- **多种求解方法**：支持 LDLT（快速）和 SVD（稳健）两种求解方法。

### 3. 技术实现

- **Pinocchio 库**：用于计算正运动学。
- **Eigen 库**：用于矩阵运算。
- **Levenberg-Marquardt 算法**：用于求解逆运动学。
- **ROS 2**：用于与其他组件通信。

## 节点功能

### 1. fa_arm_kinematic_node

**功能描述**：FA 机器人双手 IK 验证节点，用于离线验证正逆运动学的正确性。

**主要功能**：
- **正逆运动学验证**：生成随机关节角度，计算正运动学，然后使用逆运动学求解，验证正逆解的一致性。
- **失败案例记录**：将求解失败的案例记录到日志文件。
- **性能统计**：统计求解成功率、平均耗时等性能指标。
- **双手支持**：通过参数选择验证左臂或右臂。

**运行效果**：
- 控制台输出测试进度、成功率、平均耗时等信息。
- 生成失败案例日志文件，方便分析求解失败的原因。

### 2. arm_ik_rviz_node

**功能描述**：RViz 可视化验证节点，用于在 RViz 中展示机械臂的运动并验证正逆运动学的正确性。

**主要功能**：
- **正逆运动学验证**：生成随机关节角度，计算正运动学，然后使用逆运动学求解，验证正逆解的一致性。
- **机械臂运动**：使用 MoveIt 2 规划并执行机械臂的运动，使机械臂移动到求解的关节角度。
- **末端位姿可视化**：使用 MarkerArray 在 RViz 中显示末端执行器的位姿。
- **双手支持**：通过参数选择验证左臂或右臂。

**运行效果**：
1. 机械臂从 SRDF 中定义的 `leftArmHome` 或 `rightArmHome` 状态开始。
2. 机械臂会生成随机关节角度，计算正运动学，然后使用逆运动学求解。
3. 机械臂会平滑地移动到求解的关节角度。
4. 在 RViz 中，您将看到末端执行器的位姿通过箭头标记显示。
5. 控制台会输出正逆运动学验证的结果，包括成功率和误差。

## 启动方式

### 1. 构建项目

```bash
cd /home/likunwei/humanoid_ws
colcon build --packages-select ik_7dof
source install/setup.bash
```

### 2. 运行 FA 机器人 IK 验证

#### 验证左臂

```bash
ros2 launch ik_7dof fa_arm_kinematic_verify.launch.py arm_side:=left num_tests:=1000
```

#### 验证右臂

```bash
ros2 launch ik_7dof fa_arm_kinematic_verify.launch.py arm_side:=right num_tests:=1000
```

### 3. 运行 RViz 可视化验证

#### 验证左臂

```bash
ros2 launch ik_7dof arm_ik_rviz.launch.py arm_side:=left
```

#### 验证右臂

```bash
ros2 launch ik_7dof arm_ik_rviz.launch.py arm_side:=right
```

### 4. 在 RViz 中添加 MarkerArray 显示器

- 点击 "Add" 按钮
- 选择 "By topic"
- 选择 "/end_effector_markers" 话题

## 配置参数

### FA 机器人 IK 验证参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `arm_side` | string | left | 手臂选择：left 或 right |
| `num_tests` | int | 100 | 测试次数 |
| `max_iters` | int | 1000 | 最大迭代次数 |
| `eps` | double | 1e-3 | 收敛精度 |

### RViz 可视化验证参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `arm_side` | string | left | 手臂选择：left 或 right |
| `num_tests` | int | 10000 | 测试次数 |
| `max_iters` | int | 1000 | 最大迭代次数 |
| `eps` | double | 1e-3 | 收敛精度 |
| `move_delay` | double | 1.0 | 运动间隔时间（秒） |
| `trajectory_duration` | double | 2.0 | 轨迹持续时间（秒） |

## 依赖项

- **ROS 2 Humble**：基础框架
- **MoveIt 2**：机械臂规划和控制
- **Pinocchio**：正运动学计算
- **Eigen**：矩阵运算
- **rviz2**：可视化
- **tf2_ros**：坐标变换
- **geometry_msgs**：几何消息
- **sensor_msgs**：传感器消息
- **srdfdom**：SRDF 文件解析

## 代码结构

```
ik_7dof/
├── CMakeLists.txt
├── package.xml
├── include/ik_7dof/
│   ├── ik_solver.hpp            # 通用 IK 求解器接口
│   └── fa_ik_solver.hpp         # FA 机器人专用 IK 求解器接口
├── src/
│   ├── ik_solver.cpp             # 通用 IK 求解器实现
│   ├── fa_ik_solver.cpp          # FA 机器人专用 IK 求解器实现
│   ├── left_arm_ik_node.cpp      # 左臂 IK 验证节点
│   ├── fa_arm_kinematic_node.cpp # FA 机器人双手 IK 验证节点
│   └── arm_ik_rviz_node.cpp      # RViz 可视化验证节点
└── launch/
    ├── fa_arm_kinematic_verify.launch.py  # FA 机器人 IK 验证
    └── arm_ik_rviz.launch.py     # RViz 可视化验证
```

## 注意事项

- 确保您的机械臂模型（URDF 和 SRDF 文件）正确配置。
- 确保 MoveIt 2 环境正确设置。
- 如果遇到控制器启动失败的问题，请检查控制器配置文件。
- 运行右臂验证时，确保 URDF 文件中右手末端坐标系名称为 `hand_base_link`。

## 故障排除

- **控制器启动失败**：检查控制器配置文件，确保控制器名称和关节名称正确。
- **机械臂不运动**：检查 MoveIt 2 配置，确保规划组名称正确。
- **逆运动学求解失败**：检查末端执行器的位姿是否在机械臂的工作空间内。
- **Frame not found**：检查末端执行器的 frame 名称是否与 URDF 文件一致。

## 示例输出

```
[INFO] [fa_arm_kinematic_node]: ========== FA左臂正逆解验证开始 ==========
[INFO] [fa_arm_kinematic_node]: --- 测试 1/1000 ---
[INFO] [fa_arm_kinematic_node]: 测试 1 成功！
[INFO] [fa_arm_kinematic_node]: --- 测试 2/1000 ---
[INFO] [fa_arm_kinematic_node]: 测试 2 成功！
...
[INFO] [fa_arm_kinematic_node]: 进度: 500/1000, 当前成功率: 98.2%
...
[INFO] [fa_arm_kinematic_node]: ========== 验证结果 ==========
[INFO] [fa_arm_kinematic_node]: 总测试次数: 1000
[INFO] [fa_arm_kinematic_node]: 成功次数: 985
[INFO] [fa_arm_kinematic_node]: 算法分布统计:
[INFO] [fa_arm_kinematic_node]:   - 第一阶段 (LDLT) 成功: 970
[INFO] [fa_arm_kinematic_node]:   - 第二阶段 (SVD) 成功: 15
[INFO] [fa_arm_kinematic_node]: 平均成功耗时: 2.3456 ms
[INFO] [fa_arm_kinematic_node]: 平均成功步长: 12.34 steps
[INFO] [fa_arm_kinematic_node]: 成功率: 98.5%
[INFO] [fa_arm_kinematic_node]: 失败案例已保存至: fa_left_arm_ik_failed_cases.log
```

## FA 机器人正逆运动学使用方法

### 1. 代码中使用 fa_ik_solver

#### 1.1 实例化 IKSolver

```cpp
#include "ik_7dof/fa_ik_solver.hpp"

// 实例化求解器
std::string urdf_file = "/path/to/fa_robot.urdf";
std::string srdf_file = "/path/to/fa_robot.srdf";
fa_arm_kinematic::IKSolver solver(urdf_file, srdf_file);
```

#### 1.2 正运动学计算

```cpp
// 定义关节角度（7 个关节）
Eigen::VectorXd q(7);
q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 初始化为零位置

// 计算左臂正运动学
fa_arm_kinematic::PoseRPY left_fk = solver.computeArmFK(q, fa_arm_kinematic::ArmSide::LEFT);
std::cout << "左臂末端位姿：" << std::endl;
std::cout << "位置：x=" << left_fk.x << ", y=" << left_fk.y << ", z=" << left_fk.z << std::endl;
std::cout << "姿态：roll=" << left_fk.roll << ", pitch=" << left_fk.pitch << ", yaw=" << left_fk.yaw << std::endl;

// 计算右臂正运动学
fa_arm_kinematic::PoseRPY right_fk = solver.computeArmFK(q, fa_arm_kinematic::ArmSide::RIGHT);
std::cout << "右臂末端位姿：" << std::endl;
std::cout << "位置：x=" << right_fk.x << ", y=" << right_fk.y << ", z=" << right_fk.z << std::endl;
std::cout << "姿态：roll=" << right_fk.roll << ", pitch=" << right_fk.pitch << ", yaw=" << right_fk.yaw << std::endl;
```

#### 1.3 逆运动学求解

```cpp
// 定义目标位姿（使用 Pinocchio SE3 格式）
pinocchio::SE3 T_target;
Eigen::Vector3d target_pos(0.5, 0.2, 0.5); // 目标位置
Eigen::Matrix3d target_rot = pinocchio::SE3::Random().rotation(); // 目标姿态
T_target.translation(target_pos);
T_target.rotation(target_rot);

// 求解左臂逆运动学
Eigen::VectorXd left_q_solved = solver.solveArmIK(T_target, fa_arm_kinematic::ArmSide::LEFT);
if (left_q_solved.size() > 0) {
    std::cout << "左臂逆运动学求解成功！" << std::endl;
    std::cout << "关节角度：" << left_q_solved.transpose() << std::endl;
} else {
    std::cout << "左臂逆运动学求解失败！" << std::endl;
}

// 求解右臂逆运动学
Eigen::VectorXd right_q_solved = solver.solveArmIK(T_target, fa_arm_kinematic::ArmSide::RIGHT);
if (right_q_solved.size() > 0) {
    std::cout << "右臂逆运动学求解成功！" << std::endl;
    std::cout << "关节角度：" << right_q_solved.transpose() << std::endl;
} else {
    std::cout << "右臂逆运动学求解失败！" << std::endl;
}
```

#### 1.4 指定求解器方法（solveArmIK内部自动选择）

```cpp
// 使用 LDLT 方法（快速）
Eigen::VectorXd q_solved_ldlt = solver.solveIK_Core(
    T_target,              // 目标位姿
    Eigen::VectorXd(),     // 初始关节角度（默认零）
    100,                   // 最大迭代次数
    1e-3,                  // 收敛精度
    iters,                 // 输出实际迭代次数
    fa_arm_kinematic::SolverMethod::LDLT,  // 求解方法
    fa_arm_kinematic::ArmSide::LEFT        // 手臂侧
);

// 使用 SVD 方法（稳健）
Eigen::VectorXd q_solved_svd = solver.solveIK_Core(
    T_target,              // 目标位姿
    Eigen::VectorXd(),     // 初始关节角度（默认零）
    100,                   // 最大迭代次数
    1e-3,                  // 收敛精度
    iters,                 // 输出实际迭代次数
    fa_arm_kinematic::SolverMethod::SVD,   // 求解方法
    fa_arm_kinematic::ArmSide::LEFT        // 手臂侧
);
```

### 2. 获取关节信息

```cpp
// 获取左臂关节名称
const auto& left_joint_names = solver.getArmJointNames(fa_arm_kinematic::ArmSide::LEFT);
std::cout << "左臂关节名称：" << std::endl;
for (const auto& name : left_joint_names) {
    std::cout << "  " << name << std::endl;
}

// 获取右臂关节名称
const auto& right_joint_names = solver.getArmJointNames(fa_arm_kinematic::ArmSide::RIGHT);
std::cout << "右臂关节名称：" << std::endl;
for (const auto& name : right_joint_names) {
    std::cout << "  " << name << std::endl;
}

// 获取关节限位
auto left_limits = solver.getArmJointLimits(fa_arm_kinematic::ArmSide::LEFT);
std::cout << "左臂关节限位：" << std::endl;
for (size_t i = 0; i < left_joint_names.size(); ++i) {
    std::cout << "  " << left_joint_names[i] << ": [" << left_limits.first[i] << ", " << left_limits.second[i] << "]" << std::endl;
}

auto right_limits = solver.getArmJointLimits(fa_arm_kinematic::ArmSide::RIGHT);
std::cout << "右臂关节限位：" << std::endl;
for (size_t i = 0; i < right_joint_names.size(); ++i) {
    std::cout << "  " << right_joint_names[i] << ": [" << right_limits.first[i] << ", " << right_limits.second[i] << "]" << std::endl;
}
```

### 3. 集成到 ROS 2 节点

```cpp
#include <rclcpp/rclcpp.hpp>
#include "ik_7dof/fa_ik_solver.hpp"

class MyIKNode : public rclcpp::Node
{
public:
    MyIKNode() : Node("my_ik_node")
    {
        // 初始化求解器
        std::string urdf_file = this->declare_parameter<std::string>("urdf_file", "");
        std::string srdf_file = this->declare_parameter<std::string>("srdf_file", "");
        solver_ = std::make_unique<fa_arm_kinematic::IKSolver>(urdf_file, srdf_file);
        
        // 创建定时器，定期执行正逆运动学计算
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyIKNode::runIKTest, this)
        );
    }
    
private:
    void runIKTest()
    {
        // 生成随机关节角度
        Eigen::VectorXd q(7);
        std::random_device rd;
        std::mt19937 gen(rd());
        auto limits = solver_->getArmJointLimits(fa_arm_kinematic::ArmSide::LEFT);
        for (int i = 0; i < 7; ++i) {
            std::uniform_real_distribution<> dist(limits.first[i], limits.second[i]);
            q[i] = dist(gen);
        }
        
        // 计算正运动学
        auto fk_result = solver_->computeArmFK_SE3(q, fa_arm_kinematic::ArmSide::LEFT);
        
        // 构建目标位姿
        pinocchio::SE3 T_target;
        T_target.translation(fk_result.p);
        T_target.rotation(fk_result.R);
        
        // 求解逆运动学
        int iters = 0;
        Eigen::VectorXd q_solved = solver_->solveArmIK(
            T_target, 
            fa_arm_kinematic::ArmSide::LEFT, 
            Eigen::VectorXd(), 
            100, 
            1e-3, 
            &iters
        );
        
        // 验证结果
        if (q_solved.size() > 0) {
            auto fk_verify = solver_->computeArmFK_SE3(q_solved, fa_arm_kinematic::ArmSide::LEFT);
            double pos_error = (fk_result.p - fk_verify.p).norm();
            Eigen::Matrix3d R_diff = fk_result.R.transpose() * fk_verify.R;
            Eigen::AngleAxisd angle_diff(R_diff);
            double rot_error = std::abs(angle_diff.angle());
            
            RCLCPP_INFO(this->get_logger(), "正逆解验证成功！位置误差: %.4f, 姿态误差: %.4f", pos_error, rot_error);
        } else {
            RCLCPP_ERROR(this->get_logger(), "逆运动学求解失败！");
        }
    }
    
    std::unique_ptr<fa_arm_kinematic::IKSolver> solver_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyIKNode>());
    rclcpp::shutdown();
    return 0;
}
```

## 贡献

欢迎提交问题和建议，帮助我们改进这个项目。

## 许可证

本项目采用 MIT 许可证。
