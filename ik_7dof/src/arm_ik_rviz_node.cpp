#include <rclcpp/rclcpp.hpp>
#include "ik_7dof/ik_solver.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <random>
#include <memory>
#include <chrono>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_state/robot_state.h>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace left_arm_ik_test;

class ArmIKRvizNode : public rclcpp::Node
{
public:
    ArmIKRvizNode() : Node("arm_ik_rviz_node")
    {
        this->declare_parameter<std::string>("urdf_file", "");
        this->declare_parameter<std::string>("srdf_file", "");
        this->declare_parameter<int>("num_tests", 5);
        this->declare_parameter<int>("max_iters", 1000);
        this->declare_parameter<double>("eps", 1e-3);
        this->declare_parameter<double>("move_delay", 1.0);
        this->declare_parameter<double>("trajectory_duration", 2.0);
        
        std::string urdf_file = this->get_parameter("urdf_file").as_string();
        std::string srdf_file = this->get_parameter("srdf_file").as_string();
        num_tests_ = this->get_parameter("num_tests").as_int();
        max_iters_ = this->get_parameter("max_iters").as_int();
        eps_ = this->get_parameter("eps").as_double();
        move_delay_ = this->get_parameter("move_delay").as_double();
        trajectory_duration_ = this->get_parameter("trajectory_duration").as_double();
        
        if (urdf_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "必须提供urdf_file参数");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "URDF文件: %s", urdf_file.c_str());
        RCLCPP_INFO(this->get_logger(), "SRDF文件: %s", srdf_file.c_str());
        RCLCPP_INFO(this->get_logger(), "测试次数: %d", num_tests_);
        
        try {
            // ---  初始化IK求解器 ---
            solver_ = std::make_unique<IKSolver>(urdf_file, srdf_file);
            printJointInfo();
            
            // --- 初始化发布器 ---
            // 发布关节状态，用于驱动 RViz 中的机器人模型
            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            // 发布MarkerArray，用于在 RViz 中观察末端位姿
            marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("end_effector_markers", 10);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "初始化失败: %s", e.what());
        }
    }

    // ✨ 等待move_group加载完成并启动验证
    void start_execution()
    {
        try {
            // 等待机器人模型加载完成
            waitForPlanningScene();
            // 启动验证和运动
            runValidationAndMove();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "运行过程中发生异常: %s", e.what());
        }
    }

private:
    /**
     * @brief 获取初始关节值
     */
    Eigen::VectorXd getHomeJointValues(std::shared_ptr<rclcpp::Node> node)
    {
        // 1. 初始化 MoveGroupInterface
        moveit::planning_interface::MoveGroupInterface move_group(node, "leftArm");

        // 2. 获取当前的 RobotState 指针
        moveit::core::RobotStatePtr kinematic_state = move_group.getCurrentState();
        if (!kinematic_state) {
            RCLCPP_ERROR(node->get_logger(), "无法获取 RobotState");
            return Eigen::VectorXd();
        }

        // 3. 获取对应的 JointModelGroup
        const moveit::core::JointModelGroup* joint_model_group = 
            kinematic_state->getJointModelGroup("leftArm");

        // 4. ✨ 核心步骤：将这个状态对象强制设置为 SRDF 中定义的状态
        // 如果找不到该名字，会返回 false
        bool found = kinematic_state->setToDefaultValues(joint_model_group, "leftArmHome");
        
        if (found) {
            // 5. 提取出具体的关节角度
            std::vector<double> home_joint_values;
            kinematic_state->copyJointGroupPositions(joint_model_group, home_joint_values);

            // 6. 打印验证
            const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
            RCLCPP_INFO(node->get_logger(), "--- leftArmHome 关节角度 ---");
            for (size_t i = 0; i < joint_names.size(); ++i) {
                RCLCPP_INFO(node->get_logger(), "%s: %.4f", joint_names[i].c_str(), home_joint_values[i]);
            }
            
            // 7. 转换为 Eigen::VectorXd 并返回
            return Eigen::Map<Eigen::VectorXd>(home_joint_values.data(), home_joint_values.size());
        } else {
            RCLCPP_WARN(node->get_logger(), "在 SRDF 中未找到名为 'leftArmHome' 的状态！");
            return Eigen::VectorXd();
        }
    }

    /**
     * @brief 移动到leftArmHome状态
     */
    void moveToHomeState(std::shared_ptr<rclcpp::Node> node)
    {
        // 等待控制器启动
        RCLCPP_INFO(node->get_logger(), "等待控制器启动...");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        moveit::planning_interface::MoveGroupInterface move_group(node, "leftArm");

        // 设置规划时间
        move_group.setPlanningTime(5.0);

        // 获取所有可用的命名目标名称 (修复 getNamedTargetNames 错误)
        std::vector<std::string> named_targets = move_group.getNamedTargets();

        if (!named_targets.empty()) {
            RCLCPP_INFO(node->get_logger(), "控制器已就绪，可用的命名目标数量: %zu", named_targets.size());
            for (const auto& name : named_targets) {
                RCLCPP_INFO(node->get_logger(), "  - %s", name.c_str());
            }
        }else {
            RCLCPP_WARN(node->get_logger(), "控制器可能尚未就绪，尝试继续...");
        }
        
        // ✨ 核心步骤：直接设置命名目标
        move_group.setNamedTarget("leftArmHome");

        // 执行运动 - 使用 move() 方法，它会自动处理规划和执行
        RCLCPP_INFO(node->get_logger(), "开始规划并执行到 leftArmHome...");
        bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(node->get_logger(), "执行到 leftArmHome 成功！");
        } else {
            RCLCPP_ERROR(node->get_logger(), "执行到 leftArmHome 失败！");
        }
    }

    /**
     * @brief 打印关节的基本信息和上下限
     */
    void printJointInfo()
    {
        RCLCPP_INFO(this->get_logger(), "左臂关节 (%zu 个):", solver_->getLeftArmJointCount());
        const auto& joint_names = solver_->getLeftArmJointNames();
        for (size_t i = 0; i < joint_names.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  %2zu: %s", i, joint_names[i].c_str());
        }
        
        auto limits = solver_->getJointLimits();
        RCLCPP_INFO(this->get_logger(), "关节限制:");
        for (size_t i = 0; i < joint_names.size(); ++i) {
            double lower = limits.first[i];
            double upper = limits.second[i];
            RCLCPP_INFO(this->get_logger(), "  %s: [%.3f, %.3f]", 
                    joint_names[i].c_str(), lower, upper);
        }
    }
    
    /**
     * @brief 等待机器人模型加载完成
     */
    /**
     * @brief 显式等待 MoveIt 2 的 Planning Scene 完全加载
     */
    void waitForPlanningScene()
    {
        RCLCPP_INFO(this->get_logger(), "等待 MoveIt 2 Planning Scene 服务加载...");
        
        // 创建一个客户端，去寻找 move_group 节点提供的 get_planning_scene 服务
        auto planning_scene_client = 
            this->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
        
        // 循环等待，每 2 秒打印一次状态
        while (!planning_scene_client->wait_for_service(std::chrono::seconds(2))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待 Planning Scene 时被打断 (Ctrl+C)");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "还在等待 Planning Scene 服务，请确保 move_group 已启动...");
        }
        
        RCLCPP_INFO(this->get_logger(), "✅ Planning Scene 及机器人模型已完全加载！");
    }
    

    
    /**
     * @brief 发布末端执行器的位姿到 MarkerArray
     */
    void publishEndEffectorPose(const PoseSE3& pose, int index = 0)
    {
        // 创建 MarkerArray
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 转换为四元数
        Eigen::Quaterniond q(pose.R);
        
        // 创建坐标轴标记
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "pelvis";
        marker.id = index;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 设置位置
        marker.pose.position.x = pose.p.x();
        marker.pose.position.y = pose.p.y();
        marker.pose.position.z = pose.p.z();
        
        // 设置旋转
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        
        // 设置大小
        marker.scale.x = 0.1;  // X轴长度
        marker.scale.y = 0.05; // 箭头宽度
        marker.scale.z = 0.05; // 箭头高度
        
        // 设置颜色（红色）
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        // 添加到 MarkerArray
        marker_array.markers.push_back(marker);
        
        // 发布 MarkerArray
        marker_array_pub_->publish(marker_array);
    }
    
    /**
     * @brief 核心业务逻辑：循环生成随机位姿，求解IK并在可视化界面演示
     */
    void runValidationAndMove()
    {
        RCLCPP_INFO(this->get_logger(), "========== 左臂正逆解验证与运动开始 ==========");
        
        std::random_device rd;
        std::mt19937 gen(rd());
        
        int success_count = 0;
        
        auto limits = solver_->getJointLimits();
        const auto& joint_names = solver_->getLeftArmJointNames();
        
        // 初始关节角度（从 SRDF 中读取的 leftArmHome 状态）
        RCLCPP_INFO(this->get_logger(), "移动到 leftArmHome 状态...");
        moveToHomeState(shared_from_this());
        
        // 获取 leftArmHome 状态的关节角度
        Eigen::VectorXd current_q = getHomeJointValues(shared_from_this());
        
        // 如果获取失败，使用中性位姿作为备选
        if (current_q.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "无法获取 leftArmHome 状态，使用中性位姿作为备选");
            return ;
        }
        
        for (int test = 0; test < num_tests_; ++test) {
            RCLCPP_INFO(this->get_logger(), "\n--- 测试 %d/%d ---", test + 1, num_tests_);
            
            // 1. 生成随机关节角度（在限位内）
            Eigen::VectorXd q_rand(7);
            for (int i = 0; i < 7; ++i) {
                if (limits.second[i] > limits.first[i] + 1e-6) {
                    std::uniform_real_distribution<> dist(
                        limits.first[i], limits.second[i]);
                    q_rand[i] = dist(gen);
                } else {
                    q_rand[i] = 0.0;
                    RCLCPP_WARN(this->get_logger(), "关节 %s 限制异常 [%.3f, %.3f]", 
                               joint_names[i].c_str(), limits.first[i], limits.second[i]);
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "随机关节角度:");
            for (int i = 0; i < 7; ++i) {
                RCLCPP_INFO(this->get_logger(), "  %s: %.6f rad", 
                        joint_names[i].c_str(), q_rand[i]);
            }
            
            // 2. 计算正运动学
            PoseSE3 fk_result = solver_->computeLeftArmFK_SE3(q_rand);
            RCLCPP_INFO(this->get_logger(), "正运动学结果:");
            RCLCPP_INFO(this->get_logger(), "  位置: [%.6f, %.6f, %.6f] m", 
                    fk_result.p.x(), fk_result.p.y(), fk_result.p.z());
            
            // 3. 构造目标位姿
            pinocchio::SE3 T_target;
            T_target.translation(fk_result.p);
            T_target.rotation(fk_result.R);
            
            // 4. 执行逆运动学求解
            int iters = 0;
            auto start = std::chrono::high_resolution_clock::now();
            Eigen::VectorXd q_solved = solver_->solveLeftArmIK(
                T_target, Eigen::VectorXd(), max_iters_, eps_, &iters);
            auto end = std::chrono::high_resolution_clock::now();
            
            // 5. 检查结果
            bool consistency_ok = false;
            if (q_solved.size() == 0) {
                RCLCPP_ERROR(this->get_logger(), "逆运动学求解失败！");
            } else {
                RCLCPP_INFO(this->get_logger(), "逆运动学结果:");
                for (int i = 0; i < 7; ++i) {
                    RCLCPP_INFO(this->get_logger(), "  %s: %.6f rad", 
                            joint_names[i].c_str(), q_solved[i]);
                }
                
                // 验证正逆解一致性
                auto fk_verify_se3 = solver_->computeLeftArmFK_SE3(q_solved);
                double pos_error = (fk_result.p - fk_verify_se3.p).norm();
                Eigen::Matrix3d R_diff = fk_result.R.transpose() * fk_verify_se3.R;
                Eigen::AngleAxisd angle_diff(R_diff);
                double rot_error = std::abs(angle_diff.angle());
                
                RCLCPP_INFO(this->get_logger(), "一致性验证:");
                RCLCPP_INFO(this->get_logger(), "  位置误差: %.8f m", pos_error);
                RCLCPP_INFO(this->get_logger(), "  姿态误差: %.8f rad", rot_error);
                
                if (pos_error < 1e-3 && rot_error < 1e-3) {
                    RCLCPP_INFO(this->get_logger(), "  ✅ 正逆解一致！");
                    consistency_ok = true;
                    success_count++;
                } else {
                    RCLCPP_INFO(this->get_logger(), "  ❌ 正逆解不一致！");
                }
                
                //  6. 让 MoveIt 2 移动到 IK 解
                RCLCPP_INFO(this->get_logger(), "让 MoveIt 规划并移动到求解的关节角度...");
                
                // 实例化 MoveGroup
                moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "leftArm");
                
                // 设置规划时间
                move_group.setPlanningTime(5.0);
                
                // 将 Eigen::VectorXd 转换为 std::vector<double>
                std::vector<double> target_joint_values(q_solved.data(), q_solved.data() + q_solved.size());
                
                // 检查关节值是否在限位内
                auto limits = solver_->getJointLimits();
                bool within_limits = true;
                for (int i = 0; i < 7; ++i) {
                    if (target_joint_values[i] < limits.first[i] || target_joint_values[i] > limits.second[i]) {
                        within_limits = false;
                        RCLCPP_WARN(this->get_logger(), "关节 %s 值 %.6f 超出限位 [%.6f, %.6f]", 
                                   joint_names[i].c_str(), target_joint_values[i], limits.first[i], limits.second[i]);
                    }
                }
                
                if (!within_limits) {
                    RCLCPP_ERROR(this->get_logger(), "目标关节值超出限位，跳过执行");
                    continue;
                }
                
                // 设置目标并执行
                move_group.setJointValueTarget(target_joint_values);
                
                // 执行运动 - 使用 move() 方法，它会自动处理规划和执行
                RCLCPP_INFO(this->get_logger(), "开始规划并执行到 IK 解...");
                bool move_success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
                if (move_success) {
                    RCLCPP_INFO(this->get_logger(), "执行成功！");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "执行失败！");
                }
                
                // ✨ 7. 发布 Marker (用于在 RViz 中留下永久的 XYZ 坐标轴)
                publishEndEffectorPose(fk_verify_se3, test);
                RCLCPP_INFO(this->get_logger(), "已发布 Marker: end_effector_%d", test);
                
                // 等待一下以便观察
                std::this_thread::sleep_for(std::chrono::duration<double>(move_delay_));
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "\n========== 验证结果 ==========");
        RCLCPP_INFO(this->get_logger(), "总测试次数: %d", num_tests_);
        RCLCPP_INFO(this->get_logger(), "成功次数: %d", success_count);
        RCLCPP_INFO(this->get_logger(), "成功率: %.1f%%", 100.0 * success_count / num_tests_);
        RCLCPP_INFO(this->get_logger(), "========== 验证完成 ==========");
    }

    std::unique_ptr<IKSolver> solver_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    int num_tests_;
    int max_iters_;
    double eps_;
    double move_delay_;
    double trajectory_duration_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmIKRvizNode>();

    std::thread execution_thread([node]() {
        node->start_execution();
    });

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    if (execution_thread.joinable()) {
        execution_thread.join();
    }

    return 0;
}