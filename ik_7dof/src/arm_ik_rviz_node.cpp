#include <rclcpp/rclcpp.hpp>
#include "ik_7dof/ik_solver.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <random>
#include <memory>
#include <chrono>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
using namespace left_arm_ik_test;

class ArmIKRvizNode : public rclcpp::Node
{
public:
    ArmIKRvizNode() : Node("arm_ik_rviz_node")
    {
        this->declare_parameter<std::string>("urdf_file", "");
        this->declare_parameter<std::string>("srdf_file", "");
        this->declare_parameter<int>("num_tests", 5);
        this->declare_parameter<int>("max_iters", 200);
        this->declare_parameter<double>("eps", 1e-3);
        this->declare_parameter<double>("move_delay", 1.0);
        
        std::string urdf_file = this->get_parameter("urdf_file").as_string();
        std::string srdf_file = this->get_parameter("srdf_file").as_string();
        num_tests_ = this->get_parameter("num_tests").as_int();
        max_iters_ = this->get_parameter("max_iters").as_int();
        eps_ = this->get_parameter("eps").as_double();
        move_delay_ = this->get_parameter("move_delay").as_double();
        
        if (urdf_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "必须提供urdf_file参数");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "URDF文件: %s", urdf_file.c_str());
        RCLCPP_INFO(this->get_logger(), "SRDF文件: %s", srdf_file.c_str());
        RCLCPP_INFO(this->get_logger(), "测试次数: %d", num_tests_);
        
        try {
            solver_ = std::make_unique<IKSolver>(urdf_file, srdf_file);
            printJointInfo();
            
            // 初始化发布器
            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
            
            // 启动验证和运动
            runValidationAndMove();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "初始化失败: %s", e.what());
        }
    }

private:
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

    void publishJointState(const Eigen::VectorXd& q)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        
        const auto& joint_names = solver_->getLeftArmJointNames();
        for (size_t i = 0; i < joint_names.size(); ++i) {
            msg.name.push_back(joint_names[i]);
            msg.position.push_back(q[i]);
        }
        
        joint_state_pub_->publish(msg);
    }

    void publishEndEffectorFrame(const PoseSE3& pose, const std::string& frame_id)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "base_link";
        transform.child_frame_id = frame_id;
        
        // 位置
        transform.transform.translation.x = pose.p.x();
        transform.transform.translation.y = pose.p.y();
        transform.transform.translation.z = pose.p.z();
        
        // 旋转（转换为四元数）
        Eigen::Quaterniond q(pose.R);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
    }

    void runValidationAndMove()
    {
        RCLCPP_INFO(this->get_logger(), "========== 左臂正逆解验证与运动开始 ==========");
        
        std::random_device rd;
        std::mt19937 gen(rd());
        
        int success_count = 0;
        
        auto limits = solver_->getJointLimits();
        const auto& joint_names = solver_->getLeftArmJointNames();

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
            auto fk_result = solver_->computeLeftArmFK_SE3(q_rand);
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
                
                // 6. 在Rviz中运动到求解的关节角度
                RCLCPP_INFO(this->get_logger(), "在Rviz中运动到目标关节角度...");
                publishJointState(q_solved);
                
                // 7. 发布末端执行器坐标系
                std::string frame_id = "end_effector_" + std::to_string(test + 1);
                publishEndEffectorFrame(fk_verify_se3, frame_id);
                RCLCPP_INFO(this->get_logger(), "已在Rviz中创建末端位姿坐标系: %s", frame_id.c_str());
                
                // 等待一段时间，让Rviz显示
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
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    int num_tests_;
    int max_iters_;
    double eps_;
    double move_delay_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmIKRvizNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}