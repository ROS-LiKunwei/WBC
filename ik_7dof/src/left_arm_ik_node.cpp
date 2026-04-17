#include <rclcpp/rclcpp.hpp>
#include "ik_7dof/ik_solver.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <random>
#include <memory>
#include <fstream>  
#include <chrono>   
using namespace fa_arm_kinematic;

class LeftArmIKNode : public rclcpp::Node
{
public:
    LeftArmIKNode() : Node("left_arm_ik_node")
    {
        this->declare_parameter<std::string>("urdf_file", "");
        this->declare_parameter<std::string>("srdf_file", "");
        this->declare_parameter<int>("num_tests", 10);
        this->declare_parameter<int>("max_iters", 200);
        this->declare_parameter<double>("eps", 1e-3);
        
        std::string urdf_file = this->get_parameter("urdf_file").as_string();
        std::string srdf_file = this->get_parameter("srdf_file").as_string();
        num_tests_ = this->get_parameter("num_tests").as_int();
        max_iters_ = this->get_parameter("max_iters").as_int();
        eps_ = this->get_parameter("eps").as_double();
        
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
            runValidation();
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
        RCLCPP_INFO(this->get_logger(), "关节限制（验证是否与URDF一致）:");
        
        // 期望的URDF限制
        std::vector<std::pair<double, double>> expected_limits = {
            {-3.14159, 0.785398},    // left_joint1
            {-0.087, 1.74533},        // shoulder_roll_l_joint
            {-2.26893, 2.26893},      // left_joint3
            {-2.26893, 0.1},          // elbow_l_joint
            {-2.26893, 2.26893},      // left_joint5
            {-1.309, 1.309},          // left_joint6
            {-1.309, 1.309}           // left_joint7
        };
        
        for (size_t i = 0; i < joint_names.size(); ++i) {
            double lower = limits.first[i];
            double upper = limits.second[i];
            double exp_lower = expected_limits[i].first;
            double exp_upper = expected_limits[i].second;
            
            std::string match = (std::abs(lower - exp_lower) < 0.001 && 
                                std::abs(upper - exp_upper) < 0.001) ? "✓" : "❌";
            
            RCLCPP_INFO(this->get_logger(), "  %s: [%.3f, %.3f] 期望 [%.3f, %.3f] %s", 
                    joint_names[i].c_str(), lower, upper, 
                    exp_lower, exp_upper, match.c_str());
        }
    }
    
    void runValidation()
    {
        RCLCPP_INFO(this->get_logger(), "========== 左臂正逆解验证开始 ==========");
        
        // 打开失败日志文件
        std::ofstream log_file("ik_failed_cases.log", std::ios::out);
        if (log_file.is_open()) {
            log_file << "Test_ID, Target_X, Target_Y, Target_Z, Target_R, Target_P, Target_Y, "
                    << "True_q1, True_q2, True_q3, True_q4, True_q5, True_q6, True_q7\n";
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        
        int success_count = 0;
        double total_time_ms = 0.0;
        long total_steps = 0;

        auto limits = solver_->getJointLimits();
        const auto& joint_names = solver_->getLeftArmJointNames();

        for (int test = 0; test < num_tests_; ++test) {
            RCLCPP_INFO(this->get_logger(), "\n--- 测试 %d/%d ---", test + 1, num_tests_);
            
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
            
            // RCLCPP_INFO(this->get_logger(), "随机关节角度:");
            // for (int i = 0; i < 7; ++i) {
            //     RCLCPP_INFO(this->get_logger(), "  %s: %.6f rad", 
            //             joint_names[i].c_str(), q_rand[i]);
            // }
            
            auto fk_result = solver_->computeLeftArmFK_SE3(q_rand);
            
            // 构造 Pinocchio SE3 对象作为求解器输入
            pinocchio::SE3 T_target;
            T_target.translation(fk_result.p);
            T_target.rotation(fk_result.R);
            // RCLCPP_INFO(this->get_logger(), "正运动学结果:");
            // RCLCPP_INFO(this->get_logger(), "  位置: [%.6f, %.6f, %.6f] m", 
            //         fk_result.x, fk_result.y, fk_result.z);
            // RCLCPP_INFO(this->get_logger(), "  姿态: [roll=%.6f, pitch=%.6f, yaw=%.6f] rad",
            //         fk_result.roll, fk_result.pitch, fk_result.yaw);
            
            // std::array<double, 6> target = {
            //     fk_result.x, fk_result.y, fk_result.z,
            //     fk_result.roll, fk_result.pitch, fk_result.yaw
            // };
            
            // 执行逆运动学求解并计时
            int iters = 0;
            auto start = std::chrono::high_resolution_clock::now();
            Eigen::VectorXd q_solved = solver_->solveLeftArmIK(
                T_target, Eigen::VectorXd(), max_iters_, eps_, &iters);
            auto end = std::chrono::high_resolution_clock::now();

            // 检查结果
            bool consistency_ok = false;
            bool is_failed = false;
            if (q_solved.size() == 0) { //失败
                RCLCPP_ERROR(this->get_logger(), "逆运动学求解失败！");
                is_failed = true;
            }else{//成功
                // RCLCPP_INFO(this->get_logger(), "逆运动学结果:");
                // for (int i = 0; i < 7; ++i) {
                //     RCLCPP_INFO(this->get_logger(), "  %s: %.6f rad", 
                //             joint_names[i].c_str(), q_solved[i]);
                // }
#if 0 
                auto fk_verify = solver_->computeLeftArmFK(q_solved);
                
                double pos_error = std::sqrt(
                    std::pow(fk_result.x - fk_verify.x, 2) +
                    std::pow(fk_result.y - fk_verify.y, 2) +
                    std::pow(fk_result.z - fk_verify.z, 2));

                auto rpy_to_matrix = [](double r, double p, double y) {
                    return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
                };

                Eigen::Matrix3d R_target = rpy_to_matrix(fk_result.roll, fk_result.pitch, fk_result.yaw);
                Eigen::Matrix3d R_verify = rpy_to_matrix(fk_verify.roll, fk_verify.pitch, fk_verify.yaw);
                
                Eigen::AngleAxisd angle_diff(R_target.transpose() * R_verify);
                double rot_error = std::abs(angle_diff.angle());
#endif
                // auto fk_result_se3 = solver_->computeLeftArmFK_SE3(q_rand);
                auto fk_verify_se3 = solver_->computeLeftArmFK_SE3(q_solved);

                // 位置误差
                double pos_error = (fk_result.p - fk_verify_se3.p).norm();
                
                // 姿态误差：使用角轴距离（Geodesic distance on SO(3)）
                Eigen::Matrix3d R_diff = fk_result.R.transpose() * fk_verify_se3.R;
                Eigen::AngleAxisd angle_diff(R_diff);
                double rot_error = std::abs(angle_diff.angle());
                
                // RCLCPP_INFO(this->get_logger(), "一致性验证:");
                // RCLCPP_INFO(this->get_logger(), "  位置误差: %.8f m", pos_error);
                // RCLCPP_INFO(this->get_logger(), "  姿态误差: %.8f rad", rot_error);

                if (pos_error < 1e-3 && rot_error < 1e-3) {
                    // RCLCPP_INFO(this->get_logger(), "  ✅ 正逆解一致！");
                    consistency_ok = true;
                } else {
                    RCLCPP_INFO(this->get_logger(), "  ❌ 正逆解不一致！");
                    is_failed = true;
                }

                double joint_error = (q_rand - q_solved).norm();
                // RCLCPP_INFO(this->get_logger(), "  关节角度差异: %.8f rad", joint_error);

                // 4. 仅记录成功的统计数据
                if (consistency_ok) {
                    std::chrono::duration<double, std::milli> elapsed = end - start;
                    success_count++;
                    total_time_ms += elapsed.count();
                    total_steps += iters;
                }
            }
            
            
            if (is_failed) {
                RCLCPP_WARN(this->get_logger(), "测试 %d 失败，已记录至日志", test + 1);
                Eigen::Vector3d euler = T_target.rotation().eulerAngles(2, 1, 0);
                if (log_file.is_open()) {
                    log_file << test << ", " 
                            << T_target.translation().x() << ", " << T_target.translation().y() << ", " << T_target.translation().z() << ", "
                            << euler[0] << ", " << euler[1] << ", " << euler[2] << ", "
                            << q_rand[0] << ", " << q_rand[1] << ", " << q_rand[2] << ", "
                            << q_rand[3] << ", " << q_rand[4] << ", " << q_rand[5] << ", " << q_rand[6] << "\n";
                }
            }
            
            // 每 1000 次打印一次进度
            if ((test + 1) % 1000 == 0) {
                RCLCPP_INFO(this->get_logger(), "进度: %d/%d, 当前成功率: %.1f%%", 
                            test + 1, num_tests_, 100.0 * success_count / (test + 1));
            }
            
        }
        
        RCLCPP_INFO(this->get_logger(), "\n========== 验证结果 ==========");
        RCLCPP_INFO(this->get_logger(), "总测试次数: %d", num_tests_);
        RCLCPP_INFO(this->get_logger(), "成功次数: %d", success_count);
        RCLCPP_INFO(this->get_logger(), "算法分布统计:");
        RCLCPP_INFO(this->get_logger(), "  - 第一阶段 (QR) 成功: %d", solver_->stats_.qr_stage_success);
        RCLCPP_INFO(this->get_logger(), "  - 第二阶段 (SVD) 成功: %d", solver_->stats_.svd_stage_success);
        if (success_count > 0) {
            RCLCPP_INFO(this->get_logger(), "平均成功耗时: %.4f ms", total_time_ms / success_count);
            RCLCPP_INFO(this->get_logger(), "平均成功步长: %.2f steps", (double)total_steps / success_count);
            RCLCPP_INFO(this->get_logger(), "成功率: %.1f%%", 100.0 * success_count / num_tests_);
        } else {
            RCLCPP_WARN(this->get_logger(), "没有成功的测试样本，无法统计平均值。");
        }

        if (log_file.is_open()) log_file.close();
    }
    
    std::unique_ptr<IKSolver> solver_;
    int num_tests_;
    int max_iters_;
    double eps_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeftArmIKNode>());
    rclcpp::shutdown();
    return 0;
}