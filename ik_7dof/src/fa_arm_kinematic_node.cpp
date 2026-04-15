#include <rclcpp/rclcpp.hpp>
#include "ik_7dof/fa_ik_solver.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <random>
#include <memory>
#include <fstream>
#include <chrono>
using namespace left_arm_ik_test;

class FaArmKinematicNode : public rclcpp::Node
{
public:
    FaArmKinematicNode() : Node("fa_arm_kinematic_node")
    {
        this->declare_parameter<std::string>("urdf_file", "");
        this->declare_parameter<std::string>("srdf_file", "");
        this->declare_parameter<std::string>("arm_side", "left");
        this->declare_parameter<int>("num_tests", 10);
        this->declare_parameter<int>("max_iters", 200);
        this->declare_parameter<double>("eps", 1e-3);

        std::string urdf_file = this->get_parameter("urdf_file").as_string();
        std::string srdf_file = this->get_parameter("srdf_file").as_string();
        std::string arm_side_str = this->get_parameter("arm_side").as_string();
        num_tests_ = this->get_parameter("num_tests").as_int();
        max_iters_ = this->get_parameter("max_iters").as_int();
        eps_ = this->get_parameter("eps").as_double();

        if (urdf_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "必须提供urdf_file参数");
            return;
        }

        if (arm_side_str == "right") {
            arm_side_ = ArmSide::RIGHT;
            arm_name_ = "右臂";
        } else {
            arm_side_ = ArmSide::LEFT;
            arm_name_ = "左臂";
        }

        RCLCPP_INFO(this->get_logger(), "URDF文件: %s", urdf_file.c_str());
        RCLCPP_INFO(this->get_logger(), "SRDF文件: %s", srdf_file.c_str());
        RCLCPP_INFO(this->get_logger(), "手臂: %s", arm_name_.c_str());
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
        RCLCPP_INFO(this->get_logger(), "FA%s关节 (%zu 个):", arm_name_.c_str(), solver_->getArmJointCount(arm_side_));
        const auto& joint_names = solver_->getArmJointNames(arm_side_);
        for (size_t i = 0; i < joint_names.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  %2zu: %s", i, joint_names[i].c_str());
        }

        auto limits = solver_->getArmJointLimits(arm_side_);
        RCLCPP_INFO(this->get_logger(), "关节限制（从URDF读取）:");
        for (size_t i = 0; i < joint_names.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  %s: [%.3f, %.3f]",
                joint_names[i].c_str(), limits.first[i], limits.second[i]);
        }
    }

    void runValidation()
    {
        RCLCPP_INFO(this->get_logger(), "========== FA%s正逆解验证开始 ==========", arm_name_.c_str());

        std::string log_filename = std::string("fa_") + (arm_side_ == ArmSide::LEFT ? "left" : "right") + "_arm_ik_failed_cases.log";
        std::ofstream log_file(log_filename, std::ios::out);
        if (log_file.is_open()) {
            log_file << "Test_ID, Target_X, Target_Y, Target_Z, Target_R, Target_P, Target_Y, "
                     << "True_q1, True_q2, True_q3, True_q4, True_q5, True_q6, True_q7\n";
        }

        std::random_device rd;
        std::mt19937 gen(rd());

        int success_count = 0;
        double total_time_ms = 0.0;
        long total_steps = 0;

        auto limits = solver_->getArmJointLimits(arm_side_);
        const auto& joint_names = solver_->getArmJointNames(arm_side_);

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

            auto fk_result = solver_->computeArmFK_SE3(q_rand, arm_side_);

            pinocchio::SE3 T_target;
            T_target.translation(fk_result.p);
            T_target.rotation(fk_result.R);

            int iters = 0;
            auto start = std::chrono::high_resolution_clock::now();
            Eigen::VectorXd q_solved = solver_->solveArmIK(
                T_target, arm_side_, Eigen::VectorXd(), max_iters_, eps_, &iters);
            auto end = std::chrono::high_resolution_clock::now();

            bool consistency_ok = false;
            bool is_failed = false;
            if (q_solved.size() == 0) {
                RCLCPP_ERROR(this->get_logger(), "逆运动学求解失败！");
                is_failed = true;
            } else {
                auto fk_verify_se3 = solver_->computeArmFK_SE3(q_solved, arm_side_);

                double pos_error = (fk_result.p - fk_verify_se3.p).norm();
                Eigen::Matrix3d R_diff = fk_result.R.transpose() * fk_verify_se3.R;
                Eigen::AngleAxisd angle_diff(R_diff);
                double rot_error = std::abs(angle_diff.angle());

                if (pos_error < 1e-3 && rot_error < 1e-3) {
                    consistency_ok = true;
                } else {
                    RCLCPP_INFO(this->get_logger(), "  ❌ 正逆解不一致！");
                    is_failed = true;
                }

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

            if ((test + 1) % 1000 == 0) {
                RCLCPP_INFO(this->get_logger(), "进度: %d/%d, 当前成功率: %.1f%%",
                    test + 1, num_tests_, 100.0 * success_count / (test + 1));
            }
        }

        RCLCPP_INFO(this->get_logger(), "\n========== 验证结果 ==========");
        RCLCPP_INFO(this->get_logger(), "总测试次数: %d", num_tests_);
        RCLCPP_INFO(this->get_logger(), "成功次数: %d", success_count);
        RCLCPP_INFO(this->get_logger(), "算法分布统计:");
        RCLCPP_INFO(this->get_logger(), "  - 第一阶段 (LDLT) 成功: %d", solver_->stats_.qr_stage_success);
        RCLCPP_INFO(this->get_logger(), "  - 第二阶段 (SVD) 成功: %d", solver_->stats_.svd_stage_success);
        if (success_count > 0) {
            RCLCPP_INFO(this->get_logger(), "平均成功耗时: %.4f ms", total_time_ms / success_count);
            RCLCPP_INFO(this->get_logger(), "平均成功步长: %.2f steps", static_cast<double>(total_steps) / success_count);
            RCLCPP_INFO(this->get_logger(), "成功率: %.1f%%", 100.0 * success_count / num_tests_);
        } else {
            RCLCPP_WARN(this->get_logger(), "没有成功的测试样本，无法统计平均值。");
        }

        if (log_file.is_open()) {
            log_file.close();
            RCLCPP_INFO(this->get_logger(), "失败案例已保存至: %s", log_filename.c_str());
        }
    }

    std::unique_ptr<IKSolver> solver_;
    int num_tests_;
    int max_iters_;
    double eps_;
    ArmSide arm_side_;
    std::string arm_name_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaArmKinematicNode>());
    rclcpp::shutdown();
    return 0;
}