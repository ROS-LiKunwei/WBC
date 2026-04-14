#ifndef LEFT_ARM_IK_TEST__FA_IK_SOLVER_HPP_
#define LEFT_ARM_IK_TEST__FA_IK_SOLVER_HPP_

#include <string>
#include <vector>
#include <array>
#include <memory>
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>

namespace left_arm_ik_test
{

struct PoseRPY
{
    double x, y, z;
    double roll, pitch, yaw;
};

struct PoseSE3 {
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
};

class IKSolver
{
public:
    IKSolver(const std::string& urdf_file, const std::string& srdf_file = "");
    ~IKSolver();

    PoseSE3 computeLeftArmFK_SE3(const Eigen::VectorXd& q);

    Eigen::VectorXd solveIK_Core(
        const pinocchio::SE3& T_target,
        const Eigen::VectorXd& q_init,
        int max_iters,
        double eps,
        int& iters_out,
        bool use_robust_svd);

    Eigen::VectorXd solveIK_Internal(
        const pinocchio::SE3& T_target,
        const Eigen::VectorXd& q_init,
        int max_iters,
        double eps,
        int& iters_out);

    Eigen::VectorXd solveLeftArmIK(
        const pinocchio::SE3& T_target_in,
        const Eigen::VectorXd& initial_q = Eigen::VectorXd(),
        int max_iters = 100,
        double eps = 1e-3,
        int* iterations = nullptr);

    PoseRPY computeLeftArmFK(const Eigen::VectorXd& q);

    const std::vector<std::string>& getLeftArmJointNames() const { return left_arm_joints_; }
    size_t getLeftArmJointCount() const { return left_arm_joints_.size(); }

    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointLimits() const;

    struct IKStatistics {
        int total_calls = 0;
        int qr_stage_success = 0;
        int svd_stage_success = 0;
        int total_failures = 0;

        void reset() {
            total_calls = qr_stage_success = svd_stage_success = total_failures = 0;
        }
    };
    IKStatistics stats_;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;

    // fa_robot 左臂 7 关节
    const std::vector<std::string> left_arm_joints_ = {
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_yaw_joint",
        "left_wrist_pitch_joint",
        "left_wrist_roll_joint"
    };

    mutable std::vector<int> q_indices_;
    mutable std::vector<int> v_indices_;

    // fa_robot 左手末端 frame
    const std::string ee_frame_ = "left_hand_base_link";

    std::string urdf_file_;
    std::string srdf_file_;
};

} // namespace left_arm_ik_test

#endif // LEFT_ARM_IK_TEST__FA_IK_SOLVER_HPP_
