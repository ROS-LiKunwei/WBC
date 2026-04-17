#ifndef FA_ARM_KINEMATIC__FA_IK_SOLVER_HPP_
#define FA_ARM_KINEMATIC__FA_IK_SOLVER_HPP_

#include <string>
#include <vector>
#include <array>
#include <memory>
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>

namespace fa_arm_kinematic
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

enum class ArmSide {
    LEFT,
    RIGHT
};

enum class SolverMethod {
    LDLT,
    SVD
};

class IKSolver
{
public:
    IKSolver(const std::string& urdf_file, const std::string& srdf_file = "");
    ~IKSolver();

    PoseSE3 computeArmFK_SE3(const Eigen::VectorXd& q, ArmSide arm_side);

    Eigen::VectorXd solveIK_Core(
        const pinocchio::SE3& T_target,
        const Eigen::VectorXd& q_init,
        int max_iters,
        double eps,
        int& iters_out,
        SolverMethod method,
        ArmSide arm_side = ArmSide::LEFT);

    Eigen::VectorXd solveArmIK(
        const pinocchio::SE3& T_target_in,
        ArmSide arm_side,
        const Eigen::VectorXd& initial_q = Eigen::VectorXd(),
        int max_iters = 100,
        double eps = 1e-3,
        int* iterations = nullptr);

    PoseRPY computeArmFK(const Eigen::VectorXd& q, ArmSide arm_side);

    const std::vector<std::string>& getArmJointNames(ArmSide arm_side) const;
    size_t getArmJointCount(ArmSide arm_side) const;

    std::pair<Eigen::VectorXd, Eigen::VectorXd> getArmJointLimits(ArmSide arm_side) const;

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

    // fa_robot 右臂 7 关节
    const std::vector<std::string> right_arm_joints_ = {
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_yaw_joint",
        "right_wrist_pitch_joint",
        "right_wrist_roll_joint"
    };

    mutable std::vector<int> q_indices_;
    mutable std::vector<int> v_indices_;

    // fa_robot 左手末端 frame
    const std::string left_ee_frame_ = "left_hand_base_link";
    // fa_robot 右手末端 frame
    const std::string right_ee_frame_ = "hand_base_link";

    std::string urdf_file_;
    std::string srdf_file_;
};

} // namespace fa_arm_kinematic

#endif // FA_ARM_KINEMATIC__FA_IK_SOLVER_HPP_
