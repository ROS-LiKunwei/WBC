#include "ik_7dof/fa_ik_solver.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>

namespace
{

bool isFiniteVector(const Eigen::VectorXd& q)
{
    for (int i = 0; i < q.size(); ++i) {
        if (!std::isfinite(q[i])) {
            return false;
        }
    }
    return true;
}

bool isWithinLimits(
    const Eigen::VectorXd& q,
    const std::pair<Eigen::VectorXd, Eigen::VectorXd>& limits)
{
    for (int i = 0; i < q.size(); ++i) {
        if (q[i] < limits.first[i] - 1e-9 || q[i] > limits.second[i] + 1e-9) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main()
{
    using namespace fa_arm_kinematic;

    const std::string urdf_file =
        std::string(IK_7DOF_WORKSPACE_SRC_DIR) + "/sysmo_description/urdf/fa_robot.urdf";
    const std::string srdf_file =
        std::string(IK_7DOF_WORKSPACE_SRC_DIR) + "/fa_moveit2_config/config/fa_robot.srdf";

    IKSolver solver(urdf_file, srdf_file);
    ArmKinematicsOptions options;
    options.reference_frame = ArmReferenceFrame::PELVIS;

    pinocchio::SE3 unreachable_target;
    unreachable_target.translation(Eigen::Vector3d(2.0, 2.0, 2.0));
    unreachable_target.rotation(Eigen::Matrix3d::Identity());

    int core_iters = 0;
    bool exact_solution = true;
    double best_error = 0.0;
    Eigen::VectorXd core_q = solver.solveIK_Core(
        unreachable_target,
        Eigen::VectorXd::Zero(7),
        80,
        1e-6,
        core_iters,
        SolverMethod::LDLT,
        ArmSide::LEFT,
        options,
        &exact_solution,
        &best_error);

    if (exact_solution) {
        std::cerr << "不可达目标不应被标记为精确收敛" << std::endl;
        return 1;
    }
    if (core_q.size() != 7 || !isFiniteVector(core_q) || !std::isfinite(best_error)) {
        std::cerr << "solveIK_Core 未返回有效的 7 维最佳近似解" << std::endl;
        return 1;
    }

    IKResult ik_result = solver.solveArmIK(
        unreachable_target,
        ArmSide::LEFT,
        Eigen::VectorXd(),
        options);

    const auto limits = solver.getArmJointLimits(ArmSide::LEFT);
    if (ik_result.success) {
        std::cerr << "不可达目标不应被标记为精确收敛" << std::endl;
        return 1;
    }
    if (!ik_result.has_solution) {
        std::cerr << "solveArmIK 对不可达目标没有返回可用近似解" << std::endl;
        return 1;
    }

    const Eigen::VectorXd& q = ik_result.q_solution;
    if (q.size() != 7) {
        std::cerr << "solveArmIK 对不可达目标返回了空解" << std::endl;
        return 1;
    }
    if (!isFiniteVector(q) || !isWithinLimits(q, limits)) {
        std::cerr << "solveArmIK 返回的近似解包含非法数值或超出关节限位" << std::endl;
        return 1;
    }
    if (q.cwiseAbs().maxCoeff() <= 1e-9) {
        std::cerr << "solveArmIK 对不可达目标返回了全 0 关节角" << std::endl;
        return 1;
    }

    const PoseSE3 approx_pose = solver.computeArmFK_SE3(q, ArmSide::LEFT, options);
    const double position_error = (unreachable_target.translation() - approx_pose.p).norm();
    if (!std::isfinite(position_error)) {
        std::cerr << "近似解 FK 结果无效" << std::endl;
        return 1;
    }
    if (!std::isfinite(ik_result.position_error) || !std::isfinite(ik_result.orientation_error)) {
        std::cerr << "IKResult 误差字段无效" << std::endl;
        return 1;
    }

    std::cout << "不可达目标近似解测试通过: position_error="
              << position_error << ", iterations=" << ik_result.iterations << std::endl;
    return 0;
}
