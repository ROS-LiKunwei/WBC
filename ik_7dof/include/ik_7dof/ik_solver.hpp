#ifndef LEFT_ARM_IK_TEST__IK_SOLVER_HPP_
#define LEFT_ARM_IK_TEST__IK_SOLVER_HPP_

#include <string>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>

namespace left_arm_ik_test
{

struct PoseRPY
{
    double x, y, z;      // 位置
    double roll, pitch, yaw;  // 姿态
};

struct PoseSE3 {
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
};

/**
 * @brief 逆运动学求解器类
 */
class IKSolver
{
public:
    /**
     * @brief 构造函数
     * @param urdf_file URDF文件路径
     * @param srdf_file SRDF文件路径（可选）
     */
    IKSolver(const std::string& urdf_file, const std::string& srdf_file = "");
    
    /**
     * @brief 析构函数
     */
    ~IKSolver();
    
    PoseSE3 computeLeftArmFK_SE3(const Eigen::VectorXd& q);

    /**
     * @brief 核心迭代求解器
     * @param use_robust_svd 是否使用稳健但较慢的 SVD 分解模式
     * @param q_init 初始关节角度
     * @param max_iters 最大迭代次数
     * @param eps 收敛精度
     * @param iters_out 输出参数，用于保存迭代步数
     * @param use_robust_svd 是否使用稳健但较慢的 SVD 分解模式
     * @return 求解得到的7维关节角度，失败返回空向量
     */
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

    /**
     * @brief 左臂逆运动学求解
     * @param target_pose 目标位姿 [x, y, z, roll, pitch, yaw]
     * @param initial_q 初始关节角度（可选，默认使用中性位姿）
     * @param max_iters 最大迭代次数
     * @param eps 收敛精度
     * @param iterations 输出参数，用于保存迭代步数
     * @return 求解得到的7维关节角度，失败返回空向量
     */
    Eigen::VectorXd solveLeftArmIK(
        const pinocchio::SE3& T_target_in,
        const Eigen::VectorXd& initial_q = Eigen::VectorXd(),
        int max_iters = 100,
        double eps = 1e-3,
        int* iterations = nullptr); // 新增参数用于传出步数
    
    /**
     * @brief 左臂正运动学
     * @param q 关节角度（7维）
     * @return 末端执行器位姿
     */
    PoseRPY computeLeftArmFK(const Eigen::VectorXd& q);
    
    /**
     * @brief 获取关节名称
     */
    const std::vector<std::string>& getLeftArmJointNames() const { return left_arm_joints_; }
    
    /**
     * @brief 获取关节数量
     */
    size_t getLeftArmJointCount() const { return left_arm_joints_.size(); }
    
    /**
     * @brief 获取关节限制
     */
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointLimits() const;

    // 定义计数结构体
    struct IKStatistics {
        int total_calls = 0;       // 总调用次数
        int qr_stage_success = 0;  // 第一阶段 (QR/LDLT) 成功次数
        int svd_stage_success = 0; // 第二阶段 (SVD+随机重启) 成功次数
        int total_failures = 0;    // 最终失败次数

        // 重置函数
        void reset() {
            total_calls = qr_stage_success = svd_stage_success = total_failures = 0;
        }
    };
    IKStatistics stats_;
private:
    // PIMPL模式隐藏Pinocchio细节
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    
    // 左臂关节名称
    const std::vector<std::string> left_arm_joints_ = {
        "left_joint1",           // 肩关节1
        "shoulder_roll_l_joint", // 肩关节2
        "left_joint3",           // 肩关节3
        "elbow_l_joint",         // 肘关节
        "left_joint5",           // 前臂关节1
        "left_joint6",           // 前臂关节2
        "left_joint7"            // 腕关节
    };
    
    // 存储关节在模型中的索引，在构造函数中初始化
    mutable std::vector<int> q_indices_;
    mutable std::vector<int> v_indices_;

    // 末端执行器frame名称
    const std::string ee_frame_ = "L_hand_base_link";
    
    // URDF/SRDF文件路径
    std::string urdf_file_;
    std::string srdf_file_;
};

} // namespace left_arm_ik_test

#endif // LEFT_ARM_IK_TEST__IK_SOLVER_HPP_