#include "ik_7dof/ik_solver.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/spatial/log.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <memory>

namespace fa_arm_kinematic
{

// PIMPL实现
// 这个内部类封装了所有Pinocchio相关的实现细节
class IKSolver::Impl
{
public:
    Impl(const std::string& urdf_file, const std::string& srdf_file)
        : urdf_file_(urdf_file), srdf_file_(srdf_file)
    {
        // 加载URDF模型
        // buildModel()解析URDF文件，构建运动学树
        pinocchio::urdf::buildModel(urdf_file_, model_);
        
        // 如果提供了SRDF，加载关节限制和默认配置
        if (!srdf_file_.empty()) {
            pinocchio::srdf::loadReferenceConfigurations(model_, srdf_file_, true);
        }
        
        // 3. 创建数据对象
        // Data存储计算过程中的中间结果
        data_ = std::make_unique<pinocchio::Data>(model_);
        
        std::cout << "模型加载成功!" << std::endl;
        std::cout << "  关节数量: " << model_.nq << std::endl;
        std::cout << "  框架数量: " << model_.nframes << std::endl;
    }
    
    ~Impl() = default;
    
    // RPY到SE3齐次变换矩阵转换 顺序：Z(roll) * Y(pitch) * X(yaw)
    pinocchio::SE3 rpyzyxToSE3(double x, double y, double z, double roll, double pitch, double yaw)
    {
        Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d R = Rz.toRotationMatrix() * Ry.toRotationMatrix() * Rx.toRotationMatrix();
        return pinocchio::SE3(R, Eigen::Vector3d(x, y, z));
    }
    
    // 获取末端执行器Frame ID
    pinocchio::FrameIndex getEeFrameId(const std::string& ee_frame)
    {
        if (model_.existFrame(ee_frame)) {
            return model_.getFrameId(ee_frame);
        }
        throw std::runtime_error("Frame not found: " + ee_frame);
    }
    
    // 获取关节索引：建立**“关节名称”与“数值向量索引”**之间的映射
    // 在机器人学中，不同的关节可能有不同的自由度（DoF）。例如，旋转关节占 1 个位置，而球关节可能占 4 个位置（四元数表示 q）和 3 个速度（角速度 v）。因此，不能简单地通过关节 ID 来遍历，必须查找它们在全局向量中的具体索引。
    // q_indices: 配置空间索引(model.nq) 描述机器人姿态，包括关节角/位置以及浮动基姿态（如果有）
    // v_indices: 速度空间索引(model.nv) 描述机器人关节速度，包括关节角速度和基座速度
    bool getJointIndices(const std::vector<std::string>& joint_names,
                     std::vector<int>& q_indices,
                     std::vector<int>& v_indices) const
    {
        q_indices.clear();
        v_indices.clear();
        
        // std::cout << "\n关节索引映射详情:" << std::endl;
        // std::cout << std::left << std::setw(5) << "序号" 
        //         << std::setw(25) << "关节名称" 
        //         << std::setw(10) << "JointID"
        //         << std::setw(10) << "q_idx"
        //         << std::setw(10) << "v_idx"
        //         << std::setw(15) << "下限"
        //         << std::setw(15) << "上限" << std::endl;
        // std::cout << std::string(85, '-') << std::endl;
        
        int index = 0;
        for (const auto& name : joint_names) {
            if (!model_.existJointName(name)) {
                std::cerr << "警告: 关节 '" << name << "' 未找到!" << std::endl;
                continue;
            }
            pinocchio::JointIndex jid = model_.getJointId(name);
            int q_idx = model_.joints[jid].idx_q();
            int v_idx = model_.joints[jid].idx_v();
            
            q_indices.push_back(q_idx);
            v_indices.push_back(v_idx);
            
            // std::cout << std::left << std::setw(5) << index++
            //         << std::setw(25) << name
            //         << std::setw(10) << jid
            //         << std::setw(10) << q_idx
            //         << std::setw(10) << v_idx
            //         << std::setw(15) << model_.lowerPositionLimit[q_idx]
            //         << std::setw(15) << model_.upperPositionLimit[q_idx]
            //         << std::endl;
        }
        
        return !q_indices.empty();
    }
    
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    std::string urdf_file_;
    std::string srdf_file_;
};

// IKSolver类的公共接口
IKSolver::IKSolver(const std::string& urdf_file, const std::string& srdf_file)
    : urdf_file_(urdf_file), srdf_file_(srdf_file)
{
    pimpl_ = std::make_unique<Impl>(urdf_file, srdf_file);
    stats_.reset();
    // 根据实际的 URDF 文件，左臂关节的正确 q_idx 应该是：
    // left_joint1: 需要找到正确的索引
    // shoulder_roll_l_joint: 需要找到正确的索引
    // left_joint3: 需要找到正确的索引
    // ...
    
    // 我们先获取所有关节的索引，然后手动映射
    std::vector<int> temp_q_indices, temp_v_indices;
    pimpl_->getJointIndices(left_arm_joints_, temp_q_indices, temp_v_indices);
    
    // 打印实际获取的索引
    std::cout << "Pinocchio返回的索引:" << std::endl;
    for (size_t i = 0; i < left_arm_joints_.size(); ++i) {
        std::cout << "  " << left_arm_joints_[i] << ": q_idx=" << temp_q_indices[i] << std::endl;
    }
    
    // 根据 check_joint_order 的输出，左臂关节的 q_idx 应该是：
    // left_joint1: 12
    // shoulder_roll_l_joint: 13
    // left_joint3: 14
    // elbow_l_joint: 15
    // left_joint5: 16
    // left_joint6: 17
    // left_joint7: 18
    
    // 但我们需要验证这些索引对应的限制是否正确
    std::cout << "\n验证索引对应的限制:" << std::endl;
    for (size_t i = 0; i < left_arm_joints_.size(); ++i) {
        int idx = temp_q_indices[i];
        std::cout << "  " << left_arm_joints_[i] << " (idx=" << idx << "): ["
                  << pimpl_->model_.lowerPositionLimit[idx] << ", "
                  << pimpl_->model_.upperPositionLimit[idx] << "]" << std::endl;
    }
    
    // 使用获取到的索引
    q_indices_ = temp_q_indices;
    v_indices_ = temp_v_indices;
}

IKSolver::~IKSolver() = default;

Eigen::VectorXd IKSolver::solveIK_Core(
    const pinocchio::SE3& T_target,
    const Eigen::VectorXd& q_init,
    int max_iters,
    double eps,
    int& iters_out,
    bool use_robust_svd)
{
    using namespace pinocchio;
    using namespace Eigen;

    Model& model = pimpl_->model_;
    Data& data = *pimpl_->data_;
    FrameIndex ee_fid = pimpl_->getEeFrameId(ee_frame_);
    const int n_arm = left_arm_joints_.size();

    VectorXd q = neutral(model);
    for (int i = 0; i < n_arm; ++i) {
        int qi = model.joints[model.getJointId(left_arm_joints_[i])].idx_q();
        q[qi] = q_init[i];
    }

    // --- 根据模式设置初始阻 ---
    // 快速模式 (LDLT/QR) 建议阻尼稍微大一点，增加稳定性
    // 稳健模式 (SVD) 阻尼可以小一点，因为 SVD 本身有截断逻辑
    double lambda = use_robust_svd ? 1e-3 : 1e-2;
    VectorXd grad_H = VectorXd::Zero(n_arm);
    
    for (int iter = 0; iter < max_iters; ++iter) {
        iters_out = iter + 1;

        forwardKinematics(model, data, q);
        updateFramePlacements(model, data);

        SE3 T_current = data.oMf[ee_fid];
        Motion err_motion = log6(T_current.actInv(T_target));
        VectorXd err = err_motion.toVector(); // 6维向量: [v_x, v_y, v_z, w_x, w_y, w_z]

        double error_norm = err.norm();

        // 精度达标则返回
        if (error_norm < eps) {
            VectorXd res(n_arm);
            for (int i = 0; i < n_arm; ++i)
                res[i] = q[model.joints[model.getJointId(left_arm_joints_[i])].idx_q()];
            return res;
        }

        // --- 动态权重分配 ---
        Eigen::VectorXd weights(6);
        
        if (error_norm > 0.01) {
            // 第一阶段：粗调。位置权重1.0，姿态权重0.4。
            // 优先让手臂伸到目标点附近，防止姿态奇异点把手臂带飞。
            weights << 1.0, 1.0, 1.0, 0.4, 0.4, 0.4;
        }else {
            // 第二阶段：精调。恢复 1:1 的权重。
            // 当位置已经很接近了（< 1cm），必须恢复姿态权重，否则 1e-3 的精度永远达不到。
            weights << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        }
        // 应用权重
        VectorXd weighted_err = err.cwiseProduct(weights);

        // 计算雅可比
        MatrixXd J_full = MatrixXd::Zero(6, model.nv);
        computeFrameJacobian(model, data, q, ee_fid, LOCAL, J_full);
        
        MatrixXd J_arm(6, n_arm);
        for (int i = 0; i < n_arm; ++i) {
            J_arm.col(i) = J_full.col(model.joints[model.getJointId(left_arm_joints_[i])].idx_v());
        }

        VectorXd dq;
        
        
        if (!use_robust_svd) {
            MatrixXd JJt = J_arm * J_arm.transpose();
            JJt.diagonal().array() += (lambda * lambda);
            // 使用加权后的 weighted_err
            // dq = J_arm.transpose() * JJt.colPivHouseholderQr().solve(weighted_err);
            dq = J_arm.transpose() * JJt.ldlt().solve(weighted_err);
        } else {
            JacobiSVD<MatrixXd> svd(J_arm, ComputeThinU | ComputeThinV);
            VectorXd sv = svd.singularValues();
            VectorXd sv_inv = sv;
            for (int i = 0; i < sv.size(); ++i) {
                // 提高奇异值截断阈值到 1e-4 以增强稳定性
                sv_inv(i) = (sv(i) > 1e-4) ? (1.0 / sv(i)) : 0.0;
            }

            MatrixXd J_pinv = svd.matrixV() * sv_inv.asDiagonal() * svd.matrixU().transpose();
            
            // 基础解使用加权后的 weighted_err
            dq = J_pinv * weighted_err;

            // 冗余项：零空间投影
            auto limits = getJointLimits();
            VectorXd q_arm(n_arm);
            for(int i=0; i<n_arm; ++i){
                q_arm[i] = q[model.joints[model.getJointId(left_arm_joints_[i])].idx_q()];
            } 
            
            
            const double threshold_percent = 0.05; // 5% 的边缘触发力场

            for(int i=0; i<n_arm; ++i) {
                double dist_to_upper = limits.second[i] - q_arm[i];
                double dist_to_lower = q_arm[i] - limits.first[i];
                double range = limits.second[i] - limits.first[i];
                double margin = range * threshold_percent; // 触发危险区的距离
                
                if (dist_to_upper < margin) 
                    grad_H[i] = -0.01 * std::pow((0.1 - dist_to_upper)/0.1, 2); // 越近推力越大
                else if (dist_to_lower < margin)
                    grad_H[i] = 0.01 * std::pow((0.1 - dist_to_lower)/0.1, 2);
                // 安全区：grad_H[i] 保持为 0，不干扰主任务求解
            }

            // 对斥力梯度做简单的限幅，防止分母过小导致数值爆炸
            double max_grad = 10.0;
            for(int i=0; i<n_arm; ++i) 
                grad_H[i] = std::max(-max_grad, std::min(max_grad, grad_H[i]));

            // 投影到零空间
            MatrixXd J_pinvZero = svd.matrixV() * sv_inv.asDiagonal() * svd.matrixU().transpose();
            MatrixXd NullSpace = MatrixXd::Identity(n_arm, n_arm) - J_pinvZero * J_arm;

            // 最终增量 = 任务增量 + 零空间斥力
            // 注意：这里的增益系数可以根据实际效果微调,将其作为一种“软约束”而非“强推力”。
            double repulsion_gain = (error_norm < 0.05) ? 0.0 : 0.001;
            dq += NullSpace * (repulsion_gain * grad_H);
        }

        // --- 【修改点 2: 自适应步长】 ---
        // 误差大时（离目标远），步长小一点防止飞掉；误差小时（接近目标），步长大一点加速收敛;这样即便斥力很大，也会被 adaptive_dt 压住，保证平稳地推回安全区。
        double adaptive_dt = (error_norm > 1e-1) ? 0.2 : 0.6; 

        // 更新 q 并截断限位
        for (int i = 0; i < n_arm; ++i) {
            int qi = model.joints[model.getJointId(left_arm_joints_[i])].idx_q();
            q[qi] += dq[i] * adaptive_dt; // 使用自适应步长
            q[qi] = std::max(model.lowerPositionLimit[qi], std::min(model.upperPositionLimit[qi], q[qi]));
        }

        // 只有当误差确实很小时，才减小阻尼以加速最后收敛
        if (error_norm < 1e-2) {
            lambda = use_robust_svd ? 1e-4 : 1e-3;
            // 进入精调阶段：关闭零空间，使用全步长
            adaptive_dt = 1.0;
            // 如果是 SVD 模式，可以将 grad_H 置零
            grad_H.setZero();
        }
    }

    return VectorXd();
}

// 内部实际执行迭代的函数（被 solveLeftArmIK 调用）
Eigen::VectorXd IKSolver::solveIK_Internal(
    const pinocchio::SE3& T_target,
    const Eigen::VectorXd& q_init,
    int max_iters,
    double eps,
    int& iters_out)
{
    using namespace pinocchio;
    using namespace Eigen;

    Model& model = pimpl_->model_;
    Data& data = *pimpl_->data_;
    FrameIndex ee_fid = pimpl_->getEeFrameId(ee_frame_);
    const int n_arm = left_arm_joints_.size();

    // 初始化全量 q 向量
    VectorXd q = neutral(model);
    // 将传入的 7 维初值映射到 model 的正确位置
    for (int i = 0; i < n_arm; ++i) {
        int qi = model.joints[model.getJointId(left_arm_joints_[i])].idx_q();
        q[qi] = q_init[i];
    }

    double lambda = 1e-3; // 阻尼系数
    const double dt = 0.5; // 步长系数
    
    for (int iter = 0; iter < max_iters; ++iter) {
        iters_out = iter + 1; // 记录当前步数

        // 1. 更新运动学
        forwardKinematics(model, data, q);
        updateFramePlacements(model, data);

        // 2. 计算当前位姿误差 (LOCAL 坐标系)
        SE3 T_current = data.oMf[ee_fid];
        Motion err_motion = log6(T_current.actInv(T_target));
        Eigen::VectorXd err = err_motion.toVector();

        // 检查精度
        if (err.norm() < eps) {
            VectorXd res(n_arm);
            for (int i = 0; i < n_arm; ++i)
                res[i] = q[model.joints[model.getJointId(left_arm_joints_[i])].idx_q()];
            return res;
        }

        // 3. 计算雅可比矩阵 (6 x 7)
        MatrixXd J_full = MatrixXd::Zero(6, model.nv);
        computeFrameJacobian(model, data, q, ee_fid, LOCAL, J_full);
        
        MatrixXd J_arm(6, n_arm);
        for (int i = 0; i < n_arm; ++i) {
            int vi = model.joints[model.getJointId(left_arm_joints_[i])].idx_v();
            J_arm.col(i) = J_full.col(vi);
        }

        // 4. Levenberg-Marquardt 求解
        MatrixXd JJt = J_arm * J_arm.transpose();
        JJt.diagonal().array() += (lambda * lambda);
        // 使用 QR 分解获得更稳定的逆
        VectorXd dq = J_arm.transpose() * JJt.colPivHouseholderQr().solve(err);

        // 5. 更新 q 并应用硬限位约束
        for (int i = 0; i < n_arm; ++i) {
            int qi = model.joints[model.getJointId(left_arm_joints_[i])].idx_q();
            q[qi] += dq[i] * dt;
            
            // 强制限制在 URDF 定义的范围内
            q[qi] = std::max(model.lowerPositionLimit[qi], 
                    std::min(model.upperPositionLimit[qi], q[qi]));
        }

        // 动态调整阻尼 (简单的启发式)
        if (err.norm() < 0.1) lambda = 1e-4;
    }

    return VectorXd(); // 本次尝试失败
}



Eigen::VectorXd IKSolver::solveLeftArmIK(
    const pinocchio::SE3& T_target_in,
    const Eigen::VectorXd& initial_q,
    int max_iters,
    double eps,
    int* iterations)
{
    int total_iters = 0;
    int current_iters = 0;

    // 1. 转换目标位姿
    // pinocchio::SE3 T_target = pimpl_->rpyzyxToSE3(
    //     target_pose[0], target_pose[1], target_pose[2],
    //     target_pose[3], target_pose[4], target_pose[5]);

    // --- 姿态正交化/四元数化 ---
    // 目的：修正 RPY 转换过程中可能产生的微小非正交误差，确保旋转矩阵严格有效
    pinocchio::SE3 T_target = T_target_in;
    Eigen::Matrix3d R = T_target.rotation();
    Eigen::Quaterniond q_rot(R);     // 将矩阵转为四元数
    q_rot.normalize();               // 归一化四元数
    T_target.rotation(q_rot.toRotationMatrix()); // 转回旋转矩阵并重新赋值给 T_target

    // 2. 准备初始值
    Eigen::VectorXd q_start = (initial_q.size() == 7) ? initial_q : (getJointLimits().first + getJointLimits().second) / 2.0;

    // 3. 第一次尝试(尝试快速求解 (LDLT/QR))
    int first_stage_iters = std::max(max_iters, 200);
    Eigen::VectorXd result = solveIK_Core(T_target, q_start, first_stage_iters, eps, current_iters, false);
    total_iters += current_iters;

    if (result.size() > 0){
        if (iterations) *iterations = total_iters;
        stats_.qr_stage_success++; // QR 阶段成功
        return result;
    } 

    // 4. 【核心改动】如果失败，进行随机多点重启 (Multi-Start)
    // 7-DOF 很容易陷入局部极小值，随机重启能解决 99% 的失败
    // int max_restarts = 5;
    // auto limits = getJointLimits();
    // std::mt19937 gen(12345); // 固定种子或 std::random_device()

    // for (int r = 0; r < max_restarts; ++r) {
    //     Eigen::VectorXd random_q(7);
    //     int retry_iters = 0;
    //     for (int i = 0; i < 7; ++i) {
    //         std::uniform_real_distribution<double> dist(limits.first[i], limits.second[i]);
    //         random_q[i] = dist(gen);
    //     }

    //     result = solveIK_Internal(T_target, random_q, max_iters, eps, retry_iters);
    //     current_iters += retry_iters; // 累加尝试步数
    //     if (result.size() > 0) {
    //         if (iterations) *iterations = current_iters;
    //         return result;
    //     } 
    // }

    // --- 第二阶段：快速求解失败，尝试 SVD 稳健模式 + 随机重启 ---
    auto limits = getJointLimits();
    std::mt19937 gen(std::random_device{}()); // 使用真实随机数种子

    for (int r = 0; r < 10; ++r) {
        Eigen::VectorXd random_q(7);
        for (int i = 0; i < 7; ++i) {
            std::uniform_real_distribution<double> dist(limits.first[i], limits.second[i]);
            random_q[i] = dist(gen);
        }

        result = solveIK_Core(T_target, random_q, max_iters, eps, current_iters, true);
        total_iters += current_iters;
        
        if (result.size() > 0) {
            if (iterations) *iterations = total_iters;
            stats_.svd_stage_success++; // SVD 重启阶段成功
            return result;
        }
    }

    if (iterations) *iterations = total_iters;
    return Eigen::VectorXd(); // 最终失败
}

PoseRPY IKSolver::computeLeftArmFK(const Eigen::VectorXd& q)
{
    using namespace pinocchio;
    
    // 获取关节索引
    std::vector<int> q_indices, v_indices;
    pimpl_->getJointIndices(left_arm_joints_, q_indices, v_indices);
    
    // 构建完整q向量
    Eigen::VectorXd q_full = neutral(pimpl_->model_);
    for (size_t i = 0; i < q.size() && i < q_indices.size(); ++i) {
        q_full[q_indices[i]] = q[i];
    }
    
    // 正运动学
    forwardKinematics(pimpl_->model_, *pimpl_->data_, q_full);
    updateFramePlacements(pimpl_->model_, *pimpl_->data_);
    
    // 获取末端位姿
    FrameIndex ee_fid = pimpl_->getEeFrameId(ee_frame_);
    SE3 T = pimpl_->data_->oMf[ee_fid];
    
    // 转换为RPY
    PoseRPY pose;
    Eigen::Vector3d t = T.translation();
    pose.x = t.x();
    pose.y = t.y();
    pose.z = t.z();
    
    Eigen::Matrix3d R = T.rotation();
    if (std::abs(R(2,0)) < 1.0 - 1e-6) {
        pose.pitch = std::asin(-R(2,0));
        pose.roll  = std::atan2(R(2,1), R(2,2));
        pose.yaw   = std::atan2(R(1,0), R(0,0));
    } else {
        pose.pitch = (R(2,0) <= -1.0) ? M_PI/2 : -M_PI/2;
        pose.roll  = 0.0;
        pose.yaw   = std::atan2(-R(0,1), R(1,1));
    }
    
    return pose;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> IKSolver::getJointLimits() const {
    size_t n = left_arm_joints_.size();
    Eigen::VectorXd lower(n), upper(n);
    
    for (size_t i = 0; i < n; ++i) {
        // 关键：通过关节名称获取模型中的真实 ID
        if (!pimpl_->model_.existJointName(left_arm_joints_[i])) {
            continue; 
        }
        auto jid = pimpl_->model_.getJointId(left_arm_joints_[i]);
        // 获取该关节在全局 q 向量中的起始索引
        int q_idx = pimpl_->model_.joints[jid].idx_q();
        
        // 直接从 Pinocchio 加载的 URDF 数据中读取
        lower[i] = pimpl_->model_.lowerPositionLimit[q_idx];
        upper[i] = pimpl_->model_.upperPositionLimit[q_idx];
    }
    return {lower, upper};
}

PoseSE3 IKSolver::computeLeftArmFK_SE3(const Eigen::VectorXd& q)
{
    using namespace pinocchio;
    
    // 1. 将 7 维 q 映射到全量 q_full (处理有腰部或其它关节的情况)
    Eigen::VectorXd q_full = neutral(pimpl_->model_);
    for (size_t i = 0; i < left_arm_joints_.size(); ++i) {
        int qi = pimpl_->model_.joints[pimpl_->model_.getJointId(left_arm_joints_[i])].idx_q();
        q_full[qi] = q[i];
    }

    // 2. 更新运动学
    forwardKinematics(pimpl_->model_, *pimpl_->data_, q_full);
    updateFramePlacements(pimpl_->model_, *pimpl_->data_);
    
    // 3. 获取末端 Frame 的 SE3 变换
    FrameIndex ee_fid = pimpl_->getEeFrameId(ee_frame_);
    SE3 T = pimpl_->data_->oMf[ee_fid];
    
    // 4. 封装结果
    PoseSE3 pose;
    pose.p = T.translation(); // 提取位置
    pose.R = T.rotation();    // 提取旋转矩阵
    
    return pose;
}

} // namespace fa_arm_kinematic