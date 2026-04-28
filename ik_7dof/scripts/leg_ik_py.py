#!/usr/bin/env python3
"""
Leg Inverse Kinematics Solver (Pure Python + MuJoCo)

Numerical IK solver for 6-DOF leg, using MuJoCo for FK and Jacobian computation.
Algorithm mirrors the C++ fa_ik_solver: two-stage (LDLT + SVD random restart).

Input: pelvis height from ground
Output: leg joint angles (6 per leg)

Usage:
    from leg_ik_py import solve_leg_ik, solve_both_legs_ik
    left_q, right_q = solve_both_legs_ik(model, pelvis_height=0.65)
"""

import numpy as np
from enum import Enum
from typing import Optional, Tuple, List


class SolverMethod(Enum):
    LDLT = 0
    SVD = 1


LEFT_LEG_JOINTS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
]

RIGHT_LEG_JOINTS = [
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

LEG_JOINT_LIMITS = {
    "left_hip_pitch_joint":  (-2.09, 2.09),
    "left_hip_roll_joint":   (-0.52, 2.62),
    "left_hip_yaw_joint":    (-0.79, 0.79),
    "left_knee_joint":       (-0.05, 2.09),
    "left_ankle_pitch_joint":(-1.05, 0.52),
    "left_ankle_roll_joint": (-0.35, 0.35),
    "right_hip_pitch_joint": (-2.09, 2.09),
    "right_hip_roll_joint":  (-2.62, 0.52),
    "right_hip_yaw_joint":   (-0.79, 0.79),
    "right_knee_joint":      (0.05, 2.09),
    "right_ankle_pitch_joint":(-1.05, 0.52),
    "right_ankle_roll_joint":(-0.35, 0.35),
}


def _get_joint_limits(joint_names: List[str]) -> Tuple[np.ndarray, np.ndarray]:
    lower = np.array([LEG_JOINT_LIMITS[j][0] for j in joint_names])
    upper = np.array([LEG_JOINT_LIMITS[j][1] for j in joint_names])
    return lower, upper


def _build_joint_index_map(model) -> dict:
    name_to_qpos = {}
    name_to_dof = {}
    for i in range(model.njnt):
        name = model.joint(i).name
        name_to_qpos[name] = model.jnt_qposadr[i]
    for i in range(model.nu):
        name = model.actuator(i).name
        name_to_dof[name] = i
    return name_to_qpos, name_to_dof


def _get_foot_body_id(model, side: str) -> int:
    target_name = f"{side}_ankle_roll_link"
    for i in range(model.nbody):
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) == target_name:
            return i
    return -1


def _reset_pelvis_to_origin(data):
    data.qpos[0] = 0.0
    data.qpos[1] = 0.0
    data.qpos[2] = 0.0
    data.qpos[3] = 1.0
    data.qpos[4] = 0.0
    data.qpos[5] = 0.0
    data.qpos[6] = 0.0


def _compute_foot_fk(model, data, joint_names: List[str], q: np.ndarray,
                     name_to_qpos: dict, side: str) -> Tuple[np.ndarray, np.ndarray]:
    _reset_pelvis_to_origin(data)
    for i, name in enumerate(joint_names):
        if name in name_to_qpos:
            data.qpos[name_to_qpos[name]] = q[i]
    mujoco.mj_forward(model, data)
    body_id = _get_foot_body_id(model, side)
    if body_id < 0:
        return np.zeros(3), np.eye(3)
    pos = data.xpos[body_id].copy()
    rot = data.xmat[body_id].reshape(3, 3).copy()
    return pos, rot


def _compute_foot_jacobian(model, data, joint_names: List[str],
                           name_to_qpos: dict, side: str) -> np.ndarray:
    body_id = _get_foot_body_id(model, side)
    if body_id < 0:
        return np.zeros((6, len(joint_names)))
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacp, jacr, data.xpos[body_id], body_id)
    J_full = np.vstack([jacp, jacr])
    J_sub = np.zeros((6, len(joint_names)))
    for i, name in enumerate(joint_names):
        if name in name_to_qpos:
            qpos_adr = name_to_qpos[name]
            dof_adr = model.jnt_dofadr[model.jnt_qposadr[qpos_adr] == model.jnt_qposadr].flatten()
            if len(dof_adr) > 0:
                J_sub[:, i] = J_full[:, dof_adr[0]]
    return J_sub


def _compute_foot_jacobian_v2(model, data, joint_names: List[str],
                              name_to_qpos: dict, side: str) -> np.ndarray:
    body_id = _get_foot_body_id(model, side)
    if body_id < 0:
        return np.zeros((6, len(joint_names)))
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacp, jacr, data.xpos[body_id], body_id)
    J_full = np.vstack([jacp, jacr])
    J_sub = np.zeros((6, len(joint_names)))
    for i, name in enumerate(joint_names):
        if name in name_to_qpos:
            jnt_adr = None
            for j in range(model.njnt):
                if model.joint(j).name == name:
                    jnt_adr = j
                    break
            if jnt_adr is not None:
                dof_adr = model.jnt_dofadr[jnt_adr]
                J_sub[:, i] = J_full[:, dof_adr]
    return J_sub


def _se3_error(pos_cur: np.ndarray, rot_cur: np.ndarray,
               pos_tgt: np.ndarray, rot_tgt: np.ndarray) -> np.ndarray:
    R_err = rot_tgt @ rot_cur.T
    angle = np.arccos(np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0))
    if abs(angle) < 1e-10:
        axis = np.array([0.0, 0.0, 1.0])
    else:
        axis = np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ]) / (2.0 * np.sin(angle))
        if np.linalg.norm(axis) < 1e-10:
            axis = np.array([0.0, 0.0, 1.0])
        else:
            axis = axis / np.linalg.norm(axis)
    rot_err = angle * axis
    pos_err = pos_tgt - pos_cur
    return np.concatenate([pos_err, rot_err])


def _solve_ik_core(model, data, joint_names: List[str],
                   name_to_qpos: dict, side: str,
                   target_pos: np.ndarray, target_rot: np.ndarray,
                   q_init: np.ndarray, max_iters: int = 200,
                   eps: float = 1e-3, method: SolverMethod = SolverMethod.LDLT,
                   orient_weight: float = 0.4) -> Tuple[Optional[np.ndarray], float]:
    lower, upper = _get_joint_limits(joint_names)
    n_jnt = len(joint_names)
    q = q_init.copy()
    lambda_val = 1e-2 if method == SolverMethod.LDLT else 1e-3
    best_q = q.copy()
    best_err = float('inf')

    for iteration in range(max_iters):
        pos_cur, rot_cur = _compute_foot_fk(model, data, joint_names, q, name_to_qpos, side)
        err = _se3_error(pos_cur, rot_cur, target_pos, target_rot)
        error_norm = np.linalg.norm(err)

        if error_norm < best_err:
            best_err = error_norm
            best_q = q.copy()

        if error_norm < eps:
            return q.copy(), error_norm

        if error_norm > 0.01:
            weights = np.array([1.0, 1.0, 1.0, orient_weight, orient_weight, orient_weight])
        else:
            weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        weighted_err = err * weights

        J = _compute_foot_jacobian_v2(model, data, joint_names, name_to_qpos, side)

        grad_H = np.zeros(n_jnt)
        repulsion_gain = 0.0
        if error_norm >= 1e-2:
            q_rest = np.zeros(n_jnt)
            k_rest = 0.05
            threshold_percent = 0.05
            for i in range(n_jnt):
                dist_upper = upper[i] - q[i]
                dist_lower = q[i] - lower[i]
                range_val = upper[i] - lower[i]
                margin = range_val * threshold_percent
                limit_repulsion = 0.0
                if dist_upper < margin:
                    limit_repulsion = -0.01 * ((margin - dist_upper) / margin) ** 2
                elif dist_lower < margin:
                    limit_repulsion = 0.01 * ((margin - dist_lower) / margin) ** 2
                comfort_pull = -k_rest * (q[i] - q_rest[i])
                grad_H[i] = limit_repulsion + comfort_pull
            grad_H = np.clip(grad_H, -10.0, 10.0)
            repulsion_gain = 0.0 if error_norm < 0.05 else 0.001

        g0 = repulsion_gain * grad_H

        if method == SolverMethod.LDLT:
            JJt = J @ J.T
            JJt += (lambda_val ** 2) * np.eye(6)
            modified_err = weighted_err - J @ g0
            try:
                dq = J.T @ np.linalg.solve(JJt, modified_err) + g0
            except np.linalg.LinAlgError:
                dq = g0
        else:
            U, s, Vt = np.linalg.svd(J, full_matrices=False)
            s_inv = np.where(s > 1e-4, 1.0 / s, 0.0)
            J_pinv = Vt.T @ np.diag(s_inv) @ U.T
            null_space = np.eye(n_jnt) - J_pinv @ J
            dq = J_pinv @ weighted_err + null_space @ g0

        adaptive_dt = 0.5 if error_norm > 1e-1 else 0.8
        q = q + dq * adaptive_dt
        q = np.clip(q, lower, upper)

        if error_norm < 1e-2:
            lambda_val = 1e-4 if method == SolverMethod.SVD else 1e-3
            adaptive_dt = 1.0

    if best_err < 0.15:
        return best_q, best_err
    return None, best_err


def _compute_zero_config_foot_pos(model, data, joint_names: List[str],
                                  name_to_qpos: dict, side: str) -> np.ndarray:
    q_zero = np.zeros(len(joint_names))
    pos, _ = _compute_foot_fk(model, data, joint_names, q_zero, name_to_qpos, side)
    return pos


def solve_leg_ik(model, data, pelvis_height: float, side: str,
                 foot_offset_z: float = 0.0,
                 initial_q: Optional[np.ndarray] = None,
                 max_iters: int = 500, eps: float = 1e-3) -> Optional[np.ndarray]:
    joint_names = LEFT_LEG_JOINTS if side == "left" else RIGHT_LEG_JOINTS
    name_to_qpos, _ = _build_joint_index_map(model)
    lower, upper = _get_joint_limits(joint_names)
    n_leg = len(joint_names)

    # Reference starting pose: zero config foot position
    fk_zero_pos = _compute_zero_config_foot_pos(model, data, joint_names, name_to_qpos, side)
    target_pos = np.array([fk_zero_pos[0], fk_zero_pos[1], -pelvis_height + foot_offset_z])
    target_rot = np.eye(3)

    if initial_q is not None and len(initial_q) == n_leg:
        q_start = initial_q.copy()
    else:
        q_start = np.zeros(n_leg)
        q_start[3] = np.clip(0.5, lower[3], upper[3])
        q_start[4] = np.clip(-0.3, lower[4], upper[4])

    best_result = None
    best_err = float('inf')

    for ow in [0.4, 0.1, 0.01]:
        result, err = _solve_ik_core(model, data, joint_names, name_to_qpos, side,
                                     target_pos, target_rot, q_start,
                                     max_iters=max(max_iters, 200), eps=eps,
                                     method=SolverMethod.LDLT, orient_weight=ow)
        if result is not None and err < eps:
            return result
        if result is not None and err < best_err:
            best_err = err
            best_result = result

    rng = np.random.default_rng()
    for r in range(4):
        random_q = rng.uniform(lower, upper)
        random_q[3] = np.clip(rng.uniform(0.2, 1.5), lower[3], upper[3])
        result, err = _solve_ik_core(model, data, joint_names, name_to_qpos, side,
                                     target_pos, target_rot, random_q,
                                     max_iters=max_iters, eps=eps,
                                     method=SolverMethod.SVD)
        if result is not None and err < eps:
            return result
        if result is not None and err < best_err:
            best_err = err
            best_result = result

    if best_result is not None and best_err < 0.15:
        return best_result
    return None


def solve_both_legs_ik(model, data, pelvis_height: float,
                       foot_offset_z: float = 0.0,
                       max_iters: int = 500,
                       eps: float = 1e-3) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    left_q = solve_leg_ik(model, data, pelvis_height, "left",
                          foot_offset_z, max_iters=max_iters, eps=eps)
    right_q = solve_leg_ik(model, data, pelvis_height, "right",
                           foot_offset_z, max_iters=max_iters, eps=eps)
    return left_q, right_q


try:
    import mujoco
except ImportError:
    pass
