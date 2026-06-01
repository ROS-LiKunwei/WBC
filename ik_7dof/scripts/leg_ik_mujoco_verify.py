#!/usr/bin/env python3
"""
Leg IK MuJoCo Verification Script

Input: pelvis height from ground
Output: leg joint angles

Usage:
    python3 leg_ik_mujoco_verify.py --pelvis_height 0.65
    python3 leg_ik_mujoco_verify.py --pelvis_height 0.65 --knee_max 0.5 --headless
    python3 leg_ik_mujoco_verify.py --left_q 0.1,0.0,0.0,0.5,-0.3,0.0 --right_q 0.1,0.0,0.0,0.5,-0.3,0.0
"""

import argparse
import os
import sys
import numpy as np
import signal

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("Error: mujoco package not found. Install with: pip install mujoco")
    sys.exit(1)

from leg_ik_py import LEFT_LEG_JOINTS, RIGHT_LEG_JOINTS, LEG_JOINT_LIMITS

MUJOCO_MODEL_PATH = os.path.join(
    os.path.dirname(__file__), "..", "sysmo_description", "mjcf",
    "fa_robot_combined_body_collision_modified.xml"
)

LEG_JOINT_NAMES = LEFT_LEG_JOINTS + RIGHT_LEG_JOINTS

FOOT_SOLE_OFFSET_Z = 0.05 # Dist between ankle joint and sole of foot
FIXED_KNEE_ANGLE = 0.0
HARD_HEIGHT_TOL = 1e-3
HARD_ANKLE_X_TOL = 1e-3
HARD_ANKLE_Y_TOL = 1e-3
HARD_ORIENTATION_TOL_RAD = np.deg2rad(0.1)


def find_mujoco_model():
    search_paths = [
        MUJOCO_MODEL_PATH,
        os.path.join(os.path.dirname(__file__), "..", "..", "src", "sysmo_description",
                     "mjcf", "fa_robot_combined_body_collision_modified.xml"),
        os.path.expanduser("~/humanoid_ws/src/sysmo_description/mjcf/fa_robot_combined_body_collision_modified.xml"),
    ]
    for p in search_paths:
        p = os.path.abspath(p)
        if os.path.exists(p):
            return p
    return None


def rotation_error_vector(rot_cur, rot_tgt=None):
    if rot_tgt is None:
        rot_tgt = np.eye(3)
    rot_err_mat = rot_tgt @ rot_cur.T
    angle = np.arccos(np.clip((np.trace(rot_err_mat) - 1.0) / 2.0, -1.0, 1.0))
    if abs(angle) < 1e-10:
        return np.zeros(3)
    axis = np.array([
        rot_err_mat[2, 1] - rot_err_mat[1, 2],
        rot_err_mat[0, 2] - rot_err_mat[2, 0],
        rot_err_mat[1, 0] - rot_err_mat[0, 1],
    ]) / (2.0 * np.sin(angle))
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-10:
        return np.array([0.0, 0.0, angle])
    return angle * axis / axis_norm


def rpy_to_rot(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rot_x = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ])
    rot_y = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ])
    rot_z = np.array([
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ])
    return rot_z @ rot_y @ rot_x


def resolve_ankle_target_y(model, side, ankle_y, name_to_qpos=None):
    if ankle_y is not None:
        return ankle_y
    zero_offset = get_zero_config_ankle_offset(model, side, name_to_qpos)
    return zero_offset[1]


def build_joint_maps(model):
    name_to_qpos = {}
    name_to_joint_id = {}
    for i in range(model.njnt):
        name = model.joint(i).name
        name_to_qpos[name] = model.jnt_qposadr[i]
        name_to_joint_id[name] = i
    return name_to_qpos, name_to_joint_id


def compute_single_leg_fk(model, data, joint_names, q, name_to_qpos, foot_body_id):
    data.qpos[:] = 0.0
    data.qpos[3] = 1.0
    for i, name in enumerate(joint_names):
        if name in name_to_qpos:
            data.qpos[name_to_qpos[name]] = q[i]
    mujoco.mj_forward(model, data)
    pos = data.xpos[foot_body_id].copy()
    rot = data.xmat[foot_body_id].reshape(3, 3).copy()
    return pos, rot


def get_zero_config_ankle_offset(model, side, name_to_qpos=None):
    joint_names = LEFT_LEG_JOINTS if side == "left" else RIGHT_LEG_JOINTS
    if name_to_qpos is None:
        name_to_qpos, _ = build_joint_maps(model)
    body_ids = get_body_ids(model)
    pelvis_id = body_ids.get("pelvis", -1)
    foot_body_id = body_ids.get(f"{side}_ankle_roll_link", -1)
    if pelvis_id < 0 or foot_body_id < 0:
        return np.zeros(3)

    zero_data = mujoco.MjData(model)
    zero_q = np.zeros(len(joint_names))
    compute_single_leg_fk(
        model, zero_data, joint_names, zero_q, name_to_qpos, foot_body_id
    )
    return zero_data.xpos[foot_body_id].copy() - zero_data.xpos[pelvis_id].copy()


def get_zero_config_pelvis_height(model, foot_offset_z=0.0):
    name_to_qpos, _ = build_joint_maps(model)
    left_offset = get_zero_config_ankle_offset(model, "left", name_to_qpos)
    right_offset = get_zero_config_ankle_offset(model, "right", name_to_qpos)
    ankle_offset_z = 0.5 * (left_offset[2] + right_offset[2])
    return foot_offset_z + FOOT_SOLE_OFFSET_Z - ankle_offset_z


def make_fixed_knee_initial_guesses(model, data, joint_names, side, target_z,
                                    name_to_qpos, foot_body_id):
    zero_q = np.zeros(len(joint_names))
    zero_pos, _ = compute_single_leg_fk(
        model, data, joint_names, zero_q, name_to_qpos, foot_body_id
    )
    straight_leg_z = max(abs(zero_pos[2]), 1e-6)
    pitch_guess = np.arccos(np.clip(abs(target_z) / straight_leg_z, 0.0, 1.0))
    side_sign = -1.0 if side == "left" else 1.0

    guesses = []
    for pitch_sign in (1.0, -1.0):
        for roll_abs in (0.0, 0.15, 0.3):
            q = np.zeros(len(joint_names))
            q[0] = pitch_sign * pitch_guess
            q[1] = side_sign * roll_abs
            q[2] = -side_sign * 0.10 if roll_abs > 0.0 else 0.0
            q[3] = FIXED_KNEE_ANGLE
            q[4] = -pitch_sign * pitch_guess
            q[5] = -side_sign * roll_abs
            guesses.append(q)

    guesses.append(zero_q)
    return guesses


def make_knee_bounded_initial_guesses(model, data, joint_names, side, target_z,
                                      knee_max, name_to_qpos, foot_body_id):
    zero_q = np.zeros(len(joint_names))
    zero_pos, _ = compute_single_leg_fk(
        model, data, joint_names, zero_q, name_to_qpos, foot_body_id
    )
    straight_leg_z = max(abs(zero_pos[2]), 1e-6)
    pitch_guess = np.arccos(np.clip(abs(target_z) / straight_leg_z, 0.0, 1.0))
    side_sign = -1.0 if side == "left" else 1.0
    knee_candidates = sorted(set([
        0.0,
        float(np.clip(knee_max * 0.5, 0.0, knee_max)),
        float(knee_max),
    ]))

    guesses = []
    for knee in knee_candidates:
        for pitch_sign in (1.0, -1.0):
            for roll_abs in (0.0, 0.15, 0.3):
                q = np.zeros(len(joint_names))
                q[0] = pitch_sign * pitch_guess - 0.5 * knee
                q[1] = side_sign * roll_abs
                q[2] = -side_sign * 0.10 if roll_abs > 0.0 else 0.0
                q[3] = knee
                q[4] = -q[0] - knee
                q[5] = -side_sign * roll_abs
                guesses.append(q)

    guesses.append(zero_q)
    return guesses


def solve_knee_bounded_hard_pose(model, data, pelvis_height, side, knee_max,
                                 foot_offset_z=0.0, ankle_x=0.0,
                                 ankle_y=None, target_rot=None,
                                 max_iters=220, eps=1e-5):
    joint_names = LEFT_LEG_JOINTS if side == "left" else RIGHT_LEG_JOINTS
    knee_name = f"{side}_knee_joint"
    knee_idx = joint_names.index(knee_name)

    body_ids = get_body_ids(model)
    foot_body_id = body_ids.get(f"{side}_ankle_roll_link", -1)
    if foot_body_id < 0:
        return None, float("inf")

    name_to_qpos, name_to_joint_id = build_joint_maps(model)
    if target_rot is None:
        target_rot = np.eye(3)
    target_x = ankle_x
    target_y = resolve_ankle_target_y(model, side, ankle_y, name_to_qpos)
    target_z = -pelvis_height + foot_offset_z + FOOT_SOLE_OFFSET_Z
    lower = np.array([LEG_JOINT_LIMITS[name][0] for name in joint_names])
    upper = np.array([LEG_JOINT_LIMITS[name][1] for name in joint_names])
    lower[knee_idx] = 0.0
    upper[knee_idx] = min(max(knee_max, 0.0), upper[knee_idx])
    if upper[knee_idx] < lower[knee_idx]:
        return None, float("inf")

    guesses = make_knee_bounded_initial_guesses(
        model, data, joint_names, side, target_z, upper[knee_idx], name_to_qpos, foot_body_id
    )
    best_q = None
    best_err = float("inf")

    for q_start in guesses:
        q = np.clip(q_start.copy(), lower, upper)

        for _ in range(max_iters):
            pos_cur, rot_cur = compute_single_leg_fk(
                model, data, joint_names, q, name_to_qpos, foot_body_id
            )
            err = np.concatenate((
                np.array([target_x - pos_cur[0], target_y - pos_cur[1], target_z - pos_cur[2]]),
                rotation_error_vector(rot_cur, target_rot),
            ))
            err_norm = np.linalg.norm(err)
            if err_norm < best_err:
                best_err = err_norm
                best_q = q.copy()
            if err_norm < eps:
                return q.copy(), err_norm

            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            mujoco.mj_jac(model, data, jacp, jacr, data.xpos[foot_body_id], foot_body_id)

            jac_cols = []
            for idx, name in enumerate(joint_names):
                joint_id = name_to_joint_id[name]
                dof_adr = model.jnt_dofadr[joint_id]
                jac_cols.append(np.concatenate((
                    np.array([jacp[0, dof_adr], jacp[1, dof_adr], jacp[2, dof_adr]]),
                    jacr[:, dof_adr],
                )))
            jac_task = np.array(jac_cols).T

            damping = 1e-4
            try:
                dq = np.linalg.solve(
                    jac_task.T @ jac_task + damping * np.eye(len(joint_names)),
                    jac_task.T @ err,
                )
            except np.linalg.LinAlgError:
                continue

            step = 0.8 if err_norm < 0.1 else 0.5
            q = np.clip(q + step * dq, lower, upper)

    if best_err < eps:
        return best_q, best_err
    if best_q is not None:
        best_pos, best_rot = compute_single_leg_fk(
            model, data, joint_names, best_q, name_to_qpos, foot_body_id
        )
        pos_err = np.array([target_x - best_pos[0], target_y - best_pos[1], target_z - best_pos[2]])
        rot_err_norm = np.linalg.norm(rotation_error_vector(best_rot, target_rot))
        if (
            abs(pos_err[0]) <= HARD_ANKLE_X_TOL
            and abs(pos_err[1]) <= HARD_ANKLE_Y_TOL
            and abs(pos_err[2]) <= HARD_HEIGHT_TOL
            and rot_err_norm <= HARD_ORIENTATION_TOL_RAD
        ):
            return best_q, best_err
    return None, best_err


def solve_both_knee_bounded_hard_pose(model, data, pelvis_height, knee_max,
                                      foot_offset_z=0.0, ankle_x=0.0,
                                      left_ankle_y=None, right_ankle_y=None,
                                      target_rot=None, max_iters=220):
    left_q, left_err = solve_knee_bounded_hard_pose(
        model, data, pelvis_height, "left", knee_max,
        foot_offset_z=foot_offset_z, ankle_x=ankle_x,
        ankle_y=left_ankle_y, target_rot=target_rot, max_iters=max_iters
    )
    if left_q is None:
        return None, None, left_err

    right_q, right_err = solve_knee_bounded_hard_pose(
        model, data, pelvis_height, "right", knee_max,
        foot_offset_z=foot_offset_z, ankle_x=ankle_x,
        ankle_y=right_ankle_y, target_rot=target_rot, max_iters=max_iters
    )
    if right_q is None:
        return None, None, right_err

    return left_q, right_q, max(left_err, right_err)


def find_closest_pelvis_height_for_knee_max(model, data, target_height, knee_max,
                                            foot_offset_z=0.0, ankle_x=0.0,
                                            left_ankle_y=None, right_ankle_y=None,
                                            target_rot=None):
    left_q, right_q, err = solve_both_knee_bounded_hard_pose(
        model, data, target_height, knee_max, foot_offset_z=foot_offset_z,
        ankle_x=ankle_x, left_ankle_y=left_ankle_y, right_ankle_y=right_ankle_y,
        target_rot=target_rot,
        max_iters=120
    )
    if left_q is not None and right_q is not None:
        return target_height, left_q, right_q, err

    zero_height = get_zero_config_pelvis_height(model, foot_offset_z=foot_offset_z)
    zero_left_q, zero_right_q, zero_err = solve_both_knee_bounded_hard_pose(
        model, data, zero_height, knee_max, foot_offset_z=foot_offset_z,
        ankle_x=ankle_x, left_ankle_y=left_ankle_y, right_ankle_y=right_ankle_y,
        target_rot=target_rot,
        max_iters=120
    )
    if zero_left_q is None or zero_right_q is None:
        best_height = None
        best_left_q = None
        best_right_q = None
        best_err = min(err, zero_err)
        for direction in (1.0, -1.0):
            for step_i in range(1, 41):
                candidate_height = target_height + direction * 0.02 * step_i
                if candidate_height <= 0.0:
                    continue
                candidate_left_q, candidate_right_q, candidate_err = solve_both_knee_bounded_hard_pose(
                    model, data, candidate_height, knee_max,
                    foot_offset_z=foot_offset_z, ankle_x=ankle_x,
                    left_ankle_y=left_ankle_y, right_ankle_y=right_ankle_y,
                    target_rot=target_rot, max_iters=80
                )
                if candidate_left_q is None or candidate_right_q is None:
                    best_err = min(best_err, candidate_err)
                    continue
                if best_height is None or abs(candidate_height - target_height) < abs(best_height - target_height):
                    best_height = candidate_height
                    best_left_q = candidate_left_q
                    best_right_q = candidate_right_q
                    best_err = candidate_err
                break
        return best_height, best_left_q, best_right_q, best_err

    if target_height >= zero_height:
        return zero_height, zero_left_q, zero_right_q, zero_err

    low = target_height
    high = zero_height
    best_height = zero_height
    best_left_q = zero_left_q
    best_right_q = zero_right_q
    best_err = zero_err

    for _ in range(8):
        mid = 0.5 * (low + high)
        mid_left_q, mid_right_q, mid_err = solve_both_knee_bounded_hard_pose(
            model, data, mid, knee_max, foot_offset_z=foot_offset_z,
            ankle_x=ankle_x, left_ankle_y=left_ankle_y, right_ankle_y=right_ankle_y,
            target_rot=target_rot,
            max_iters=80
        )
        if mid_left_q is not None and mid_right_q is not None:
            best_height = mid
            best_left_q = mid_left_q
            best_right_q = mid_right_q
            best_err = mid_err
            high = mid
        else:
            low = mid

    return best_height, best_left_q, best_right_q, best_err


def solve_fixed_knee_hard_pose(model, data, pelvis_height, side,
                               foot_offset_z=0.0, ankle_x=0.0,
                               ankle_y=None, target_rot=None,
                               max_iters=160, eps=1e-5):
    joint_names = LEFT_LEG_JOINTS if side == "left" else RIGHT_LEG_JOINTS
    knee_name = f"{side}_knee_joint"
    knee_idx = joint_names.index(knee_name)
    active_indices = [i for i in range(len(joint_names)) if i != knee_idx]

    body_ids = get_body_ids(model)
    foot_body_id = body_ids.get(f"{side}_ankle_roll_link", -1)
    if foot_body_id < 0:
        return None, float("inf")

    name_to_qpos, name_to_joint_id = build_joint_maps(model)
    if target_rot is None:
        target_rot = np.eye(3)
    target_x = ankle_x
    target_y = resolve_ankle_target_y(model, side, ankle_y, name_to_qpos)
    target_z = -pelvis_height + foot_offset_z + FOOT_SOLE_OFFSET_Z
    lower = np.array([LEG_JOINT_LIMITS[name][0] for name in joint_names])
    upper = np.array([LEG_JOINT_LIMITS[name][1] for name in joint_names])
    lower[knee_idx] = FIXED_KNEE_ANGLE
    upper[knee_idx] = FIXED_KNEE_ANGLE

    guesses = make_fixed_knee_initial_guesses(
        model, data, joint_names, side, target_z, name_to_qpos, foot_body_id
    )
    best_q = None
    best_err = float("inf")

    for q_start in guesses:
        q = np.clip(q_start.copy(), lower, upper)
        q[knee_idx] = FIXED_KNEE_ANGLE

        for _ in range(max_iters):
            pos_cur, rot_cur = compute_single_leg_fk(
                model, data, joint_names, q, name_to_qpos, foot_body_id
            )
            err = np.concatenate((
                np.array([target_x - pos_cur[0], target_y - pos_cur[1], target_z - pos_cur[2]]),
                rotation_error_vector(rot_cur, target_rot),
            ))
            err_norm = np.linalg.norm(err)
            if err_norm < best_err:
                best_err = err_norm
                best_q = q.copy()
            if err_norm < eps:
                return q.copy(), err_norm

            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            mujoco.mj_jac(model, data, jacp, jacr, data.xpos[foot_body_id], foot_body_id)

            jac_cols = []
            for idx in active_indices:
                joint_id = name_to_joint_id[joint_names[idx]]
                dof_adr = model.jnt_dofadr[joint_id]
                jac_cols.append(np.concatenate((
                    np.array([jacp[0, dof_adr], jacp[1, dof_adr], jacp[2, dof_adr]]),
                    jacr[:, dof_adr],
                )))
            jac_task = np.array(jac_cols).T

            damping = 1e-4
            try:
                dq_active = np.linalg.solve(
                    jac_task.T @ jac_task + damping * np.eye(len(active_indices)),
                    jac_task.T @ err,
                )
            except np.linalg.LinAlgError:
                continue

            step = 0.8 if err_norm < 0.1 else 0.5
            for active_i, dq in zip(active_indices, dq_active):
                q[active_i] += step * dq
            q = np.clip(q, lower, upper)
            q[knee_idx] = FIXED_KNEE_ANGLE

    if best_err < eps:
        return best_q, best_err
    if best_q is not None:
        best_pos, best_rot = compute_single_leg_fk(
            model, data, joint_names, best_q, name_to_qpos, foot_body_id
        )
        pos_err = np.array([target_x - best_pos[0], target_y - best_pos[1], target_z - best_pos[2]])
        rot_err_norm = np.linalg.norm(rotation_error_vector(best_rot, target_rot))
        if (
            abs(pos_err[0]) <= HARD_ANKLE_X_TOL
            and abs(pos_err[1]) <= HARD_ANKLE_Y_TOL
            and abs(pos_err[2]) <= HARD_HEIGHT_TOL
            and rot_err_norm <= HARD_ORIENTATION_TOL_RAD
        ):
            return best_q, best_err
    return None, best_err



def set_leg_joints(model, data, left_q, right_q):
    joint_name_to_qpos = {}
    for i in range(model.njnt):
        name = model.joint(i).name
        qpos_adr = model.jnt_qposadr[i]
        joint_name_to_qpos[name] = qpos_adr

    for i, name in enumerate(LEFT_LEG_JOINTS):
        if name in joint_name_to_qpos and i < len(left_q):
            data.qpos[joint_name_to_qpos[name]] = left_q[i]

    for i, name in enumerate(RIGHT_LEG_JOINTS):
        if name in joint_name_to_qpos and i < len(right_q):
            data.qpos[joint_name_to_qpos[name]] = right_q[i]


def set_leg_controls(model, data, left_q, right_q):
    actuator_name_to_ctrl = {}
    for i in range(model.nu):
        name = model.actuator(i).name
        actuator_name_to_ctrl[name] = i

    for i, name in enumerate(LEFT_LEG_JOINTS):
        if name in actuator_name_to_ctrl and i < len(left_q):
            data.ctrl[actuator_name_to_ctrl[name]] = left_q[i]

    for i, name in enumerate(RIGHT_LEG_JOINTS):
        if name in actuator_name_to_ctrl and i < len(right_q):
            data.ctrl[actuator_name_to_ctrl[name]] = right_q[i]


def enforce_fixed_pose(model, data, left_q, right_q, pelvis_pos, pelvis_rot):
    set_leg_joints(model, data, left_q, right_q)
    set_leg_controls(model, data, left_q, right_q)
    data.qpos[:3] = pelvis_pos
    data.qpos[3:7] = pelvis_rot
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def get_body_ids(model):
    body_name_to_id = {}
    for i in range(model.nbody):
        body_name_to_id[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)] = i
    return body_name_to_id


def verify_foot_pose(model, data, pelvis_height, foot_offset_z=0.0,
                     ankle_x=0.0, left_ankle_y=None, right_ankle_y=None,
                     target_rot=None):
    body_ids = get_body_ids(model)
    pelvis_id = body_ids.get("pelvis", -1)
    left_foot_id = body_ids.get("left_ankle_roll_link", -1)
    right_foot_id = body_ids.get("right_ankle_roll_link", -1)

    pelvis_pos = data.xpos[pelvis_id].copy() if pelvis_id >= 0 else np.array([0, 0, 1])
    if target_rot is None:
        target_rot = np.eye(3)
    left_y_offset = resolve_ankle_target_y(model, "left", left_ankle_y)
    right_y_offset = resolve_ankle_target_y(model, "right", right_ankle_y)
    constraint_errors = {
        "pelvis_height": abs(pelvis_pos[2] - pelvis_height),
        "max_ankle_x": 0.0,
        "max_ankle_y": 0.0,
        "max_sole_z": 0.0,
        "max_orientation": 0.0,
    }

    print("\n===== MuJoCo FK验证 =====")
    print(f"骨盆位置: ({pelvis_pos[0]:.4f}, {pelvis_pos[1]:.4f}, {pelvis_pos[2]:.4f})")
    print(f"  骨盆高度误差: {pelvis_pos[2] - pelvis_height:.6f} m")

    if left_foot_id >= 0:
        left_pos = data.xpos[left_foot_id]
        left_rot = data.xmat[left_foot_id].reshape(3, 3)
        left_x_target = pelvis_pos[0] + ankle_x
        left_x_err = left_pos[0] - left_x_target
        left_y_target = pelvis_pos[1] + left_y_offset
        left_y_err = left_pos[1] - left_y_target
        left_sole_z = left_pos[2] - FOOT_SOLE_OFFSET_Z
        left_sole_err = left_sole_z - foot_offset_z
        print(f"左脚位置: ({left_pos[0]:.4f}, {left_pos[1]:.4f}, {left_pos[2]:.4f})")
        print(f"  左脚踝x误差: {left_x_err:.6f} m (目标偏移: {ankle_x:.6f} m)")
        print(f"  左脚踝y误差: {left_y_err:.6f} m (目标偏移: {left_y_offset:.6f} m)")
        print(f"  左脚底z: {left_sole_z:.4f} m (目标: {foot_offset_z:.4f} m, 误差: {left_sole_err:.6f} m)")
        print(f"  骨盆->左脚踝距离: {pelvis_pos[2] - left_pos[2]:.4f} m")
        angle_err = np.linalg.norm(rotation_error_vector(left_rot, target_rot))
        print(f"  左脚朝向误差: {np.degrees(angle_err):.2f} deg")
        constraint_errors["max_ankle_x"] = max(
            constraint_errors["max_ankle_x"], abs(left_x_err)
        )
        constraint_errors["max_ankle_y"] = max(
            constraint_errors["max_ankle_y"], abs(left_y_err)
        )
        constraint_errors["max_sole_z"] = max(
            constraint_errors["max_sole_z"], abs(left_sole_err)
        )
        constraint_errors["max_orientation"] = max(
            constraint_errors["max_orientation"], angle_err
        )

    if right_foot_id >= 0:
        right_pos = data.xpos[right_foot_id]
        right_rot = data.xmat[right_foot_id].reshape(3, 3)
        right_x_target = pelvis_pos[0] + ankle_x
        right_x_err = right_pos[0] - right_x_target
        right_y_target = pelvis_pos[1] + right_y_offset
        right_y_err = right_pos[1] - right_y_target
        right_sole_z = right_pos[2] - FOOT_SOLE_OFFSET_Z
        right_sole_err = right_sole_z - foot_offset_z
        print(f"右脚位置: ({right_pos[0]:.4f}, {right_pos[1]:.4f}, {right_pos[2]:.4f})")
        print(f"  右脚踝x误差: {right_x_err:.6f} m (目标偏移: {ankle_x:.6f} m)")
        print(f"  右脚踝y误差: {right_y_err:.6f} m (目标偏移: {right_y_offset:.6f} m)")
        print(f"  右脚底z: {right_sole_z:.4f} m (目标: {foot_offset_z:.4f} m, 误差: {right_sole_err:.6f} m)")
        print(f"  骨盆->右脚踝距离: {pelvis_pos[2] - right_pos[2]:.4f} m")
        angle_err = np.linalg.norm(rotation_error_vector(right_rot, target_rot))
        print(f"  右脚朝向误差: {np.degrees(angle_err):.2f} deg")
        constraint_errors["max_ankle_x"] = max(
            constraint_errors["max_ankle_x"], abs(right_x_err)
        )
        constraint_errors["max_ankle_y"] = max(
            constraint_errors["max_ankle_y"], abs(right_y_err)
        )
        constraint_errors["max_sole_z"] = max(
            constraint_errors["max_sole_z"], abs(right_sole_err)
        )
        constraint_errors["max_orientation"] = max(
            constraint_errors["max_orientation"], angle_err
        )

    return constraint_errors


def hard_constraints_passed(constraint_errors):
    return (
        constraint_errors["pelvis_height"] <= HARD_HEIGHT_TOL
        and constraint_errors["max_ankle_x"] <= HARD_ANKLE_X_TOL
        and constraint_errors["max_ankle_y"] <= HARD_ANKLE_Y_TOL
        and constraint_errors["max_sole_z"] <= HARD_HEIGHT_TOL
        and constraint_errors["max_orientation"] <= HARD_ORIENTATION_TOL_RAD
    )


def print_hard_constraint_status(constraint_errors):
    status = "PASS" if hard_constraints_passed(constraint_errors) else "FAIL"
    print("\n===== 硬约束验证 =====")
    print(f"  状态: {status}")
    print(f"  骨盆高度误差: {constraint_errors['pelvis_height']:.6f} m")
    print(f"  最大脚踝x误差: {constraint_errors['max_ankle_x']:.6f} m")
    print(f"  最大脚踝y误差: {constraint_errors['max_ankle_y']:.6f} m")
    print(f"  最大脚底高度误差: {constraint_errors['max_sole_z']:.6f} m")
    print(f"  最大脚朝向误差: {np.degrees(constraint_errors['max_orientation']):.4f} deg")


def main():
    def signal_handler(sig, frame):
        print("\nCtrl+C detected, exiting...")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser(description="Leg IK MuJoCo Verification (Pure Python)")
    parser.add_argument("--pelvis_height", type=float, default=0.65,
                        help="Pelvis height from ground (m)")
    parser.add_argument("--foot_offset_z", type=float, default=0.0,
                        help="Foot sole offset from ankle joint (m)")
    parser.add_argument("--left_q", type=str, default=None,
                        help="Left leg joint angles (comma-separated, 6 values)")
    parser.add_argument("--right_q", type=str, default=None,
                        help="Right leg joint angles (comma-separated, 6 values)")
    parser.add_argument("--knee_max", type=float, default=None,
                        help="Maximum knee joint angle for automatic IK search (rad)")
    parser.add_argument("--ankle_x", type=float, default=0.0,
                        help="Target ankle_roll_link.x offset from pelvis.x (m)")
    parser.add_argument("--left_ankle_y", type=float, default=None,
                        help="Target left_ankle_roll_link.y offset from pelvis.y (m); default uses zero-leg offset")
    parser.add_argument("--right_ankle_y", type=float, default=None,
                        help="Target right_ankle_roll_link.y offset from pelvis.y (m); default uses zero-leg offset")
    parser.add_argument("--ankle_roll", type=float, default=0.0,
                        help="Target ankle_roll_link roll angle in world frame (rad)")
    parser.add_argument("--ankle_pitch", type=float, default=0.0,
                        help="Target ankle_roll_link pitch angle in world frame (rad)")
    parser.add_argument("--ankle_yaw", type=float, default=0.0,
                        help="Target ankle_roll_link yaw angle in world frame (rad)")
    parser.add_argument("--headless", action="store_true",
                        help="Run without interactive viewer (just verify FK)")
    parser.add_argument("--sim_time", type=float, default=10.0,
                        help="Simulation time in seconds before pausing (0=run indefinitely)")
    args = parser.parse_args()

    model_path = find_mujoco_model()
    if model_path is None:
        print("Error: MuJoCo model file not found!")
        sys.exit(1)

    print(f"MuJoCo模型: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    effective_pelvis_height = args.pelvis_height
    target_rot = rpy_to_rot(args.ankle_roll, args.ankle_pitch, args.ankle_yaw)
    left_ankle_y = resolve_ankle_target_y(model, "left", args.left_ankle_y)
    right_ankle_y = resolve_ankle_target_y(model, "right", args.right_ankle_y)

    print(
        "脚踝目标偏移/姿态: "
        f"x={args.ankle_x:.6f} m, left_y={left_ankle_y:.6f} m, "
        f"right_y={right_ankle_y:.6f} m, "
        f"rpy=({args.ankle_roll:.6f}, {args.ankle_pitch:.6f}, {args.ankle_yaw:.6f}) rad"
    )

    left_q = None
    right_q = None
    left_q_from_cli = args.left_q is not None
    right_q_from_cli = args.right_q is not None

    if left_q_from_cli:
        left_q = np.array([float(x) for x in args.left_q.split(",")])
        print(f"使用命令行指定的左腿关节角: {left_q.tolist()}")

    if right_q_from_cli:
        right_q = np.array([float(x) for x in args.right_q.split(",")])
        print(f"使用命令行指定的右腿关节角: {right_q.tolist()}")

    if args.knee_max is not None and (left_q_from_cli or right_q_from_cli):
        print("Error: --knee_max is only supported for automatic IK; do not combine it with --left_q/--right_q")
        sys.exit(1)

    if left_q is None or right_q is None:
        if args.knee_max is not None:
            if args.knee_max < 0.0:
                print("Error: --knee_max must be non-negative")
                sys.exit(1)
            print(
                f"\n调用膝关节上限硬约束IK搜索器 "
                f"(目标骨盆高度={args.pelvis_height}m, knee_max={args.knee_max}rad)..."
            )
            effective_pelvis_height, left_q, right_q, bounded_err = find_closest_pelvis_height_for_knee_max(
                model, data, args.pelvis_height, args.knee_max,
                foot_offset_z=args.foot_offset_z,
                ankle_x=args.ankle_x,
                left_ankle_y=left_ankle_y,
                right_ankle_y=right_ankle_y,
                target_rot=target_rot,
            )
            if left_q is None or right_q is None:
                print(
                    "膝关节上限IK搜索失败: knee不超过给定最大值、脚踝目标x/y偏移、"
                    f"脚底高度、脚朝向无法同时满足，硬约束残差={bounded_err:.6f}"
                )
                sys.exit(1)
            print(f"目标骨盆高度: {args.pelvis_height:.6f} m")
            print(f"采用骨盆高度: {effective_pelvis_height:.6f} m")
            print(f"骨盆高度偏差: {effective_pelvis_height - args.pelvis_height:.6f} m")
            print(f"膝关节上限IK结果: 左腿={[f'{v:.6f}' for v in left_q]} 右腿={[f'{v:.6f}' for v in right_q]} (残差={bounded_err:.6f})")
            print(f"膝关节角度上限: left_knee_joint/right_knee_joint <= {args.knee_max:.6f} rad")
        else:
            print(f"\n调用固定膝关节硬约束IK求解器 (骨盆高度={args.pelvis_height}m)...")

            if left_q is None:
                left_q, left_err = solve_fixed_knee_hard_pose(
                    model, data, args.pelvis_height, "left",
                    foot_offset_z=args.foot_offset_z,
                    ankle_x=args.ankle_x,
                    ankle_y=left_ankle_y,
                    target_rot=target_rot,
                )
                if left_q is None:
                    print(
                        "左腿固定膝关节IK求解失败: knee=0、脚踝目标x/y偏移、脚底高度、脚朝向无法同时满足，"
                        f"硬约束残差={left_err:.6f}"
                    )
                    sys.exit(1)
                print(f"固定膝关节IK结果: 左腿={[f'{v:.6f}' for v in left_q]} (残差={left_err:.6f})")

            if right_q is None:
                right_q, right_err = solve_fixed_knee_hard_pose(
                    model, data, args.pelvis_height, "right",
                    foot_offset_z=args.foot_offset_z,
                    ankle_x=args.ankle_x,
                    ankle_y=right_ankle_y,
                    target_rot=target_rot,
                )
                if right_q is None:
                    print(
                        "右腿固定膝关节IK求解失败: knee=0、脚踝目标x/y偏移、脚底高度、脚朝向无法同时满足，"
                        f"硬约束残差={right_err:.6f}"
                    )
                    sys.exit(1)
                print(f"固定膝关节IK结果: 右腿={[f'{v:.6f}' for v in right_q]} (残差={right_err:.6f})")

            fixed_joint_names = []
            if not left_q_from_cli:
                fixed_joint_names.append("left_knee_joint")
            if not right_q_from_cli:
                fixed_joint_names.append("right_knee_joint")
            fixed_text = ", ".join(
                f"{name}={FIXED_KNEE_ANGLE:.2f}" for name in fixed_joint_names
            )
            print(f"固定膝关节角度: {fixed_text}")

    set_leg_joints(model, data, left_q, right_q)

    data.qpos[0] = 0.0
    data.qpos[1] = 0.0
    data.qpos[2] = effective_pelvis_height
    data.qpos[3] = 1.0
    data.qpos[4] = 0.0
    data.qpos[5] = 0.0
    data.qpos[6] = 0.0

    mujoco.mj_forward(model, data)

    body_ids = get_body_ids(model)
    left_foot_id = body_ids.get("left_ankle_roll_link", -1)
    right_foot_id = body_ids.get("right_ankle_roll_link", -1)

    set_leg_controls(model, data, left_q, right_q)

    constraint_errors = verify_foot_pose(
        model, data, effective_pelvis_height, args.foot_offset_z,
        ankle_x=args.ankle_x, left_ankle_y=left_ankle_y,
        right_ankle_y=right_ankle_y, target_rot=target_rot
    )
    print_hard_constraint_status(constraint_errors)
    if not hard_constraints_passed(constraint_errors):
        sys.exit(1)

    print("\n===== 关节角度 =====")
    joint_name_to_qpos = {}
    for i in range(model.njnt):
        name = model.joint(i).name
        qpos_adr = model.jnt_qposadr[i]
        joint_name_to_qpos[name] = qpos_adr

    for name in LEG_JOINT_NAMES:
        if name in joint_name_to_qpos:
            rad = float(data.qpos[joint_name_to_qpos[name]])
            deg = float(np.degrees(rad))
            print(f"  {name}: {rad:.6f} rad  ({deg:.3f} deg)")

    if args.headless:
        print("\n===== 模型状态 =====")
        print(f"  骨盆位置: ({data.qpos[0]:.4f}, {data.qpos[1]:.4f}, {data.qpos[2]:.4f})")
        if left_foot_id >= 0:
            print(f"  左脚位置: ({data.xpos[left_foot_id][0]:.4f}, {data.xpos[left_foot_id][1]:.4f}, {data.xpos[left_foot_id][2]:.4f})")
        if right_foot_id >= 0:
            print(f"  右脚位置: ({data.xpos[right_foot_id][0]:.4f}, {data.xpos[right_foot_id][1]:.4f}, {data.xpos[right_foot_id][2]:.4f})")
        
        # 在headless模式下进行仿真
        if args.sim_time > 0:
            print(f"\n在Headless模式下进行 {args.sim_time} 秒仿真...")
            start_time = data.time
            # 保存初始骨盆位置和姿态
            initial_pelvis_pos = data.qpos[:3].copy()
            initial_pelvis_rot = data.qpos[3:7].copy()
            enforce_fixed_pose(model, data, left_q, right_q, initial_pelvis_pos, initial_pelvis_rot)
            while data.time - start_time < args.sim_time:
                enforce_fixed_pose(model, data, left_q, right_q, initial_pelvis_pos, initial_pelvis_rot)
                mujoco.mj_step(model, data)
                enforce_fixed_pose(model, data, left_q, right_q, initial_pelvis_pos, initial_pelvis_rot)
            print(f"仿真完成，当前时间: {data.time:.2f}秒")
            # 验证脚的位置
            sim_constraint_errors = verify_foot_pose(
                model, data, effective_pelvis_height, args.foot_offset_z,
                ankle_x=args.ankle_x, left_ankle_y=left_ankle_y,
                right_ankle_y=right_ankle_y, target_rot=target_rot
            )
            print_hard_constraint_status(sim_constraint_errors)
            if not hard_constraints_passed(sim_constraint_errors):
                sys.exit(1)
        
        print("\nHeadless模式，模型状态已设置")
        return

    print("\n启动MuJoCo交互式查看器...")
    print("提示: 可以用鼠标旋转/缩放视图，按Esc退出")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 保存初始骨盆位置和姿态
        initial_pelvis_pos = data.qpos[:3].copy()
        initial_pelvis_rot = data.qpos[3:7].copy()
        enforce_fixed_pose(model, data, left_q, right_q, initial_pelvis_pos, initial_pelvis_rot)
        start_time = data.time
        paused = False
        
        while viewer.is_running():
            if not paused:
                enforce_fixed_pose(model, data, left_q, right_q, initial_pelvis_pos, initial_pelvis_rot)
                mujoco.mj_step(model, data)
                enforce_fixed_pose(model, data, left_q, right_q, initial_pelvis_pos, initial_pelvis_rot)
                viewer.sync()
                
                if args.sim_time > 0 and data.time - start_time >= args.sim_time:
                    print(f"\n仿真时间达到 {args.sim_time} 秒，暂停仿真...")
                    paused = True
                    # 验证脚的位置
                    sim_constraint_errors = verify_foot_pose(
                        model, data, effective_pelvis_height, args.foot_offset_z,
                        ankle_x=args.ankle_x, left_ankle_y=left_ankle_y,
                        right_ankle_y=right_ankle_y, target_rot=target_rot
                    )
                    print_hard_constraint_status(sim_constraint_errors)
            else:
                # 暂停时仍然需要同步查看器，否则会卡住
                viewer.sync()


if __name__ == "__main__":
    main()
