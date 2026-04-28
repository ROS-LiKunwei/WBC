#!/usr/bin/env python3
"""
Leg IK MuJoCo Verification Script

Input: pelvis height from ground
Output: leg joint angles

Usage:
    python3 leg_ik_mujoco_verify.py --pelvis_height 0.65
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

from leg_ik_py import solve_both_legs_ik, LEFT_LEG_JOINTS, RIGHT_LEG_JOINTS

MUJOCO_MODEL_PATH = os.path.join(
    os.path.dirname(__file__), "..", "sysmo_description", "mjcf",
    "fa_robot_combined_body_collision_modified.xml"
)

LEG_JOINT_NAMES = LEFT_LEG_JOINTS + RIGHT_LEG_JOINTS

FOOT_SOLE_OFFSET_Z = 0.05 # Dist between ankle joint and sole of foot


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


    target_pos = np.array([fk_zero_pos[0], fk_zero_pos[1], -pelvis_height + foot_offset_z])



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


def get_body_ids(model):
    body_name_to_id = {}
    for i in range(model.nbody):
        body_name_to_id[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)] = i
    return body_name_to_id


def find_foot_min_z(model, data, body_ids):
    left_foot_id = body_ids.get("left_ankle_roll_link", -1)
    right_foot_id = body_ids.get("right_ankle_roll_link", -1)
    min_z = 0.0
    for fid in [left_foot_id, right_foot_id]:
        if fid >= 0:
            for geom_id in range(model.ngeom):
                if model.geom_bodyid[geom_id] == fid:
                    geom_pos = data.geom_xpos[geom_id]
                    geom_size = model.geom_size[geom_id]
                    geom_min_z = geom_pos[2] - geom_size[2]
                    min_z = min(min_z, geom_min_z)
    return min_z


def verify_foot_pose(model, data, pelvis_height, foot_offset_z=0.0):
    body_ids = get_body_ids(model)
    pelvis_id = body_ids.get("pelvis", -1)
    left_foot_id = body_ids.get("left_ankle_roll_link", -1)
    right_foot_id = body_ids.get("right_ankle_roll_link", -1)

    pelvis_pos = data.xpos[pelvis_id].copy() if pelvis_id >= 0 else np.array([0, 0, 1])

    print("\n===== MuJoCo FK验证 =====")
    print(f"骨盆位置: ({pelvis_pos[0]:.4f}, {pelvis_pos[1]:.4f}, {pelvis_pos[2]:.4f})")

    if left_foot_id >= 0:
        left_pos = data.xpos[left_foot_id]
        left_rot = data.xmat[left_foot_id].reshape(3, 3)
        left_sole_z = left_pos[2] - FOOT_SOLE_OFFSET_Z
        print(f"左脚位置: ({left_pos[0]:.4f}, {left_pos[1]:.4f}, {left_pos[2]:.4f})")
        print(f"  左脚底z: {left_sole_z:.4f} m (目标: 0.0000 m)")
        print(f"  骨盆->左脚踝距离: {pelvis_pos[2] - left_pos[2]:.4f} m")
        R_err = left_rot.T @ np.eye(3)
        angle_err = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
        print(f"  左脚朝向误差: {np.degrees(angle_err):.2f} deg")

    if right_foot_id >= 0:
        right_pos = data.xpos[right_foot_id]
        right_rot = data.xmat[right_foot_id].reshape(3, 3)
        right_sole_z = right_pos[2] - FOOT_SOLE_OFFSET_Z
        print(f"右脚位置: ({right_pos[0]:.4f}, {right_pos[1]:.4f}, {right_pos[2]:.4f})")
        print(f"  右脚底z: {right_sole_z:.4f} m (目标: 0.0000 m)")
        print(f"  骨盆->右脚踝距离: {pelvis_pos[2] - right_pos[2]:.4f} m")
        R_err = right_rot.T @ np.eye(3)
        angle_err = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
        print(f"  右脚朝向误差: {np.degrees(angle_err):.2f} deg")


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

    left_q = None
    right_q = None

    if args.left_q is not None:
        left_q = np.array([float(x) for x in args.left_q.split(",")])
        print(f"使用命令行指定的左腿关节角: {left_q.tolist()}")

    if args.right_q is not None:
        right_q = np.array([float(x) for x in args.right_q.split(",")])
        print(f"使用命令行指定的右腿关节角: {right_q.tolist()}")

    if left_q is None or right_q is None:
        print(f"\n调用Python IK求解器 (骨盆高度={args.pelvis_height}m)...")
        ik_left_q, ik_right_q = solve_both_legs_ik(
            model, data, args.pelvis_height,
            foot_offset_z=args.foot_offset_z + FOOT_SOLE_OFFSET_Z,
            max_iters=500, eps=1e-3
        )
        if ik_left_q is not None and left_q is None:
            left_q = ik_left_q
            print(f"Python IK结果: 左腿={[f'{v:.6f}' for v in left_q]}")
        if ik_right_q is not None and right_q is None:
            right_q = ik_right_q
            print(f"Python IK结果: 右腿={[f'{v:.6f}' for v in right_q]}")

        if left_q is None:
            print("左腿IK求解失败，使用默认站立姿态")
            left_q = np.array([0.0, 0.0, 0.0, 0.5, -0.3, 0.0])
        if right_q is None:
            print("右腿IK求解失败，使用默认站立姿态")
            right_q = np.array([0.0, 0.0, 0.0, 0.5, -0.3, 0.0])

    set_leg_joints(model, data, left_q, right_q)

    data.qpos[0] = 0.0
    data.qpos[1] = 0.0
    data.qpos[2] = args.pelvis_height
    data.qpos[3] = 1.0
    data.qpos[4] = 0.0
    data.qpos[5] = 0.0
    data.qpos[6] = 0.0

    mujoco.mj_forward(model, data)

    body_ids = get_body_ids(model)
    left_foot_id = body_ids.get("left_ankle_roll_link", -1)
    right_foot_id = body_ids.get("right_ankle_roll_link", -1)

    min_foot_z = find_foot_min_z(model, data, body_ids)

    if min_foot_z < 0:
        adjustment = -min_foot_z + 0.001
        data.qpos[2] += adjustment
        print(f"调整骨盆高度: {args.pelvis_height:.3f}m -> {data.qpos[2]:.3f}m (脚底最低点: {min_foot_z:.3f}m)")
        mujoco.mj_forward(model, data)

    set_leg_controls(model, data, left_q, right_q)

    verify_foot_pose(model, data, args.pelvis_height, args.foot_offset_z)

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
            # 先设置初始关节位置
            set_leg_joints(model, data, left_q, right_q)
            # 然后在仿真过程中只设置控制输入，并保持骨盆位置和姿态
            while data.time - start_time < args.sim_time:
                set_leg_controls(model, data, left_q, right_q)
                # 保持骨盆位置和姿态稳定
                data.qpos[:3] = initial_pelvis_pos
                data.qpos[3:7] = initial_pelvis_rot
                mujoco.mj_step(model, data)
            print(f"仿真完成，当前时间: {data.time:.2f}秒")
            # 验证脚的位置
            verify_foot_pose(model, data, args.pelvis_height, args.foot_offset_z)
        
        print("\nHeadless模式，模型状态已设置")
        return

    print("\n启动MuJoCo交互式查看器...")
    print("提示: 可以用鼠标旋转/缩放视图，按Esc退出")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        set_leg_controls(model, data, left_q, right_q)
        # 保存初始骨盆位置和姿态
        initial_pelvis_pos = data.qpos[:3].copy()
        initial_pelvis_rot = data.qpos[3:7].copy()
        start_time = data.time
        paused = False
        
        while viewer.is_running():
            if not paused:
                set_leg_controls(model, data, left_q, right_q)
                # 保持骨盆位置和姿态稳定
                data.qpos[:3] = initial_pelvis_pos
                data.qpos[3:7] = initial_pelvis_rot
                mujoco.mj_step(model, data)
                viewer.sync()
                
                if args.sim_time > 0 and data.time - start_time >= args.sim_time:
                    print(f"\n仿真时间达到 {args.sim_time} 秒，暂停仿真...")
                    paused = True
                    # 验证脚的位置
                    verify_foot_pose(model, data, args.pelvis_height, args.foot_offset_z)
            else:
                # 暂停时仍然需要同步查看器，否则会卡住
                viewer.sync()


if __name__ == "__main__":
    main()
