#!/usr/bin/env python3
"""Run online min_snap target-combination tests against the FA RViz command bridge."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import rclpy
from min_snap.msg import MinSnapTarget
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


LEFT_NAMES = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
]
RIGHT_NAMES = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
]
NECK_NAMES = ["neck_yaw_joint", "neck_pitch_joint"]
GROUP_NAMES = {
    "left": LEFT_NAMES,
    "right": RIGHT_NAMES,
    "neck": NECK_NAMES,
}
COMMAND_SLICES = {
    "left": slice(0, 7),
    "right": slice(7, 14),
    "neck": slice(14, 16),
}
GROUP_OFFSETS = {
    "left": [0.16, -0.12, 0.10, -0.16, 0.08, -0.06, 0.07],
    "right": [-0.16, 0.12, -0.10, -0.16, -0.08, 0.06, -0.07],
    "neck": [0.10, -0.07],
}
GROUP_LABELS = {
    "left": "左臂",
    "right": "右臂",
    "neck": "头部",
}
SCENARIO_LABELS = {
    "left_only": "只发左臂目标",
    "right_only": "只发右臂目标",
    "neck_only": "只发头部目标",
    "left_right": "发左臂、右臂目标",
    "left_neck": "发左臂、头部目标",
    "right_neck": "发右臂、头部目标",
    "left_right_neck": "发左臂、右臂、头部目标",
    "all_targets_then_pause": "全目标执行中暂停",
    "all_targets_pause_then_resume_without_new_target": "全目标暂停后恢复，不重放历史轨迹",
}


@dataclass
class ScenarioResult:
    name: str
    groups: list[str]
    passed: bool
    checks: dict[str, bool] = field(default_factory=dict)
    details: dict[str, object] = field(default_factory=dict)


class Harness(Node):
    def __init__(self) -> None:
        super().__init__("min_snap_online_scenario_harness")
        self.target_pub = self.create_publisher(MinSnapTarget, "/min_snap/target", 10)
        self.commands: list[list[float]] = []
        self.latest_joint_state: JointState | None = None
        self.create_subscription(Float64MultiArray, "/upper_position_controller/commands", self.on_command, 50)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 50)
        self.pause_client = self.create_client(Trigger, "/min_snap/pause_trajectory_publish")
        self.resume_client = self.create_client(Trigger, "/min_snap/resume_trajectory_publish")

    def on_command(self, msg: Float64MultiArray) -> None:
        self.commands.append(list(msg.data))

    def on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def wait_for_joint_state(self, timeout_s: float = 3.0) -> JointState:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_joint_state is not None:
                return self.latest_joint_state
        raise RuntimeError("timed out waiting for /joint_states")

    def call_trigger(self, client, timeout_s: float = 3.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"service not available: {client.srv_name}")
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                return future.result()
        raise RuntimeError(f"service call timed out: {client.srv_name}")

    def wait_for_target_subscriber(self, timeout_s: float = 3.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.target_pub.get_subscription_count() > 0:
                return
        raise RuntimeError("/min_snap/target has no subscriber; min_snap_node may not be running")


def spin_for(node: Harness, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def group_values(msg: JointState, names: Iterable[str]) -> list[float]:
    lookup = {name: pos for name, pos in zip(msg.name, msg.position)}
    values = []
    for name in names:
        if name not in lookup:
            raise RuntimeError(f"/joint_states missing required joint: {name}")
        values.append(float(lookup[name]))
    return values


def current_groups(node: Harness) -> dict[str, list[float]]:
    msg = node.wait_for_joint_state()
    return {group: group_values(msg, names) for group, names in GROUP_NAMES.items()}


def add_delta(values: list[float], deltas: list[float]) -> list[float]:
    return [v + d for v, d in zip(values, deltas)]


def scaled_offsets(group: str, scale: float) -> list[float]:
    return [value * scale for value in GROUP_OFFSETS[group]]


def format_values(values: list[float]) -> str:
    return "[" + ", ".join(f"{value:.3f}" for value in values) + "]"


def format_target_summary(targets: dict[str, list[float]]) -> str:
    if not targets:
        return "无"
    parts = []
    for group in ("left", "right", "neck"):
        if group in targets:
            parts.append(f"{GROUP_LABELS[group]}={format_values(targets[group])}")
    return "；".join(parts)


def wait_for_enter(enabled: bool, prompt: str) -> None:
    if enabled:
        input(prompt)


def max_abs_delta(a: list[float], b: list[float]) -> float:
    if len(a) != len(b):
        return math.inf
    return max((abs(x - y) for x, y in zip(a, b)), default=0.0)


def publish_target(node: Harness, targets: dict[str, list[float]], duration_s: float) -> None:
    msg = MinSnapTarget()
    msg.left_arm_target_rad = targets.get("left", [])
    msg.right_arm_target_rad = targets.get("right", [])
    msg.neck_target_rad = targets.get("neck", [])
    msg.expected_duration_s = duration_s
    msg.max_velocity_rad_s = 0.8
    msg.max_acceleration_rad_s2 = 4.0
    node.commands.clear()
    node.target_pub.publish(msg)


def wait_for_commands(node: Harness, min_count: int, timeout_s: float = 5.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if len(node.commands) >= min_count:
            return
    raise RuntimeError(f"timed out waiting for {min_count} command samples")


def ensure_publish_ready(node: Harness) -> None:
    node.wait_for_target_subscriber()
    response = node.call_trigger(node.resume_client)
    if not response.success:
        raise RuntimeError(f"resume service rejected request: {response.message}")


def describe_failure_context(node: Harness, targets: dict[str, list[float]]) -> str:
    current = current_groups(node)
    lines = [
        "当前诊断信息：",
        f"- 已收到 command 样本数: {len(node.commands)}",
        f"- 目标: {format_target_summary(targets)}",
    ]
    for group in ("left", "right", "neck"):
        lines.append(f"- 当前 {GROUP_LABELS[group]} joint_states: {format_values(current[group])}")
    return "\n".join(lines)


def run_motion_scenario(
    node: Harness,
    name: str,
    groups: list[str],
    duration_s: float,
    amplitude_scale: float,
) -> ScenarioResult:
    start = current_groups(node)
    targets = {group: add_delta(start[group], scaled_offsets(group, amplitude_scale)) for group in groups}
    print(f"  测试目标：{format_target_summary(targets)}", flush=True)
    ensure_publish_ready(node)
    publish_target(node, targets, duration_s)
    try:
        wait_for_commands(node, 3)
    except RuntimeError as exc:
        raise RuntimeError(f"{exc}\n{describe_failure_context(node, targets)}") from exc
    first_command = node.commands[0]
    spin_for(node, duration_s + 0.5)
    end = current_groups(node)
    last_command = node.commands[-1] if node.commands else []

    checks: dict[str, bool] = {"command_published": bool(node.commands)}
    details: dict[str, object] = {
        "command_samples": len(node.commands),
        "targets": targets,
    }
    for group in ("left", "right", "neck"):
        command_group = last_command[COMMAND_SLICES[group]] if len(last_command) >= 16 else []
        if group in groups:
            checks[f"{group}_reached_target"] = max_abs_delta(end[group], targets[group]) < 0.03
            checks[f"{group}_command_targets_goal"] = max_abs_delta(command_group, targets[group]) < 0.03
            details[f"{group}_final_error"] = max_abs_delta(end[group], targets[group])
        else:
            start_hold = first_command[COMMAND_SLICES[group]] if len(first_command) >= 16 else []
            checks[f"{group}_command_holds_joint_state"] = max_abs_delta(start_hold, start[group]) < 0.01
            checks[f"{group}_joint_state_holds"] = max_abs_delta(end[group], start[group]) < 0.03
            details[f"{group}_hold_error"] = max_abs_delta(end[group], start[group])

    return ScenarioResult(name=name, groups=groups, passed=all(checks.values()), checks=checks, details=details)


def run_pause_scenario(node: Harness, duration_s: float, amplitude_scale: float) -> ScenarioResult:
    start = current_groups(node)
    targets = {
        group: add_delta(start[group], scaled_offsets(group, amplitude_scale))
        for group in ("left", "right", "neck")
    }
    print(f"  测试目标：{format_target_summary(targets)}", flush=True)
    ensure_publish_ready(node)
    publish_target(node, targets, duration_s)
    try:
        wait_for_commands(node, 3)
    except RuntimeError as exc:
        raise RuntimeError(f"{exc}\n{describe_failure_context(node, targets)}") from exc
    spin_for(node, 0.4)
    before_pause_count = len(node.commands)
    pause_response = node.call_trigger(node.pause_client)
    state_at_pause = current_groups(node)
    spin_for(node, 0.5)
    after_pause_count = len(node.commands)
    state_after_pause = current_groups(node)
    state_after_error = max(
        max_abs_delta(state_after_pause[group], state_at_pause[group]) for group in ("left", "right", "neck")
    )
    checks = {
        "command_published_before_pause": before_pause_count > 0,
        "pause_service_success": bool(pause_response.success),
        "no_commands_after_pause": after_pause_count == before_pause_count,
        "joint_state_stops_after_pause": state_after_error < 0.03,
    }
    details = {
        "targets": targets,
        "pause_message": pause_response.message,
        "commands_before_pause": before_pause_count,
        "commands_after_pause_wait": after_pause_count,
        "post_pause_motion_error": state_after_error,
    }
    return ScenarioResult(
        name="all_targets_then_pause",
        groups=["left", "right", "neck"],
        passed=all(checks.values()),
        checks=checks,
        details=details,
    )


def run_pause_resume_scenario(node: Harness, duration_s: float, amplitude_scale: float) -> ScenarioResult:
    start = current_groups(node)
    targets = {
        group: add_delta(start[group], scaled_offsets(group, amplitude_scale))
        for group in ("left", "right", "neck")
    }
    print(f"  测试目标：{format_target_summary(targets)}", flush=True)
    ensure_publish_ready(node)
    publish_target(node, targets, duration_s)
    try:
        wait_for_commands(node, 3)
    except RuntimeError as exc:
        raise RuntimeError(f"{exc}\n{describe_failure_context(node, targets)}") from exc
    spin_for(node, 0.4)
    pause_response = node.call_trigger(node.pause_client)
    commands_at_pause = len(node.commands)
    state_at_pause = current_groups(node)
    resume_response = node.call_trigger(node.resume_client)
    spin_for(node, 0.7)
    commands_after_resume = len(node.commands)
    state_after_resume = current_groups(node)
    resume_motion_error = max(
        max_abs_delta(state_after_resume[group], state_at_pause[group]) for group in ("left", "right", "neck")
    )
    checks = {
        "pause_service_success": bool(pause_response.success),
        "resume_service_success": bool(resume_response.success),
        "no_old_commands_after_resume": commands_after_resume == commands_at_pause,
        "robot_holds_after_resume_without_new_target": resume_motion_error < 0.03,
    }
    details = {
        "targets": targets,
        "pause_message": pause_response.message,
        "resume_message": resume_response.message,
        "commands_at_pause": commands_at_pause,
        "commands_after_resume_wait": commands_after_resume,
        "post_resume_motion_error": resume_motion_error,
    }
    return ScenarioResult(
        name="all_targets_pause_then_resume_without_new_target",
        groups=["left", "right", "neck"],
        passed=all(checks.values()),
        checks=checks,
        details=details,
    )


def start_min_snap(args) -> subprocess.Popen | None:
    if not args.start_min_snap:
        return None
    cmd = [
        "ros2",
        "run",
        "min_snap",
        "min_snap_node",
        "--ros-args",
        "--params-file",
        str(args.params_file),
        "-p",
        f"publish_hz:={args.publish_hz}",
        "-p",
        "record_run_log:=false",
        "-p",
        "record_tracking:=false",
        "-p",
        f"joint_state_timeout_s:={args.joint_state_timeout}",
    ]
    env = os.environ.copy()
    return subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def set_external_min_snap_timeout(timeout_s: float) -> None:
    result = subprocess.run(
        [
            "ros2",
            "param",
            "set",
            "/min_snap_node",
            "joint_state_timeout_s",
            f"{timeout_s}",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "设置 /min_snap_node joint_state_timeout_s 失败。"
            "如果你是手动启动 min_snap，请确认 /min_snap_node 正在运行。\n"
            + result.stdout
        )
    print(f"已设置 /min_snap_node joint_state_timeout_s={timeout_s:.3f}s", flush=True)


def stop_process(proc: subprocess.Popen | None) -> str:
    if proc is None:
        return ""
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=3)
    return proc.stdout.read() if proc.stdout else ""


def write_report(results: list[ScenarioResult], out_dir: Path) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    json_path = out_dir / f"min_snap_online_scenarios_{stamp}.json"
    md_path = out_dir / f"min_snap_online_scenarios_{stamp}.md"
    payload = {
        "stamp": stamp,
        "results": [
            {
                "name": r.name,
                "groups": r.groups,
                "passed": r.passed,
                "checks": r.checks,
                "details": r.details,
            }
            for r in results
        ],
    }
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# min_snap 在线场景测试", "", f"- 时间戳: `{stamp}`", ""]
    for idx, result in enumerate(results, start=1):
        status = "通过" if result.passed else "失败"
        lines.append(f"## {idx}. {SCENARIO_LABELS.get(result.name, result.name)}: {status}")
        lines.append("")
        lines.append(f"- 目标组: `{','.join(GROUP_LABELS.get(group, group) for group in result.groups)}`")
        lines.append(f"- 命令采样数: `{result.details.get('command_samples', 'n/a')}`")
        for key, value in result.checks.items():
            lines.append(f"- {key}: `{value}`")
        lines.append("")
    md_path.write_text("\n".join(lines), encoding="utf-8")
    return json_path, md_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params-file", type=Path, default=Path("/home/likunwei/humanoid_ws/src/min_snap/config/min_snap.yaml"))
    parser.add_argument("--out-dir", type=Path, default=Path("/home/likunwei/humanoid_ws/src/min_snap/logs/online_scenarios"))
    parser.add_argument("--publish-hz", type=float, default=100.0)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--pause-duration", type=float, default=2.0)
    parser.add_argument("--joint-state-timeout", type=float, default=2.0)
    parser.add_argument("--amplitude-scale", type=float, default=1.0, help="scale the default target offsets")
    parser.add_argument("--manual", action="store_true", help="press Enter before each scenario")
    parser.add_argument("--start-min-snap", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    os.environ.setdefault("ROS_LOG_DIR", "/tmp/min_snap_online_ros_log")
    print("\n========== min_snap 在线测试启动 ==========", flush=True)
    print(f"是否由脚本启动 min_snap: {args.start_min_snap}", flush=True)
    print(f"参数文件: {args.params_file}", flush=True)
    print(f"结果目录: {args.out_dir}", flush=True)
    print(f"动作幅度倍率: {args.amplitude_scale:.2f}", flush=True)
    print(f"joint_states 超时阈值: {args.joint_state_timeout:.3f}s", flush=True)
    print(f"手动逐项测试: {args.manual}", flush=True)
    proc = start_min_snap(args)
    rclpy.init()
    node = Harness()
    results: list[ScenarioResult] = []
    try:
        print("\n========== 阶段 0：环境检查 ==========", flush=True)
        print("等待 fa_command_rviz bridge 发布 /joint_states ...", flush=True)
        spin_for(node, 1.0)
        node.wait_for_joint_state()
        print("已收到 /joint_states。", flush=True)
        if args.start_min_snap:
            print("等待 min_snap 暂停/恢复 service ...", flush=True)
            if not node.pause_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("min_snap pause service did not appear")
            if not node.resume_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("min_snap resume service did not appear")
            print("已发现 min_snap 暂停/恢复 service。", flush=True)
        else:
            print("使用外部 min_snap_node，调整 joint_state_timeout_s 以适配低频 RViz bridge ...", flush=True)
            set_external_min_snap_timeout(args.joint_state_timeout)

        scenarios = [
            ("left_only", ["left"]),
            ("right_only", ["right"]),
            ("neck_only", ["neck"]),
            ("left_right", ["left", "right"]),
            ("left_neck", ["left", "neck"]),
            ("right_neck", ["right", "neck"]),
            ("left_right_neck", ["left", "right", "neck"]),
        ]
        for index, (name, groups) in enumerate(scenarios, start=1):
            label = SCENARIO_LABELS.get(name, name)
            print(f"\n========== 场景 {index}：{label} ==========", flush=True)
            print(f"目标组：{', '.join(GROUP_LABELS[group] for group in groups)}", flush=True)
            wait_for_enter(args.manual, "按 Enter 开始该场景...")
            result = run_motion_scenario(node, name, groups, args.duration, args.amplitude_scale)
            results.append(result)
            print(f"测试结果：{'通过' if result.passed else '失败'}", flush=True)
            print(f"检查项：{json.dumps(result.checks, ensure_ascii=False)}", flush=True)
            spin_for(node, 0.3)
        print(f"\n========== 场景 8：{SCENARIO_LABELS['all_targets_then_pause']} ==========", flush=True)
        print("目标组：左臂、右臂、头部；执行中调用暂停 service。", flush=True)
        wait_for_enter(args.manual, "按 Enter 开始该场景...")
        result = run_pause_scenario(node, args.pause_duration, args.amplitude_scale)
        results.append(result)
        print(f"测试结果：{'通过' if result.passed else '失败'}", flush=True)
        print(f"检查项：{json.dumps(result.checks, ensure_ascii=False)}", flush=True)
        node.call_trigger(node.resume_client)
        spin_for(node, 0.3)
        print(
            f"\n========== 场景 9：{SCENARIO_LABELS['all_targets_pause_then_resume_without_new_target']} ==========",
            flush=True,
        )
        print("目标组：左臂、右臂、头部；先暂停，再恢复，不发送新目标。", flush=True)
        wait_for_enter(args.manual, "按 Enter 开始该场景...")
        result = run_pause_resume_scenario(node, args.pause_duration, args.amplitude_scale)
        results.append(result)
        print(f"测试结果：{'通过' if result.passed else '失败'}", flush=True)
        print(f"检查项：{json.dumps(result.checks, ensure_ascii=False)}", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        min_snap_output = stop_process(proc)

    json_path, md_path = write_report(results, args.out_dir)
    print("\n========== 测试汇总 ==========", flush=True)
    for idx, result in enumerate(results, start=1):
        label = SCENARIO_LABELS.get(result.name, result.name)
        print(f"{idx}. {label}: {'通过' if result.passed else '失败'}")
    print(f"JSON 结果: {json_path}")
    print(f"Markdown 报告: {md_path}")
    if min_snap_output:
        log_path = args.out_dir / "last_min_snap_node_output.log"
        log_path.write_text(min_snap_output, encoding="utf-8")
        print(f"min_snap 节点输出: {log_path}")
    return 0 if all(r.passed for r in results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
