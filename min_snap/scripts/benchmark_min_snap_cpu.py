#!/usr/bin/env python3
"""Launch min_snap_node with synthetic inputs and measure CPU by phase."""

from __future__ import annotations

import argparse
import csv
import os
import re
import signal
import subprocess
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


PAGE_SIZE = os.sysconf(os.sysconf_names["SC_PAGE_SIZE"])

JOINT_NAMES = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
    "neck_yaw_joint",
    "neck_pitch_joint",
]


@dataclass
class Sample:
    phase: str
    cpu_pct: float
    rss_mib: float
    threads: int


def run_ros_command(setup: Path, command: str, log_path: Path) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8")
    return subprocess.Popen(
        ["bash", "-lc", f"source {setup}; exec {command}"],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def stop_process(proc: subprocess.Popen | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=3)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            os.killpg(proc.pid, signal.SIGKILL)


def find_min_snap_pid() -> int | None:
    pattern = re.compile(r"(^|/|\s)min_snap_node(\s|$)")
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            cmd = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            )
        except OSError:
            continue
        if pattern.search(cmd):
            return int(entry.name)
    return None


def read_total_ticks() -> int:
    return sum(int(v) for v in Path("/proc/stat").read_text().splitlines()[0].split()[1:])


def read_process(pid: int) -> tuple[int, float, int] | None:
    proc = Path("/proc") / str(pid)
    try:
        stat = (proc / "stat").read_text()
        status = (proc / "status").read_text()
    except OSError:
        return None
    end_comm = stat.rfind(")")
    fields = stat[end_comm + 2 :].split()
    proc_ticks = int(fields[11]) + int(fields[12])
    rss_mib = int(fields[21]) * PAGE_SIZE / 1024.0 / 1024.0
    threads = 0
    for line in status.splitlines():
        if line.startswith("Threads:"):
            threads = int(line.split()[1])
            break
    return proc_ticks, rss_mib, threads


def publish_joint_states(setup: Path, log_dir: Path) -> subprocess.Popen:
    names = "[" + ", ".join(f"'{name}'" for name in JOINT_NAMES) + "]"
    zeros = "[" + ", ".join("0.0" for _ in JOINT_NAMES) + "]"
    msg = f"{{name: {names}, position: {zeros}, velocity: {zeros}}}"
    return run_ros_command(
        setup,
        f"ros2 topic pub -r 100 /joint_states sensor_msgs/msg/JointState \"{msg}\"",
        log_dir / "joint_states_pub.log",
    )


def send_target(setup: Path, log_dir: Path) -> subprocess.Popen:
    msg = (
        "{"
        "left_arm_target_rad: [0.0, 0.2, 0.0, -0.5, 0.0, 0.0, 0.0], "
        "right_arm_target_rad: [0.0, -0.2, 0.0, -0.5, 0.0, 0.0, 0.0], "
        "neck_target_rad: [0.0, 0.1], "
        "expected_duration_s: 0.8, "
        "max_velocity_rad_s: 1.2, "
        "max_acceleration_rad_s2: 8.0"
        "}"
    )
    return run_ros_command(
        setup,
        f"ros2 topic pub --once /min_snap/target min_snap/msg/MinSnapTarget \"{msg}\"",
        log_dir / "target_pub.log",
    )


def summarize(samples: list[Sample]) -> None:
    grouped: dict[str, list[Sample]] = defaultdict(list)
    for sample in samples:
        grouped[sample.phase].append(sample)
    print("phase                  samples  avg_cpu%  max_cpu%  avg_rss_mib  max_rss_mib")
    for phase in ("startup", "idle_static", "planning_executing"):
        rows = grouped.get(phase, [])
        if not rows:
            continue
        avg_cpu = sum(r.cpu_pct for r in rows) / len(rows)
        max_cpu = max(r.cpu_pct for r in rows)
        avg_rss = sum(r.rss_mib for r in rows) / len(rows)
        max_rss = max(r.rss_mib for r in rows)
        print(f"{phase:<22} {len(rows):>7} {avg_cpu:>9.2f} {max_cpu:>9.2f} {avg_rss:>12.1f} {max_rss:>12.1f}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup", type=Path, default=Path("/home/likunwei/humanoid_ws/install/setup.bash"))
    parser.add_argument("--params-file", type=Path, default=Path("/home/likunwei/humanoid_ws/src/min_snap/config/min_snap.yaml"))
    parser.add_argument("--out-dir", type=Path, default=Path("/home/likunwei/humanoid_ws/src/min_snap/logs/cpu_benchmark"))
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--startup-s", type=float, default=5.0)
    parser.add_argument("--idle-s", type=float, default=5.0)
    parser.add_argument("--execute-s", type=float, default=8.0)
    args = parser.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = args.out_dir / stamp
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "min_snap_cpu.csv"

    joint_proc = min_snap_proc = target_proc = None
    samples: list[Sample] = []
    try:
      joint_proc = publish_joint_states(args.setup, out_dir)
      time.sleep(1.0)
      min_snap_proc = run_ros_command(
          args.setup,
          f"ros2 launch min_snap min_snap.launch.py params_file:={args.params_file}",
          out_dir / "min_snap_launch.log",
      )

      deadline = time.monotonic() + 15.0
      pid = None
      while time.monotonic() < deadline:
          pid = find_min_snap_pid()
          if pid is not None:
              break
          time.sleep(0.1)
      if pid is None:
          raise RuntimeError("min_snap_node PID not found; see min_snap_launch.log")

      print(f"min_snap_node pid={pid}")
      prev_total = read_total_ticks()
      proc_state = read_process(pid)
      if proc_state is None:
          raise RuntimeError("min_snap_node exited before sampling")
      prev_proc = proc_state[0]
      start = time.monotonic()
      target_sent = False
      total_s = args.startup_s + args.idle_s + args.execute_s

      with csv_path.open("w", newline="", encoding="utf-8") as f:
          writer = csv.writer(f)
          writer.writerow(["elapsed_s", "phase", "cpu_pct", "rss_mib", "threads"])
          while time.monotonic() - start < total_s:
              elapsed = time.monotonic() - start
              if elapsed < args.startup_s:
                  phase = "startup"
              elif elapsed < args.startup_s + args.idle_s:
                  phase = "idle_static"
              else:
                  phase = "planning_executing"
                  if not target_sent:
                      target_proc = send_target(args.setup, out_dir)
                      target_sent = True

              time.sleep(args.interval)
              total = read_total_ticks()
              proc_state = read_process(pid)
              if proc_state is None:
                  break
              proc_ticks, rss_mib, threads = proc_state
              delta_total = total - prev_total
              delta_proc = proc_ticks - prev_proc
              cpu_pct = 0.0 if delta_total <= 0 else 100.0 * os.cpu_count() * delta_proc / delta_total
              prev_total = total
              prev_proc = proc_ticks
              sample = Sample(phase, cpu_pct, rss_mib, threads)
              samples.append(sample)
              writer.writerow([f"{elapsed:.3f}", phase, f"{cpu_pct:.3f}", f"{rss_mib:.3f}", threads])

      summarize(samples)
      print(f"csv={csv_path}")
    finally:
      stop_process(target_proc)
      stop_process(min_snap_proc)
      stop_process(joint_proc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
