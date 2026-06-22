#!/usr/bin/env python3
"""Plot min_snap tracking CSV files."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
from typing import Iterable, Sequence

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

GROUPS = {
    "left": list(range(0, 7)),
    "right": list(range(7, 14)),
    "neck": list(range(14, 16)),
}


def _finite_limits(values: Iterable[float]) -> tuple[float, float]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return -1.0, 1.0
    lo = min(finite)
    hi = max(finite)
    if abs(hi - lo) < 1e-12:
        return lo - 1.0, hi + 1.0
    margin = 0.08 * (hi - lo)
    return lo - margin, hi + margin


def _column(data, name: str):
    if name not in data.columns:
        print(f"[WARN] Missing CSV column: {name}")
        return None
    return data[name].to_numpy(dtype=float)


def plot_tracking(csv_path: Path, groups: Sequence[str], output_dir: Path | None = None) -> list[Path]:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd

    data = pd.read_csv(csv_path)
    if "time_s" not in data.columns or data.empty:
        raise RuntimeError(f"{csv_path} does not contain time_s data")
    times = (data["time_s"] - data["time_s"].iloc[0]).to_numpy(dtype=float)
    output_dir = csv_path.parent if output_dir is None else output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs: list[Path] = []

    for group in groups:
        fig, axes = plt.subplots(4, 1, figsize=(16, 14), sharex=True)
        plotted = False
        for joint_idx in GROUPS[group]:
            joint = JOINT_NAMES[joint_idx]
            label = joint.replace("_joint", "")
            actual_pos = _column(data, f"joint_states.position.{joint}")
            desired_pos = _column(data, f"desired.position.{joint}")
            desired_vel = _column(data, f"desired.velocity.{joint}")
            desired_acc = _column(data, f"desired.acceleration.{joint}")
            desired_jerk = _column(data, f"desired.jerk.{joint}")
            if actual_pos is not None:
                axes[0].plot(times, actual_pos, "--", linewidth=1.0, label=f"{label} actual")
                plotted = True
            if desired_pos is not None:
                axes[0].plot(times, desired_pos, linewidth=1.2, label=f"{label} desired")
                plotted = True
            if desired_vel is not None:
                axes[1].plot(times, desired_vel, linewidth=1.0, label=label)
            if desired_acc is not None:
                axes[2].plot(times, desired_acc, linewidth=1.0, label=label)
            if desired_jerk is not None:
                axes[3].plot(times, desired_jerk, linewidth=1.0, label=label)

        if not plotted:
            print(f"[WARN] No plottable columns for group: {group}")
        axes[0].set_ylabel("position (rad)")
        axes[1].set_ylabel("desired velocity (rad/s)")
        axes[2].set_ylabel("desired acceleration (rad/s^2)")
        axes[3].set_ylabel("desired jerk (rad/s^3)")
        axes[3].set_xlabel("time (s)")
        for axis in axes:
            axis.grid(True, alpha=0.3)
            if axis.lines:
                axis.set_ylim(*_finite_limits(value for line in axis.lines for value in line.get_ydata()))
                axis.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), fontsize=7)
        fig.suptitle(f"FA min-snap {group} tracking")
        fig.tight_layout()
        output = output_dir / f"{csv_path.stem}_{group}_tracking.png"
        fig.savefig(output, dpi=160)
        plt.close(fig)
        outputs.append(output)
    return outputs


def parse_args():
    parser = argparse.ArgumentParser(description="Plot min_snap tracking CSV.")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--groups", nargs="+", choices=list(GROUPS), default=["left", "right", "neck"])
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    for output in plot_tracking(args.csv, args.groups, args.output_dir):
        print(f"Saved plot: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
