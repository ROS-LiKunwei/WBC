#!/usr/bin/env python3
"""Monitor min_snap_node CPU usage and annotate runtime phases."""

from __future__ import annotations

import argparse
import csv
import os
import re
import signal
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


PAGE_SIZE = os.sysconf(os.sysconf_names["SC_PAGE_SIZE"])


@dataclass
class ProcSample:
    pid: int
    proc_ticks: int
    rss_bytes: int
    threads: int
    command: str


@dataclass
class CpuRow:
    wall_time: float
    phase: str
    pid: int
    cpu_pct: float
    rss_mib: float
    threads: int
    command: str


class PhaseTracker:
    def __init__(self, log_dir: Path, startup_s: float) -> None:
        self.log_dir = log_dir
        self.startup_s = startup_s
        self.process_seen_time: float | None = None
        self.phase = "waiting_for_node"
        self.log_path: Path | None = None
        self.log_pos = 0

    def mark_process_seen(self, now: float) -> None:
        if self.process_seen_time is None:
            self.process_seen_time = now
            self.phase = "startup"

    def update(self, now: float) -> str:
        if self.process_seen_time is None:
            return self.phase
        self._select_latest_log()
        for line in self._read_new_lines():
            self._apply_log_line(line)
        if self.process_seen_time is not None:
            if self.phase == "startup" and now - self.process_seen_time >= self.startup_s:
                self.phase = "idle_static"
        return self.phase

    def _select_latest_log(self) -> None:
        if not self.log_dir.exists():
            return
        logs = sorted(self.log_dir.glob("*.log"), key=lambda p: p.stat().st_mtime)
        if not logs:
            return
        latest = logs[-1]
        if self.process_seen_time is not None and latest.stat().st_mtime < self.process_seen_time - 1.0:
            return
        if latest != self.log_path:
            self.log_path = latest
            self.log_pos = 0

    def _read_new_lines(self) -> Iterable[str]:
        if self.log_path is None:
            return []
        try:
            with self.log_path.open("r", encoding="utf-8", errors="replace") as f:
                f.seek(self.log_pos)
                lines = f.readlines()
                self.log_pos = f.tell()
                return lines
        except OSError:
            return []

    def _apply_log_line(self, line: str) -> None:
        if "node_ready" in line:
            self.phase = "startup"
        elif "plan_timing" in line or "Planned min-snap trajectory" in line:
            self.phase = "planning_or_executing"
        elif "Replanned min-snap trajectory" in line:
            self.phase = "planning_or_executing"
        elif "publish_sample" in line or "publish seq=" in line:
            self.phase = "executing"
        elif "trajectory_completed" in line or "trajectories_deactivated" in line:
            self.phase = "idle_static"
        elif "pause_publish_service success=true" in line:
            self.phase = "idle_static"


def read_total_cpu_ticks() -> int:
    with Path("/proc/stat").open("r", encoding="utf-8") as f:
        first = f.readline().split()
    return sum(int(v) for v in first[1:])


def read_proc_sample(pid: int) -> ProcSample | None:
    proc_dir = Path("/proc") / str(pid)
    try:
        stat = (proc_dir / "stat").read_text(encoding="utf-8")
        status = (proc_dir / "status").read_text(encoding="utf-8")
        cmdline = (proc_dir / "cmdline").read_bytes().replace(b"\0", b" ").decode(
            "utf-8", errors="replace"
        ).strip()
    except OSError:
        return None

    end_comm = stat.rfind(")")
    fields = stat[end_comm + 2 :].split()
    utime = int(fields[11])
    stime = int(fields[12])
    rss_pages = int(fields[21])
    threads = 0
    for line in status.splitlines():
        if line.startswith("Threads:"):
            threads = int(line.split()[1])
            break
    return ProcSample(
        pid=pid,
        proc_ticks=utime + stime,
        rss_bytes=rss_pages * PAGE_SIZE,
        threads=threads,
        command=cmdline,
    )


def find_min_snap_pids(pattern: re.Pattern[str]) -> list[int]:
    pids: list[int] = []
    self_pid = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == self_pid:
            continue
        try:
            cmd = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            )
        except OSError:
            continue
        if not cmd:
            continue
        if "monitor_min_snap_cpu.py" in cmd:
            continue
        if pattern.search(cmd):
            pids.append(pid)
    return sorted(pids)


def open_csv(path: Path | None):
    if path is None:
        return None, None
    path.parent.mkdir(parents=True, exist_ok=True)
    f = path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(f)
    writer.writerow(["wall_time", "phase", "pid", "cpu_pct", "rss_mib", "threads", "command"])
    f.flush()
    return f, writer


def print_row(row: CpuRow) -> None:
    stamp = time.strftime("%H:%M:%S", time.localtime(row.wall_time))
    print(
        f"{stamp} phase={row.phase:<22} pid={row.pid:<7} "
        f"cpu={row.cpu_pct:6.2f}% rss={row.rss_mib:7.1f}MiB "
        f"threads={row.threads:<3} cmd={row.command[:90]}",
        flush=True,
    )


def summarize(rows: list[CpuRow]) -> None:
    if not rows:
        print("No min_snap_node CPU samples collected.")
        return
    grouped: dict[str, list[CpuRow]] = defaultdict(list)
    for row in rows:
        grouped[row.phase].append(row)
    print("\nSummary by phase:")
    print("phase                  samples  avg_cpu%  max_cpu%  avg_rss_mib  max_rss_mib")
    for phase, phase_rows in grouped.items():
        avg_cpu = sum(r.cpu_pct for r in phase_rows) / len(phase_rows)
        max_cpu = max(r.cpu_pct for r in phase_rows)
        avg_rss = sum(r.rss_mib for r in phase_rows) / len(phase_rows)
        max_rss = max(r.rss_mib for r in phase_rows)
        print(
            f"{phase:<22} {len(phase_rows):>7} {avg_cpu:>9.2f} "
            f"{max_cpu:>9.2f} {avg_rss:>12.1f} {max_rss:>12.1f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interval", type=float, default=0.2, help="sample interval in seconds")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds; 0 means until Ctrl-C")
    parser.add_argument("--startup-s", type=float, default=5.0, help="phase window after first PID appears")
    parser.add_argument("--log-dir", type=Path, default=Path("/home/likunwei/humanoid_ws/src/min_snap/Log"))
    parser.add_argument("--csv", type=Path, default=None, help="optional CSV output path")
    parser.add_argument(
        "--process-regex",
        default=r"(^|/|\s)min_snap_node(\s|$)",
        help="regex matched against /proc/*/cmdline",
    )
    args = parser.parse_args()

    if args.interval <= 0:
        parser.error("--interval must be > 0")

    pattern = re.compile(args.process_regex)
    phase_tracker = PhaseTracker(args.log_dir, args.startup_s)
    csv_file, writer = open_csv(args.csv)
    rows: list[CpuRow] = []
    stop = False

    def on_signal(signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    print(
        "Monitoring min_snap_node CPU. Start min_snap_node if it is not running; press Ctrl-C to stop.",
        flush=True,
    )
    start = time.monotonic()
    previous: dict[int, tuple[float, int, int]] = {}

    while not stop:
        now_mono = time.monotonic()
        if args.duration > 0 and now_mono - start >= args.duration:
            break

        pids = find_min_snap_pids(pattern)
        wall_now = time.time()
        total_ticks = read_total_cpu_ticks()
        if pids:
            phase_tracker.mark_process_seen(wall_now)
        phase = phase_tracker.update(wall_now)

        if not pids:
            print(
                f"{time.strftime('%H:%M:%S', time.localtime(wall_now))} "
                f"phase={phase:<22} waiting for min_snap_node",
                flush=True,
            )
            time.sleep(args.interval)
            continue

        for pid in pids:
            sample = read_proc_sample(pid)
            if sample is None:
                continue
            prev = previous.get(pid)
            previous[pid] = (wall_now, total_ticks, sample.proc_ticks)
            if prev is None:
                continue
            prev_wall, prev_total, prev_proc = prev
            delta_proc = sample.proc_ticks - prev_proc
            delta_total = total_ticks - prev_total
            cpu_pct = 0.0
            if delta_total > 0:
                cpu_pct = 100.0 * os.cpu_count() * delta_proc / delta_total
            row = CpuRow(
                wall_time=wall_now,
                phase=phase,
                pid=pid,
                cpu_pct=cpu_pct,
                rss_mib=sample.rss_bytes / 1024.0 / 1024.0,
                threads=sample.threads,
                command=sample.command,
            )
            rows.append(row)
            print_row(row)
            if writer is not None:
                writer.writerow([
                    f"{row.wall_time:.6f}",
                    row.phase,
                    row.pid,
                    f"{row.cpu_pct:.3f}",
                    f"{row.rss_mib:.3f}",
                    row.threads,
                    row.command,
                ])
                csv_file.flush()
        time.sleep(args.interval)

    if csv_file is not None:
        csv_file.close()
    summarize(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
