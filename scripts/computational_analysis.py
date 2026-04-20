#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import signal
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def percentile(values: list[float], p: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * (p / 100.0)
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return ordered[low]
    weight = rank - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def mean_or_nan(values: list[float]) -> float:
    if not values:
        return math.nan
    return sum(values) / len(values)


def std_or_nan(values: list[float]) -> float:
    if len(values) < 2:
        return math.nan
    avg = mean_or_nan(values)
    variance = sum((value - avg) ** 2 for value in values) / (len(values) - 1)
    return math.sqrt(variance)


def to_mb(value: float) -> float:
    return value / (1024.0 * 1024.0)


def now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


@dataclass
class ProcessSpec:
    label: str
    pattern: str
    regex: re.Pattern[str]


@dataclass
class TopicSpec:
    label: str
    name: str


@dataclass
class ProcessState:
    pid: int
    process: psutil.Process
    primed: bool = False
    last_ctx_total: int | None = None
    last_read_bytes: int | None = None
    last_write_bytes: int | None = None


@dataclass
class TopicTracker:
    spec: TopicSpec
    message_type: str = ""
    subscribed: bool = False
    message_count: int = 0
    first_message_monotonic: float | None = None
    last_message_monotonic: float | None = None
    last_age_ms: float | None = None
    inter_arrival_s: list[float] = field(default_factory=list)
    age_ms: list[float] = field(default_factory=list)
    max_gap_ms: float = 0.0
    window_messages: int = 0
    window_age_sum_ms: float = 0.0
    window_age_count: int = 0
    window_started_monotonic: float = 0.0

    def bind(self, message_type: str, now_monotonic: float) -> None:
        self.message_type = message_type
        self.subscribed = True
        self.window_started_monotonic = now_monotonic

    def on_message(self, node_clock: Callable[[], int], msg: Any, now_monotonic: float) -> None:
        if self.first_message_monotonic is None:
            self.first_message_monotonic = now_monotonic
            self.window_started_monotonic = now_monotonic
        if self.last_message_monotonic is not None:
            dt_s = now_monotonic - self.last_message_monotonic
            self.inter_arrival_s.append(dt_s)
            self.max_gap_ms = max(self.max_gap_ms, dt_s * 1000.0)
        self.last_message_monotonic = now_monotonic
        self.message_count += 1
        self.window_messages += 1

        age = extract_header_age_ms(node_clock, msg)
        if age is not None:
            self.last_age_ms = age
            self.age_ms.append(age)
            self.window_age_sum_ms += age
            self.window_age_count += 1

    def snapshot(self, now_monotonic: float) -> dict[str, Any]:
        window_elapsed = max(now_monotonic - self.window_started_monotonic, 1e-9)
        return {
            "label": self.spec.label,
            "topic_name": self.spec.name,
            "topic_type": self.message_type,
            "subscribed": self.subscribed,
            "message_count_total": self.message_count,
            "window_messages": self.window_messages,
            "window_rate_hz": self.window_messages / window_elapsed if self.window_messages else 0.0,
            "window_mean_age_ms": (
                self.window_age_sum_ms / self.window_age_count if self.window_age_count else math.nan
            ),
            "last_age_ms": self.last_age_ms if self.last_age_ms is not None else math.nan,
            "last_gap_ms": (
                (now_monotonic - self.last_message_monotonic) * 1000.0
                if self.last_message_monotonic is not None
                else math.nan
            ),
            "max_gap_ms": self.max_gap_ms if self.message_count > 1 else math.nan,
        }

    def reset_window(self, now_monotonic: float) -> None:
        self.window_messages = 0
        self.window_age_sum_ms = 0.0
        self.window_age_count = 0
        self.window_started_monotonic = now_monotonic

    def summary(self, session_duration_s: float) -> dict[str, Any]:
        mean_dt = mean_or_nan(self.inter_arrival_s)
        return {
            "label": self.spec.label,
            "topic_name": self.spec.name,
            "topic_type": self.message_type,
            "subscribed": self.subscribed,
            "messages": self.message_count,
            "effective_rate_hz": self.message_count / session_duration_s if session_duration_s > 0.0 else math.nan,
            "mean_inter_arrival_ms": mean_dt * 1000.0 if not math.isnan(mean_dt) else math.nan,
            "std_inter_arrival_ms": (
                std_or_nan(self.inter_arrival_s) * 1000.0 if self.inter_arrival_s else math.nan
            ),
            "p95_inter_arrival_ms": percentile([value * 1000.0 for value in self.inter_arrival_s], 95.0),
            "max_gap_ms": self.max_gap_ms if self.message_count > 1 else math.nan,
            "mean_age_ms": mean_or_nan(self.age_ms),
            "std_age_ms": std_or_nan(self.age_ms),
            "p95_age_ms": percentile(self.age_ms, 95.0),
            "max_age_ms": max(self.age_ms) if self.age_ms else math.nan,
        }


def extract_header_age_ms(clock_now_ns: Callable[[], int], msg: Any) -> float | None:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    if not hasattr(stamp, "sec") or not hasattr(stamp, "nanosec"):
        return None
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return (clock_now_ns() - stamp_ns) / 1_000_000.0


class TopicMonitorNode(Node):
    def __init__(self, topic_specs: list[TopicSpec]) -> None:
        super().__init__("computational_analysis_monitor")
        self.trackers = {spec.label: TopicTracker(spec=spec) for spec in topic_specs}
        self._topic_subscriptions: list[Any] = []

    def try_discover_topics(self) -> None:
        current_topics = dict(self.get_topic_names_and_types())
        now_monotonic = time.monotonic()
        for tracker in self.trackers.values():
            if tracker.subscribed:
                continue
            if tracker.spec.name not in current_topics:
                continue
            topic_types = current_topics[tracker.spec.name]
            if not topic_types:
                continue
            topic_type = topic_types[0]
            try:
                message_class = get_message(topic_type)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                self.get_logger().warning(
                    f"Could not resolve type '{topic_type}' for topic '{tracker.spec.name}': {exc}"
                )
                continue

            def callback(msg: Any, tracked: TopicTracker = tracker) -> None:
                tracked.on_message(lambda: self.get_clock().now().nanoseconds, msg, time.monotonic())

            subscription = self.create_subscription(
                message_class,
                tracker.spec.name,
                callback,
                qos_profile_sensor_data,
            )
            self._topic_subscriptions.append(subscription)
            tracker.bind(topic_type, now_monotonic)
            self.get_logger().info(
                f"Subscribed to {tracker.spec.name} as {topic_type} [{tracker.spec.label}]"
            )


class PerformanceMonitor:
    def __init__(
        self,
        output_dir: Path,
        sample_period_s: float,
        summary_period_s: float,
        process_specs: list[ProcessSpec],
        topic_specs: list[TopicSpec],
        include_self_process: bool,
    ) -> None:
        self.output_dir = output_dir
        self.sample_period_s = sample_period_s
        self.summary_period_s = summary_period_s
        self.process_specs = process_specs
        self.topic_specs = topic_specs
        self.include_self_process = include_self_process
        self.cpu_count = max(psutil.cpu_count(logical=True) or 1, 1)
        self.process_states: dict[int, ProcessState] = {}
        self.process_series: dict[str, dict[str, list[float]]] = {}
        self.system_series = {"cpu_percent": [], "memory_percent": [], "memory_used_mb": []}
        self._stop_requested = False
        self.started_at_iso = now_iso()

        self.node = TopicMonitorNode(topic_specs)
        self._prepare_output()

    def _prepare_output(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.process_samples_path = self.output_dir / "process_samples.csv"
        self.system_samples_path = self.output_dir / "system_samples.csv"
        self.topic_samples_path = self.output_dir / "topic_samples.csv"
        self.process_summary_path = self.output_dir / "process_summary.csv"
        self.topic_summary_path = self.output_dir / "topic_summary.csv"
        self.system_summary_path = self.output_dir / "system_summary.csv"
        self.session_summary_path = self.output_dir / "session_summary.json"

        self.process_samples_file = self.process_samples_path.open("w", newline="", encoding="utf-8")
        self.system_samples_file = self.system_samples_path.open("w", newline="", encoding="utf-8")
        self.topic_samples_file = self.topic_samples_path.open("w", newline="", encoding="utf-8")

        self.process_samples_writer = csv.DictWriter(
            self.process_samples_file,
            fieldnames=[
                "wall_time",
                "elapsed_s",
                "label",
                "matched_pids",
                "pid_count",
                "cpu_percent_system",
                "cpu_percent_one_core",
                "rss_mb",
                "threads",
                "ctx_switches_per_s",
                "read_mb_per_s",
                "write_mb_per_s",
            ],
        )
        self.system_samples_writer = csv.DictWriter(
            self.system_samples_file,
            fieldnames=["wall_time", "elapsed_s", "cpu_percent", "memory_percent", "memory_used_mb"],
        )
        self.topic_samples_writer = csv.DictWriter(
            self.topic_samples_file,
            fieldnames=[
                "wall_time",
                "elapsed_s",
                "label",
                "topic_name",
                "topic_type",
                "subscribed",
                "message_count_total",
                "window_messages",
                "window_rate_hz",
                "window_mean_age_ms",
                "last_age_ms",
                "last_gap_ms",
                "max_gap_ms",
            ],
        )
        self.process_samples_writer.writeheader()
        self.system_samples_writer.writeheader()
        self.topic_samples_writer.writeheader()

    def request_stop(self, *_: Any) -> None:
        self._stop_requested = True

    def run(self, duration_s: float) -> int:
        signal.signal(signal.SIGINT, self.request_stop)
        signal.signal(signal.SIGTERM, self.request_stop)

        psutil.cpu_percent(None)
        start_monotonic = time.monotonic()
        last_sample_monotonic = start_monotonic
        last_summary_monotonic = start_monotonic
        last_discovery_monotonic = 0.0
        sample_index = 0

        self.node.get_logger().info(f"Writing computational analysis to {self.output_dir}")

        while rclpy.ok() and not self._stop_requested:
            now_monotonic = time.monotonic()
            if now_monotonic - last_discovery_monotonic >= 1.0:
                self.node.try_discover_topics()
                last_discovery_monotonic = now_monotonic

            rclpy.spin_once(self.node, timeout_sec=0.1)

            if duration_s > 0.0 and (now_monotonic - start_monotonic) >= duration_s:
                break

            if (now_monotonic - last_sample_monotonic) < self.sample_period_s:
                continue

            sample_dt = now_monotonic - last_sample_monotonic
            elapsed_s = now_monotonic - start_monotonic
            wall_time = now_iso()
            self._sample_system(wall_time, elapsed_s)
            self._sample_processes(wall_time, elapsed_s, sample_dt)
            self._sample_topics(wall_time, elapsed_s, now_monotonic)
            sample_index += 1

            if (now_monotonic - last_summary_monotonic) >= self.summary_period_s:
                self._print_brief_summary(elapsed_s)
                last_summary_monotonic = now_monotonic

            last_sample_monotonic = now_monotonic

        session_duration_s = time.monotonic() - start_monotonic
        self._write_summaries(session_duration_s, sample_index)
        self._close_files()
        self.node.destroy_node()
        rclpy.shutdown()
        print(f"Saved computational analysis under {self.output_dir}")
        return 0

    def _sample_system(self, wall_time: str, elapsed_s: float) -> None:
        vm = psutil.virtual_memory()
        row = {
            "wall_time": wall_time,
            "elapsed_s": f"{elapsed_s:.3f}",
            "cpu_percent": f"{psutil.cpu_percent(None):.3f}",
            "memory_percent": f"{vm.percent:.3f}",
            "memory_used_mb": f"{to_mb(vm.used):.3f}",
        }
        self.system_samples_writer.writerow(row)
        self.system_series["cpu_percent"].append(float(row["cpu_percent"]))
        self.system_series["memory_percent"].append(float(row["memory_percent"]))
        self.system_series["memory_used_mb"].append(float(row["memory_used_mb"]))

    def _sample_processes(self, wall_time: str, elapsed_s: float, sample_dt: float) -> None:
        matches = {spec.label: [] for spec in self.process_specs}
        self_pid = os.getpid()

        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                pid = proc.info["pid"]
                if self.include_self_process and pid == self_pid:
                    continue
                name = proc.info["name"] or ""
                cmdline = " ".join(proc.info["cmdline"] or [])
                haystack = f"{name} {cmdline}"
                for spec in self.process_specs:
                    if spec.regex.search(haystack):
                        matches[spec.label].append(pid)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        if self.include_self_process:
            matches.setdefault("monitor_script", []).append(self_pid)

        for label, pids in matches.items():
            total_cpu_one_core = 0.0
            total_cpu_system = 0.0
            total_rss_mb = 0.0
            total_threads = 0
            total_ctx_rate = 0.0
            total_read_rate = 0.0
            total_write_rate = 0.0
            visible_pids: list[int] = []

            for pid in sorted(set(pids)):
                state = self.process_states.get(pid)
                if state is None:
                    try:
                        state = ProcessState(pid=pid, process=psutil.Process(pid))
                        self.process_states[pid] = state
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue

                try:
                    proc = state.process
                    visible_pids.append(pid)
                    if not state.primed:
                        proc.cpu_percent(None)
                        ctx = proc.num_ctx_switches()
                        state.last_ctx_total = ctx.voluntary + ctx.involuntary
                        try:
                            io = proc.io_counters()
                            state.last_read_bytes = io.read_bytes
                            state.last_write_bytes = io.write_bytes
                        except (psutil.AccessDenied, AttributeError):
                            state.last_read_bytes = None
                            state.last_write_bytes = None
                        state.primed = True
                        continue

                    cpu_one_core = proc.cpu_percent(None)
                    cpu_system = cpu_one_core / self.cpu_count
                    mem = proc.memory_info()
                    total_cpu_one_core += cpu_one_core
                    total_cpu_system += cpu_system
                    total_rss_mb += to_mb(mem.rss)
                    total_threads += proc.num_threads()

                    ctx = proc.num_ctx_switches()
                    ctx_total = ctx.voluntary + ctx.involuntary
                    if state.last_ctx_total is not None:
                        total_ctx_rate += max(ctx_total - state.last_ctx_total, 0) / max(sample_dt, 1e-9)
                    state.last_ctx_total = ctx_total

                    try:
                        io = proc.io_counters()
                    except (psutil.AccessDenied, AttributeError):
                        io = None
                    if io is not None:
                        if state.last_read_bytes is not None:
                            total_read_rate += max(io.read_bytes - state.last_read_bytes, 0) / max(sample_dt, 1e-9)
                        if state.last_write_bytes is not None:
                            total_write_rate += max(io.write_bytes - state.last_write_bytes, 0) / max(sample_dt, 1e-9)
                        state.last_read_bytes = io.read_bytes
                        state.last_write_bytes = io.write_bytes
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue

            row = {
                "wall_time": wall_time,
                "elapsed_s": f"{elapsed_s:.3f}",
                "label": label,
                "matched_pids": ";".join(str(pid) for pid in visible_pids),
                "pid_count": len(visible_pids),
                "cpu_percent_system": f"{total_cpu_system:.3f}",
                "cpu_percent_one_core": f"{total_cpu_one_core:.3f}",
                "rss_mb": f"{total_rss_mb:.3f}",
                "threads": total_threads,
                "ctx_switches_per_s": f"{total_ctx_rate:.3f}",
                "read_mb_per_s": f"{to_mb(total_read_rate):.6f}",
                "write_mb_per_s": f"{to_mb(total_write_rate):.6f}",
            }
            self.process_samples_writer.writerow(row)

            series = self.process_series.setdefault(
                label,
                {
                    "cpu_percent_system": [],
                    "cpu_percent_one_core": [],
                    "rss_mb": [],
                    "threads": [],
                    "pid_count": [],
                    "ctx_switches_per_s": [],
                    "read_mb_per_s": [],
                    "write_mb_per_s": [],
                },
            )
            series["cpu_percent_system"].append(float(row["cpu_percent_system"]))
            series["cpu_percent_one_core"].append(float(row["cpu_percent_one_core"]))
            series["rss_mb"].append(float(row["rss_mb"]))
            series["threads"].append(float(row["threads"]))
            series["pid_count"].append(float(row["pid_count"]))
            series["ctx_switches_per_s"].append(float(row["ctx_switches_per_s"]))
            series["read_mb_per_s"].append(float(row["read_mb_per_s"]))
            series["write_mb_per_s"].append(float(row["write_mb_per_s"]))

    def _sample_topics(self, wall_time: str, elapsed_s: float, now_monotonic: float) -> None:
        for tracker in self.node.trackers.values():
            row = {
                "wall_time": wall_time,
                "elapsed_s": f"{elapsed_s:.3f}",
                **tracker.snapshot(now_monotonic),
            }
            self.topic_samples_writer.writerow(row)
            tracker.reset_window(now_monotonic)

    def _print_brief_summary(self, elapsed_s: float) -> None:
        process_bits: list[str] = []
        for label in ("filter", "fastlio", "amcl", "monitor_script"):
            series = self.process_series.get(label)
            if not series:
                continue
            if max(series["pid_count"], default=0.0) <= 0.0:
                continue
            cpu_avg = mean_or_nan(series["cpu_percent_system"])
            rss_avg = mean_or_nan(series["rss_mb"])
            if math.isnan(cpu_avg) and math.isnan(rss_avg):
                continue
            process_bits.append(f"{label}: cpu={cpu_avg:.2f}% rss={rss_avg:.1f}MB")

        topic_bits: list[str] = []
        for label in ("filter_odom", "lidar_odom", "wheel_odom", "amcl_pose", "imu"):
            tracker = self.node.trackers.get(label)
            if tracker is None or tracker.message_count == 0:
                continue
            summary = tracker.summary(max(elapsed_s, 1e-9))
            rate = summary["effective_rate_hz"]
            age = summary["mean_age_ms"]
            if math.isnan(age):
                topic_bits.append(f"{label}: {rate:.2f}Hz")
            else:
                topic_bits.append(f"{label}: {rate:.2f}Hz age={age:.1f}ms")

        summary_text = " | ".join(process_bits + topic_bits)
        if summary_text:
            print(f"[{elapsed_s:7.1f}s] {summary_text}")

    def _write_summaries(self, session_duration_s: float, sample_count: int) -> None:
        with self.process_summary_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "label",
                    "samples",
                    "mean_pid_count",
                    "max_pid_count",
                    "mean_cpu_percent_system",
                    "p95_cpu_percent_system",
                    "max_cpu_percent_system",
                    "mean_cpu_percent_one_core",
                    "mean_rss_mb",
                    "p95_rss_mb",
                    "max_rss_mb",
                    "mean_threads",
                    "max_threads",
                    "mean_ctx_switches_per_s",
                    "mean_read_mb_per_s",
                    "mean_write_mb_per_s",
                ],
            )
            writer.writeheader()
            for label, series in sorted(self.process_series.items()):
                writer.writerow(
                    {
                        "label": label,
                        "samples": len(series["cpu_percent_system"]),
                        "mean_pid_count": f"{mean_or_nan(series['pid_count']):.3f}",
                        "max_pid_count": f"{max(series['pid_count']) if series['pid_count'] else math.nan:.3f}",
                        "mean_cpu_percent_system": f"{mean_or_nan(series['cpu_percent_system']):.3f}",
                        "p95_cpu_percent_system": f"{percentile(series['cpu_percent_system'], 95.0):.3f}",
                        "max_cpu_percent_system": f"{max(series['cpu_percent_system']) if series['cpu_percent_system'] else math.nan:.3f}",
                        "mean_cpu_percent_one_core": f"{mean_or_nan(series['cpu_percent_one_core']):.3f}",
                        "mean_rss_mb": f"{mean_or_nan(series['rss_mb']):.3f}",
                        "p95_rss_mb": f"{percentile(series['rss_mb'], 95.0):.3f}",
                        "max_rss_mb": f"{max(series['rss_mb']) if series['rss_mb'] else math.nan:.3f}",
                        "mean_threads": f"{mean_or_nan(series['threads']):.3f}",
                        "max_threads": f"{max(series['threads']) if series['threads'] else math.nan:.3f}",
                        "mean_ctx_switches_per_s": f"{mean_or_nan(series['ctx_switches_per_s']):.3f}",
                        "mean_read_mb_per_s": f"{mean_or_nan(series['read_mb_per_s']):.6f}",
                        "mean_write_mb_per_s": f"{mean_or_nan(series['write_mb_per_s']):.6f}",
                    }
                )

        with self.topic_summary_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "label",
                    "topic_name",
                    "topic_type",
                    "subscribed",
                    "messages",
                    "effective_rate_hz",
                    "mean_inter_arrival_ms",
                    "std_inter_arrival_ms",
                    "p95_inter_arrival_ms",
                    "max_gap_ms",
                    "mean_age_ms",
                    "std_age_ms",
                    "p95_age_ms",
                    "max_age_ms",
                ],
            )
            writer.writeheader()
            for tracker in self.node.trackers.values():
                writer.writerow(tracker.summary(session_duration_s))

        with self.system_summary_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "samples",
                    "mean_cpu_percent",
                    "p95_cpu_percent",
                    "max_cpu_percent",
                    "mean_memory_percent",
                    "p95_memory_percent",
                    "max_memory_percent",
                    "mean_memory_used_mb",
                    "max_memory_used_mb",
                ],
            )
            writer.writeheader()
            writer.writerow(
                {
                    "samples": len(self.system_series["cpu_percent"]),
                    "mean_cpu_percent": f"{mean_or_nan(self.system_series['cpu_percent']):.3f}",
                    "p95_cpu_percent": f"{percentile(self.system_series['cpu_percent'], 95.0):.3f}",
                    "max_cpu_percent": f"{max(self.system_series['cpu_percent']) if self.system_series['cpu_percent'] else math.nan:.3f}",
                    "mean_memory_percent": f"{mean_or_nan(self.system_series['memory_percent']):.3f}",
                    "p95_memory_percent": f"{percentile(self.system_series['memory_percent'], 95.0):.3f}",
                    "max_memory_percent": f"{max(self.system_series['memory_percent']) if self.system_series['memory_percent'] else math.nan:.3f}",
                    "mean_memory_used_mb": f"{mean_or_nan(self.system_series['memory_used_mb']):.3f}",
                    "max_memory_used_mb": f"{max(self.system_series['memory_used_mb']) if self.system_series['memory_used_mb'] else math.nan:.3f}",
                }
            )

        session_summary = {
            "started_at": self.started_at_iso,
            "finished_at": now_iso(),
            "session_duration_s": round(session_duration_s, 3),
            "sample_count": sample_count,
            "sample_period_s": self.sample_period_s,
            "summary_period_s": self.summary_period_s,
            "cpu_count": self.cpu_count,
            "process_specs": [{"label": spec.label, "pattern": spec.pattern} for spec in self.process_specs],
            "topic_specs": [{"label": spec.label, "name": spec.name} for spec in self.topic_specs],
            "output_dir": str(self.output_dir),
            "outputs": {
                "process_samples": str(self.process_samples_path),
                "system_samples": str(self.system_samples_path),
                "topic_samples": str(self.topic_samples_path),
                "process_summary": str(self.process_summary_path),
                "topic_summary": str(self.topic_summary_path),
                "system_summary": str(self.system_summary_path),
            },
        }
        self.session_summary_path.write_text(json.dumps(session_summary, indent=2), encoding="utf-8")

    def _close_files(self) -> None:
        self.process_samples_file.close()
        self.system_samples_file.close()
        self.topic_samples_file.close()


def parse_process_spec(raw: str) -> ProcessSpec:
    if "=" not in raw:
        raise argparse.ArgumentTypeError("Process specs must be in the form label=regex")
    label, pattern = raw.split("=", 1)
    label = label.strip()
    pattern = pattern.strip()
    if not label or not pattern:
        raise argparse.ArgumentTypeError("Process specs must provide both label and regex")
    return ProcessSpec(label=label, pattern=pattern, regex=re.compile(pattern, re.IGNORECASE))


def parse_topic_spec(raw: str) -> TopicSpec:
    if "=" not in raw:
        raise argparse.ArgumentTypeError("Topic specs must be in the form label=/topic_name")
    label, topic_name = raw.split("=", 1)
    label = label.strip()
    topic_name = topic_name.strip()
    if not label or not topic_name:
        raise argparse.ArgumentTypeError("Topic specs must provide both label and topic name")
    return TopicSpec(label=label, name=topic_name)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Low-overhead online computational monitor for the adaptive odom filter pipeline.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--duration", type=float, default=0.0, help="Run duration in seconds. Use 0 to run until Ctrl-C.")
    parser.add_argument("--sample-period", type=float, default=1.0, help="Sampling period for process and system metrics.")
    parser.add_argument("--summary-period", type=float, default=5.0, help="Console summary period.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for CSV and JSON outputs. Defaults to a timestamped folder under ./analysis_out.",
    )
    parser.add_argument(
        "--process",
        action="append",
        type=parse_process_spec,
        default=[],
        help="Additional process monitor rule in the form label=regex.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        type=parse_topic_spec,
        default=[],
        help="Additional topic monitor rule in the form label=/topic_name.",
    )
    parser.add_argument("--no-default-processes", action="store_true", help="Disable the default process rules.")
    parser.add_argument("--no-default-topics", action="store_true", help="Disable the default topic rules.")
    parser.add_argument(
        "--monitor-self",
        action="store_true",
        help="Add the monitoring script itself as an explicit process entry for smoke tests.",
    )
    return parser


def default_process_specs() -> list[ProcessSpec]:
    return [
        parse_process_spec("filter=adaptive_odom_filter|EKFAdaptiveFilter|EKFRobustAdaptiveFilter"),
        parse_process_spec("fastlio=fastlio|fast_lio|ekf_loam"),
        parse_process_spec("amcl=(^|/|\\s)amcl($|\\s)"),
    ]


def default_topic_specs() -> list[TopicSpec]:
    return [
        parse_topic_spec("wheel_odom=/odom_with_cov"),
        parse_topic_spec("imu=/imu/data"),
        parse_topic_spec("lidar_odom=/ekf_loam/laser_odom_with_cov"),
        parse_topic_spec("filter_odom=/ekf_loam/filter_odom_to_init"),
        parse_topic_spec("amcl_pose=/amcl_pose"),
    ]


def deduplicate_specs(specs: list[Any], key: Callable[[Any], str]) -> list[Any]:
    seen: set[str] = set()
    unique: list[Any] = []
    for spec in specs:
        value = key(spec)
        if value in seen:
            continue
        seen.add(value)
        unique.append(spec)
    return unique


def main(argv: list[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    process_specs = [] if args.no_default_processes else default_process_specs()
    topic_specs = [] if args.no_default_topics else default_topic_specs()
    process_specs.extend(args.process)
    topic_specs.extend(args.topic)
    process_specs = deduplicate_specs(process_specs, lambda spec: spec.label)
    topic_specs = deduplicate_specs(topic_specs, lambda spec: spec.label)

    if args.output_dir is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path.cwd() / "analysis_out" / f"computational_analysis_{timestamp}"
    else:
        output_dir = args.output_dir.expanduser().resolve()

    rclpy.init(args=argv)
    monitor = PerformanceMonitor(
        output_dir=output_dir,
        sample_period_s=max(args.sample_period, 0.1),
        summary_period_s=max(args.summary_period, 0.5),
        process_specs=process_specs,
        topic_specs=topic_specs,
        include_self_process=args.monitor_self,
    )
    return monitor.run(max(args.duration, 0.0))


if __name__ == "__main__":
    sys.exit(main())
