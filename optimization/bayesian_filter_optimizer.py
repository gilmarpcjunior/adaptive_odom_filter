#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


DEFAULT_SOURCE_SETUP = (
    "/opt/ros/jazzy/setup.bash",
    "/home/diogo/ros2_ws/install/setup.bash",
)


@dataclass(frozen=True)
class TopicConfig:
    fastlio: str = "/Odometry"
    wheel: str = "/odom"
    imu: str = "/imu/data"
    gt: str = "/scout_mini/Odom"


@dataclass
class OdomSeries:
    topic: str
    stamp_ns: np.ndarray
    xyz: np.ndarray
    yaw: np.ndarray

    @property
    def size(self) -> int:
        return int(self.xyz.shape[0])


@dataclass
class BagReference:
    name: str
    path: Path
    fastlio: OdomSeries
    wheel: OdomSeries
    gt: OdomSeries
    fastlio_metrics: dict[str, float]
    wheel_metrics: dict[str, float]


@dataclass(frozen=True)
class Candidate:
    lidarG: float
    wheelGVx: float
    wheelGVy: float
    wheelGWz: float
    alpha_lidar: float
    wheelVxScale: float
    enableImu: bool
    enableWheel: bool = True
    wheel_type_func: int = 1
    wheelOffset: float = 1.0e-5
    gamma_vx: float = 0.05
    gamma_omegaz: float = 0.01
    delta_vx: float = 0.0001
    delta_omegaz: float = 0.00001

    def as_ros_params(self, output_topic: str, topics: TopicConfig) -> dict[str, Any]:
        return {
            "enableFilter": True,
            "enableVisual": False,
            "enableImu": self.enableImu,
            "enableWheel": self.enableWheel,
            "enableLidar": True,
            "use_sim_time": True,
            "filterFreq": "l",
            "freq": 20.0,
            "imu_topic": topics.imu,
            "wheel_odom_topic": topics.wheel,
            "lidar_odom_topic": topics.fastlio,
            "filter_odom_topic": output_topic,
            "lidarG": self.lidarG,
            "wheelGVx": self.wheelGVx,
            "wheelGVy": self.wheelGVy,
            "wheelGWz": self.wheelGWz,
            "wheelOffset": self.wheelOffset,
            "wheel_type_func": self.wheel_type_func,
            "gamma_vx": self.gamma_vx,
            "gamma_omegaz": self.gamma_omegaz,
            "delta_vx": self.delta_vx,
            "delta_omegaz": self.delta_omegaz,
            "alpha_lidar": self.alpha_lidar,
            "wheelVxScale": self.wheelVxScale,
        }

    def csv_row(self) -> dict[str, Any]:
        return {
            "lidarG": self.lidarG,
            "wheelGVx": self.wheelGVx,
            "wheelGVy": self.wheelGVy,
            "wheelGWz": self.wheelGWz,
            "alpha_lidar": self.alpha_lidar,
            "wheelVxScale": self.wheelVxScale,
            "enableImu": int(self.enableImu),
            "enableWheel": int(self.enableWheel),
            "wheel_type_func": self.wheel_type_func,
            "wheelOffset": self.wheelOffset,
            "gamma_vx": self.gamma_vx,
            "gamma_omegaz": self.gamma_omegaz,
            "delta_vx": self.delta_vx,
            "delta_omegaz": self.delta_omegaz,
        }


@dataclass(frozen=True)
class ParamSpec:
    name: str
    low: float
    high: float
    log10: bool = False

    def from_unit(self, value: float) -> float:
        value = min(max(value, 0.0), 1.0)
        scaled = self.low + value * (self.high - self.low)
        return 10.0**scaled if self.log10 else scaled

    def to_unit(self, value: float) -> float:
        scaled = math.log10(value) if self.log10 else value
        if self.high == self.low:
            return 0.0
        return min(max((scaled - self.low) / (self.high - self.low), 0.0), 1.0)


FIXED_SEARCH_SPACE = (
    ParamSpec("lidarG", -1.0, 1.2, True),
    ParamSpec("wheelGVx", -2.2, 0.3, True),
    ParamSpec("wheelGVy", -2.0, 1.0, True),
    ParamSpec("wheelGWz", -1.0, 2.1, True),
    ParamSpec("alpha_lidar", 0.90, 0.9995, False),
    ParamSpec("wheelVxScale", 0.85, 1.15, False),
)

ADAPTIVE_SEARCH_SPACE = (
    ParamSpec("lidarG", -1.3, 0.8, True),
    ParamSpec("alpha_lidar", 0.94, 0.9995, False),
    ParamSpec("wheelVxScale", 0.90, 1.12, False),
    ParamSpec("wheelOffset", -4.0, -0.3, True),
    ParamSpec("gamma_vx", -3.0, 1.0, True),
    ParamSpec("gamma_omegaz", -3.0, 2.0, True),
    ParamSpec("delta_vx", -4.0, 0.3, True),
    ParamSpec("delta_omegaz", -4.0, 1.7, True),
)

LIDAR_ONLY_SEARCH_SPACE = (
    ParamSpec("lidarG", -1.3, 0.8, True),
    ParamSpec("alpha_lidar", 0.94, 0.9995, False),
)

CURRENT_CANDIDATE_IMU = Candidate(
    lidarG=2.0,
    wheelGVx=0.1,
    wheelGVy=1.0,
    wheelGWz=40.0,
    alpha_lidar=0.99,
    wheelVxScale=1.0,
    enableImu=True,
    wheel_type_func=1,
)

CURRENT_CANDIDATE_NO_IMU = Candidate(
    lidarG=2.0,
    wheelGVx=0.1,
    wheelGVy=1.0,
    wheelGWz=40.0,
    alpha_lidar=0.99,
    wheelVxScale=1.0,
    enableImu=False,
    wheel_type_func=1,
)

ANCHOR_CANDIDATES = (
    CURRENT_CANDIDATE_NO_IMU,
    CURRENT_CANDIDATE_IMU,
    Candidate(
        lidarG=0.5,
        wheelGVx=2.0,
        wheelGVy=10.0,
        wheelGWz=100.0,
        alpha_lidar=0.99,
        wheelVxScale=1.0,
        enableImu=False,
        wheel_type_func=1,
    ),
    Candidate(
        lidarG=1.0,
        wheelGVx=0.5,
        wheelGVy=10.0,
        wheelGWz=100.0,
        alpha_lidar=0.99,
        wheelVxScale=1.0,
        enableImu=False,
        wheel_type_func=1,
    ),
    Candidate(
        lidarG=0.75,
        wheelGVx=1.0,
        wheelGVy=5.0,
        wheelGWz=80.0,
        alpha_lidar=0.995,
        wheelVxScale=1.0,
        enableImu=False,
        wheel_type_func=1,
    ),
    Candidate(
        lidarG=0.75,
        wheelGVx=1.0,
        wheelGVy=5.0,
        wheelGWz=80.0,
        alpha_lidar=0.995,
        wheelVxScale=1.0,
        enableImu=False,
        wheel_type_func=0,
        wheelOffset=0.01,
        gamma_vx=1.0,
        gamma_omegaz=50.0,
        delta_vx=0.02,
        delta_omegaz=0.5,
    ),
    Candidate(
        lidarG=0.5,
        wheelGVx=1.0,
        wheelGVy=5.0,
        wheelGWz=80.0,
        alpha_lidar=0.998,
        wheelVxScale=1.0,
        enableImu=False,
        wheel_type_func=0,
        wheelOffset=0.05,
        gamma_vx=3.0,
        gamma_omegaz=80.0,
        delta_vx=0.05,
        delta_omegaz=2.0,
    ),
    Candidate(
        lidarG=1.0,
        wheelGVx=1.0,
        wheelGVy=5.0,
        wheelGWz=80.0,
        alpha_lidar=0.99,
        wheelVxScale=1.0,
        enableImu=False,
        wheel_type_func=0,
        wheelOffset=0.005,
        gamma_vx=0.3,
        gamma_omegaz=20.0,
        delta_vx=0.01,
        delta_omegaz=0.1,
    ),
    Candidate(
        lidarG=0.5,
        wheelGVx=0.1,
        wheelGVy=1.0,
        wheelGWz=40.0,
        alpha_lidar=0.998,
        wheelVxScale=1.0,
        enableImu=False,
        enableWheel=False,
        wheel_type_func=0,
    ),
    Candidate(
        lidarG=0.75,
        wheelGVx=0.1,
        wheelGVy=1.0,
        wheelGWz=40.0,
        alpha_lidar=0.995,
        wheelVxScale=1.0,
        enableImu=False,
        enableWheel=False,
        wheel_type_func=0,
    ),
    Candidate(
        lidarG=0.25,
        wheelGVx=0.1,
        wheelGVy=1.0,
        wheelGWz=40.0,
        alpha_lidar=0.9985,
        wheelVxScale=1.0,
        enableImu=False,
        enableWheel=False,
        wheel_type_func=0,
    ),
)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(a - b), np.cos(a - b))


def path_length_xy(xyz: np.ndarray) -> float:
    if xyz.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)))


def step_stats(xyz: np.ndarray, jump_threshold_m: float) -> dict[str, float]:
    if xyz.shape[0] < 2:
        return {
            "max_step_xy_m": 0.0,
            "p95_step_xy_m": 0.0,
            "median_step_xy_m": 0.0,
            "jump_count": 0.0,
            "jump_excess_m": 0.0,
        }
    steps = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)
    excess = np.maximum(steps - jump_threshold_m, 0.0)
    return {
        "max_step_xy_m": float(np.max(steps)),
        "p95_step_xy_m": float(np.percentile(steps, 95.0)),
        "median_step_xy_m": float(np.median(steps)),
        "jump_count": float(np.count_nonzero(steps > jump_threshold_m)),
        "jump_excess_m": float(np.sum(excess)),
    }


def cumulative_s_xy(xyz: np.ndarray) -> np.ndarray:
    if xyz.shape[0] == 0:
        return np.zeros(0)
    if xyz.shape[0] == 1:
        return np.zeros(1)
    steps = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(steps)))


def unique_progress(s: np.ndarray, xyz: np.ndarray, yaw: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if s.shape[0] <= 1:
        return s, xyz, yaw
    keep = np.concatenate(([True], np.diff(s) > 1.0e-9))
    return s[keep], xyz[keep], yaw[keep]


def resample_by_normalized_path(series: OdomSeries, samples: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if series.size == 0:
        return np.zeros((samples, 3)), np.zeros(samples), np.linspace(0.0, 1.0, samples)
    s = cumulative_s_xy(series.xyz)
    s, xyz, yaw = unique_progress(s, series.xyz, series.yaw)
    if s.shape[0] == 1 or s[-1] <= 1.0e-9:
        out_xyz = np.repeat(xyz[:1, :], samples, axis=0)
        out_yaw = np.repeat(yaw[:1], samples)
        return out_xyz, out_yaw, np.linspace(0.0, 1.0, samples)

    progress = s / s[-1]
    target = np.linspace(0.0, 1.0, samples)
    unwrapped_yaw = np.unwrap(yaw)
    out_xyz = np.column_stack(
        [
            np.interp(target, progress, xyz[:, 0]),
            np.interp(target, progress, xyz[:, 1]),
            np.interp(target, progress, xyz[:, 2]),
        ]
    )
    out_yaw = np.interp(target, progress, unwrapped_yaw)
    return out_xyz, out_yaw, target


def compute_path_errors(
    candidate: OdomSeries,
    gt: OdomSeries,
    samples: int,
    jump_threshold_m: float,
) -> tuple[dict[str, float], dict[str, np.ndarray]]:
    cand_xyz, cand_yaw, progress = resample_by_normalized_path(candidate, samples)
    gt_xyz, gt_yaw, _ = resample_by_normalized_path(gt, samples)
    diff = cand_xyz - gt_xyz
    yaw_error = angle_diff(cand_yaw, gt_yaw)

    xy_error = np.linalg.norm(diff[:, :2], axis=1)
    xyz_error = np.linalg.norm(diff, axis=1)
    gt_len = path_length_xy(gt.xyz)
    cand_len = path_length_xy(candidate.xyz)
    length_ratio = cand_len / gt_len if gt_len > 1.0e-9 else math.nan

    metrics = {
        "messages": float(candidate.size),
        "path_length_xy_m": cand_len,
        "gt_path_length_xy_m": gt_len,
        "path_length_ratio": length_ratio,
        "path_length_abs_error_m": abs(cand_len - gt_len),
        "x_rmse_m": float(math.sqrt(np.mean(diff[:, 0] ** 2))),
        "y_rmse_m": float(math.sqrt(np.mean(diff[:, 1] ** 2))),
        "z_rmse_m": float(math.sqrt(np.mean(diff[:, 2] ** 2))),
        "xy_rmse_m": float(math.sqrt(np.mean(xy_error**2))),
        "xyz_rmse_m": float(math.sqrt(np.mean(xyz_error**2))),
        "xy_mae_m": float(np.mean(np.abs(xy_error))),
        "xy_p95_m": float(np.percentile(xy_error, 95.0)),
        "yaw_rmse_rad": float(math.sqrt(np.mean(yaw_error**2))),
        "yaw_mae_rad": float(np.mean(np.abs(yaw_error))),
    }
    metrics.update(step_stats(candidate.xyz, jump_threshold_m))
    arrays = {
        "progress": progress,
        "x_error": diff[:, 0],
        "y_error": diff[:, 1],
        "z_error": diff[:, 2],
        "xy_error": xy_error,
        "yaw_error": yaw_error,
    }
    return metrics, arrays


def bag_score(filter_metrics: dict[str, float], fastlio_metrics: dict[str, float]) -> float:
    fastlio_xy = max(fastlio_metrics["xy_rmse_m"], 1.0e-6)
    rmse_ratio = filter_metrics["xy_rmse_m"] / fastlio_xy
    length_error = abs(filter_metrics["path_length_ratio"] - 1.0)
    yaw_term = min(filter_metrics["yaw_rmse_rad"], math.pi) / math.pi
    jump_term = 3.0 * filter_metrics["jump_excess_m"] + 0.2 * filter_metrics["jump_count"]
    worse_than_fastlio = max(rmse_ratio - 1.0, 0.0)
    missing_term = 20.0 if filter_metrics["messages"] < 10.0 else 0.0
    return float(
        rmse_ratio
        + 0.75 * length_error
        + 0.20 * yaw_term
        + jump_term
        + 2.0 * worse_than_fastlio
        + missing_term
    )


def aggregate_scores(per_bag_scores: list[float]) -> float:
    if not per_bag_scores:
        return math.inf
    values = np.asarray(per_bag_scores, dtype=float)
    return float(np.mean(values) + 0.25 * np.std(values))


def safe_ros_domain_id(base: int, offset: int) -> int:
    """Keep experiment domains away from ROS_DOMAIN_ID 0 and below the DDS cap."""
    return 1 + ((base + offset - 1) % 220)


def read_storage_id(bag_path: Path) -> str:
    metadata = bag_path / "metadata.yaml"
    if not metadata.exists():
        return ""
    for line in metadata.read_text(encoding="utf-8", errors="ignore").splitlines():
        stripped = line.strip()
        if stripped.startswith("storage_identifier:"):
            return stripped.split(":", 1)[1].strip()
    return ""


def read_odom_topics(bag_path: Path, topics: Iterable[str]) -> dict[str, OdomSeries]:
    wanted = set(topics)
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=read_storage_id(bag_path))
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {topic: get_message(topic_types[topic]) for topic in wanted if topic in topic_types}

    rows: dict[str, list[tuple[int, float, float, float, float]]] = {topic: [] for topic in wanted}
    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        if topic not in message_types:
            continue
        msg = deserialize_message(data, message_types[topic])
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp <= 0:
            stamp = int(bag_stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rows[topic].append((int(stamp), float(p.x), float(p.y), float(p.z), quat_to_yaw(q.x, q.y, q.z, q.w)))

    output: dict[str, OdomSeries] = {}
    for topic in wanted:
        data_rows = rows.get(topic, [])
        if not data_rows:
            output[topic] = OdomSeries(topic=topic, stamp_ns=np.zeros(0, dtype=np.int64), xyz=np.zeros((0, 3)), yaw=np.zeros(0))
            continue
        arr = np.asarray(data_rows, dtype=float)
        output[topic] = OdomSeries(
            topic=topic,
            stamp_ns=arr[:, 0].astype(np.int64),
            xyz=arr[:, 1:4],
            yaw=arr[:, 4],
        )
    return output


def load_reference_bags(
    bag_paths: list[Path],
    topics: TopicConfig,
    samples: int,
    jump_threshold_m: float,
) -> list[BagReference]:
    references: list[BagReference] = []
    for bag_path in bag_paths:
        print(f"[load] {bag_path}", flush=True)
        series = read_odom_topics(bag_path, [topics.fastlio, topics.wheel, topics.gt])
        fastlio_metrics, _ = compute_path_errors(series[topics.fastlio], series[topics.gt], samples, jump_threshold_m)
        wheel_metrics, _ = compute_path_errors(series[topics.wheel], series[topics.gt], samples, jump_threshold_m)
        references.append(
            BagReference(
                name=bag_path.name,
                path=bag_path,
                fastlio=series[topics.fastlio],
                wheel=series[topics.wheel],
                gt=series[topics.gt],
                fastlio_metrics=fastlio_metrics,
                wheel_metrics=wheel_metrics,
            )
        )
    return references


def search_space_for_wheel_mode(wheel_mode: str) -> tuple[ParamSpec, ...]:
    if wheel_mode == "adaptive":
        return ADAPTIVE_SEARCH_SPACE
    if wheel_mode == "fixed":
        return FIXED_SEARCH_SPACE
    if wheel_mode == "none":
        return LIDAR_ONLY_SEARCH_SPACE
    raise ValueError(f"unsupported wheel mode: {wheel_mode}")


def unit_to_candidate(
    vector: np.ndarray,
    tune_imu: bool,
    wheel_mode: str,
    specs: tuple[ParamSpec, ...],
) -> Candidate:
    values = {spec.name: spec.from_unit(float(vector[index])) for index, spec in enumerate(specs)}
    enable_imu = bool(vector[len(specs)] >= 0.5) if tune_imu else False
    wheel_type_func = 0 if wheel_mode in ("adaptive", "none") else 1
    base = CURRENT_CANDIDATE_NO_IMU
    return Candidate(
        lidarG=values.get("lidarG", base.lidarG),
        wheelGVx=values.get("wheelGVx", base.wheelGVx),
        wheelGVy=values.get("wheelGVy", base.wheelGVy),
        wheelGWz=values.get("wheelGWz", base.wheelGWz),
        alpha_lidar=values.get("alpha_lidar", base.alpha_lidar),
        wheelVxScale=values.get("wheelVxScale", base.wheelVxScale),
        enableImu=enable_imu,
        enableWheel=(wheel_mode != "none"),
        wheel_type_func=wheel_type_func,
        wheelOffset=values.get("wheelOffset", base.wheelOffset),
        gamma_vx=values.get("gamma_vx", base.gamma_vx),
        gamma_omegaz=values.get("gamma_omegaz", base.gamma_omegaz),
        delta_vx=values.get("delta_vx", base.delta_vx),
        delta_omegaz=values.get("delta_omegaz", base.delta_omegaz),
    )


def candidate_to_unit(candidate: Candidate, tune_imu: bool, specs: tuple[ParamSpec, ...]) -> np.ndarray:
    values = [spec.to_unit(getattr(candidate, spec.name)) for spec in specs]
    if tune_imu:
        values.append(1.0 if candidate.enableImu else 0.0)
    return np.asarray(values, dtype=float)


def candidate_key(candidate: Candidate) -> str:
    rounded = {
        "lidarG": round(candidate.lidarG, 7),
        "wheelGVx": round(candidate.wheelGVx, 7),
        "wheelGVy": round(candidate.wheelGVy, 7),
        "wheelGWz": round(candidate.wheelGWz, 7),
        "alpha_lidar": round(candidate.alpha_lidar, 7),
        "wheelVxScale": round(candidate.wheelVxScale, 7),
        "enableImu": candidate.enableImu,
        "enableWheel": candidate.enableWheel,
        "wheel_type_func": candidate.wheel_type_func,
        "wheelOffset": round(candidate.wheelOffset, 7),
        "gamma_vx": round(candidate.gamma_vx, 7),
        "gamma_omegaz": round(candidate.gamma_omegaz, 7),
        "delta_vx": round(candidate.delta_vx, 7),
        "delta_omegaz": round(candidate.delta_omegaz, 7),
    }
    return json.dumps(rounded, sort_keys=True)


def initial_vectors(
    rng: random.Random,
    init_evals: int,
    tune_imu: bool,
    wheel_mode: str,
    specs: tuple[ParamSpec, ...],
) -> list[np.ndarray]:
    vectors = []
    seen_keys: set[str] = set()
    for candidate in ANCHOR_CANDIDATES:
        if not tune_imu and candidate.enableImu:
            continue
        if wheel_mode == "adaptive" and candidate.wheel_type_func != 0:
            continue
        if wheel_mode == "fixed" and candidate.wheel_type_func != 1:
            continue
        if wheel_mode == "none" and candidate.enableWheel:
            continue
        if not tune_imu and candidate.enableImu:
            candidate = Candidate(**{**candidate.__dict__, "enableImu": False})
        key = candidate_key(candidate)
        if key in seen_keys:
            continue
        vectors.append(candidate_to_unit(candidate, tune_imu, specs))
        seen_keys.add(key)
    while len(vectors) < init_evals:
        values = [rng.random() for _ in specs]
        if tune_imu:
            values.append(float(rng.choice([0, 1])))
        vectors.append(np.asarray(values, dtype=float))
    return vectors[:init_evals]


def matern52_kernel(x1: np.ndarray, x2: np.ndarray, length_scale: float = 0.35) -> np.ndarray:
    diff = (x1[:, None, :] - x2[None, :, :]) / length_scale
    r = np.sqrt(np.sum(diff * diff, axis=2))
    sqrt5_r = math.sqrt(5.0) * r
    return (1.0 + sqrt5_r + (5.0 / 3.0) * r * r) * np.exp(-sqrt5_r)


def normal_pdf(x: np.ndarray) -> np.ndarray:
    return np.exp(-0.5 * x * x) / math.sqrt(2.0 * math.pi)


def normal_cdf(x: np.ndarray) -> np.ndarray:
    return 0.5 * (1.0 + np.vectorize(math.erf)(x / math.sqrt(2.0)))


def expected_improvement(x_train: np.ndarray, y_train: np.ndarray, x_pool: np.ndarray, xi: float = 0.01) -> np.ndarray:
    if x_train.shape[0] < 2:
        return np.ones(x_pool.shape[0])

    y_mean = float(np.mean(y_train))
    y_std = float(np.std(y_train))
    if y_std < 1.0e-9:
        y_std = 1.0
    y_scaled = (y_train - y_mean) / y_std

    noise = 1.0e-6
    k_train = matern52_kernel(x_train, x_train) + noise * np.eye(x_train.shape[0])
    try:
        chol = np.linalg.cholesky(k_train)
    except np.linalg.LinAlgError:
        return np.ones(x_pool.shape[0]) * 1.0e-9

    alpha = np.linalg.solve(chol.T, np.linalg.solve(chol, y_scaled))
    k_cross = matern52_kernel(x_train, x_pool)
    mu_scaled = k_cross.T @ alpha
    v = np.linalg.solve(chol, k_cross)
    var_scaled = np.maximum(1.0 - np.sum(v * v, axis=0), 1.0e-12)

    mu = mu_scaled * y_std + y_mean
    sigma = np.sqrt(var_scaled) * y_std
    best = float(np.min(y_train))
    improvement = best - mu - xi
    z = improvement / sigma
    return improvement * normal_cdf(z) + sigma * normal_pdf(z)


def propose_next_vector(
    rng: random.Random,
    x_train: np.ndarray,
    y_train: np.ndarray,
    tune_imu: bool,
    wheel_mode: str,
    specs: tuple[ParamSpec, ...],
    acq_samples: int,
    seen: set[str],
) -> np.ndarray:
    dim = len(specs) + (1 if tune_imu else 0)
    pool = np.asarray([[rng.random() for _ in range(dim)] for _ in range(acq_samples)], dtype=float)
    if tune_imu:
        pool[:, -1] = np.asarray([rng.choice([0.0, 1.0]) for _ in range(acq_samples)])
    acquisition = expected_improvement(x_train, y_train, pool)
    order = np.argsort(-acquisition)
    for idx in order:
        candidate = unit_to_candidate(pool[idx], tune_imu, wheel_mode, specs)
        if candidate_key(candidate) not in seen:
            return pool[idx]
    return pool[int(order[0])]


def quoted_ros_command(args: list[str], setup_files: list[str]) -> list[str]:
    setup = " && ".join(f"source {shlex.quote(path)}" for path in setup_files if path)
    command = " ".join(shlex.quote(str(arg)) for arg in args)
    if setup:
        command = f"{setup} && exec {command}"
    else:
        command = f"exec {command}"
    return ["bash", "-lc", command]


def start_process(
    label: str,
    args: list[str],
    log_path: Path,
    setup_files: list[str],
    ros_domain_id: int,
    cwd: Path,
) -> subprocess.Popen[Any]:
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(ros_domain_id)
    env.setdefault("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    log_file = log_path.open("w", encoding="utf-8")
    command = quoted_ros_command(args, setup_files)
    print(f"[proc] start {label}: {' '.join(shlex.quote(a) for a in args)}", flush=True)
    return subprocess.Popen(
        command,
        cwd=str(cwd),
        env=env,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def stop_process(label: str, process: subprocess.Popen[Any], timeout_s: float = 10.0) -> None:
    if process.poll() is not None:
        return
    print(f"[proc] stop {label}", flush=True)
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=timeout_s)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5.0)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


def ros_param_args(params: dict[str, Any]) -> list[str]:
    output: list[str] = []
    for key, value in params.items():
        if isinstance(value, bool):
            rendered = "true" if value else "false"
        else:
            rendered = str(value)
        output.extend(["-p", f"{key}:={rendered}"])
    return output


def run_filter_on_bag(
    candidate: Candidate,
    reference: BagReference,
    eval_dir: Path,
    eval_index: int,
    bag_index: int,
    topics: TopicConfig,
    setup_files: list[str],
    ros_domain_id: int,
    play_rate: float,
    workspace_root: Path,
    startup_wait_s: float,
) -> Path:
    bag_dir = eval_dir / f"{reference.name}_filter_bag"
    output_topic = f"/filter_odom_opt/e{eval_index:03d}/b{bag_index:02d}"
    filter_log = eval_dir / f"{reference.name}_filter.log"
    record_log = eval_dir / f"{reference.name}_record.log"
    play_log = eval_dir / f"{reference.name}_play.log"

    params = candidate.as_ros_params(output_topic, topics)
    filter_args = [
        "ros2",
        "run",
        "adaptive_odom_filter",
        "adaptive_odom_filter_node",
        "--ros-args",
        "--params-file",
        str(workspace_root / "config" / "adaptive_filter_parameters.yaml"),
    ] + ros_param_args(params)
    record_args = ["ros2", "bag", "record", "--storage", "mcap", "-o", str(bag_dir), output_topic]
    play_args = ["ros2", "bag", "play", str(reference.path), "--clock", "--rate", str(play_rate)]

    filter_proc: subprocess.Popen[Any] | None = None
    record_proc: subprocess.Popen[Any] | None = None
    try:
        filter_proc = start_process(
            "filter",
            filter_args,
            filter_log,
            setup_files,
            ros_domain_id,
            workspace_root,
        )
        time.sleep(startup_wait_s)
        record_proc = start_process(
            "record",
            record_args,
            record_log,
            setup_files,
            ros_domain_id,
            workspace_root,
        )
        time.sleep(startup_wait_s)
        play_proc = start_process(
            "play",
            play_args,
            play_log,
            setup_files,
            ros_domain_id,
            workspace_root,
        )
        play_return = play_proc.wait()
        if play_return != 0:
            raise RuntimeError(f"ros2 bag play failed for {reference.name}; see {play_log}")
        time.sleep(1.5)
    finally:
        if record_proc is not None:
            stop_process("record", record_proc)
        if filter_proc is not None:
            stop_process("filter", filter_proc)
    return bag_dir


def evaluate_filter_bag(
    filter_bag_path: Path,
    reference: BagReference,
    output_topic: str,
    samples: int,
    jump_threshold_m: float,
) -> tuple[dict[str, float], dict[str, np.ndarray], OdomSeries]:
    series = read_odom_topics(filter_bag_path, [output_topic])[output_topic]
    metrics, arrays = compute_path_errors(series, reference.gt, samples, jump_threshold_m)
    metrics["score"] = bag_score(metrics, reference.fastlio_metrics)
    metrics["fastlio_xy_rmse_m"] = reference.fastlio_metrics["xy_rmse_m"]
    metrics["wheel_xy_rmse_m"] = reference.wheel_metrics["xy_rmse_m"]
    metrics["beats_fastlio"] = float(metrics["xy_rmse_m"] < reference.fastlio_metrics["xy_rmse_m"])
    return metrics, arrays, series


def save_baseline_summary(references: list[BagReference], output_dir: Path) -> None:
    path = output_dir / "baseline_metrics.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "bag",
            "source",
            "messages",
            "path_length_xy_m",
            "path_length_ratio",
            "x_rmse_m",
            "y_rmse_m",
            "z_rmse_m",
            "xy_rmse_m",
            "xyz_rmse_m",
            "yaw_rmse_rad",
            "max_step_xy_m",
            "jump_count",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for ref in references:
            for source, metrics in (("fastlio", ref.fastlio_metrics), ("wheel", ref.wheel_metrics)):
                row = {"bag": ref.name, "source": source}
                row.update({field: metrics.get(field, math.nan) for field in fieldnames if field not in row})
                writer.writerow(row)


def save_json(path: Path, data: Any) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def append_evaluation_csv(path: Path, row: dict[str, Any]) -> None:
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def plot_baseline_paths(reference: BagReference, output_dir: Path, samples: int) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(11, 9), constrained_layout=True)
    series_items = [
        ("Ground truth", reference.gt, "black"),
        ("FAST-LIO2", reference.fastlio, "tab:blue"),
        ("Wheel", reference.wheel, "tab:orange"),
    ]
    for ax, (title, series, color) in zip(axes.flat, series_items):
        ax.plot(series.xyz[:, 0], series.xyz[:, 1], color=color, linewidth=1.4)
        ax.set_title(title)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
    ax = axes.flat[3]
    for title, series, color in series_items:
        ax.plot(series.xyz[:, 0], series.xyz[:, 1], label=title, linewidth=1.1, alpha=0.85, color=color)
    ax.set_title("Overlay")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.suptitle(reference.name)
    fig.savefig(output_dir / f"{reference.name}_baseline_paths.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    for source_name, series, color in (("FAST-LIO2", reference.fastlio, "tab:blue"), ("Wheel", reference.wheel, "tab:orange")):
        _, arrays = compute_path_errors(series, reference.gt, samples, jump_threshold_m=0.5)
        progress = arrays["progress"]
        axes[0].plot(progress, arrays["x_error"], label=source_name, color=color)
        axes[1].plot(progress, arrays["y_error"], label=source_name, color=color)
        axes[2].plot(progress, arrays["z_error"], label=source_name, color=color)
        axes[3].plot(progress, arrays["yaw_error"], label=source_name, color=color)
    labels = ("x error [m]", "y error [m]", "z error [m]", "yaw error [rad]")
    for ax, label in zip(axes, labels):
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.legend()
    axes[-1].set_xlabel("normalized path progress")
    fig.suptitle(f"{reference.name} baseline path-only errors")
    fig.savefig(output_dir / f"{reference.name}_baseline_errors.png", dpi=160)
    plt.close(fig)


def plot_best_candidate(
    output_dir: Path,
    references: list[BagReference],
    filter_series: dict[str, OdomSeries],
    samples: int,
    jump_threshold_m: float,
) -> None:
    plots_dir = output_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    for ref in references:
        if ref.name not in filter_series:
            continue
        filt = filter_series[ref.name]
        fig, axes = plt.subplots(2, 2, figsize=(11, 9), constrained_layout=True)
        items = [
            ("Ground truth", ref.gt, "black"),
            ("FAST-LIO2", ref.fastlio, "tab:blue"),
            ("Wheel", ref.wheel, "tab:orange"),
            ("Best filter", filt, "tab:green"),
        ]
        for ax, (title, series, color) in zip(axes.flat, items):
            ax.plot(series.xyz[:, 0], series.xyz[:, 1], color=color, linewidth=1.3)
            ax.set_title(title)
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.3)
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
        fig.suptitle(f"{ref.name} best candidate paths")
        fig.savefig(plots_dir / f"{ref.name}_best_paths_4panel.png", dpi=170)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(9, 8), constrained_layout=True)
        for title, series, color in items:
            ax.plot(series.xyz[:, 0], series.xyz[:, 1], label=title, linewidth=1.15, alpha=0.9, color=color)
        ax.set_title(f"{ref.name} best candidate overlay")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.legend()
        fig.savefig(plots_dir / f"{ref.name}_best_paths_overlay.png", dpi=170)
        plt.close(fig)

        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
        for source_name, series, color in (
            ("FAST-LIO2", ref.fastlio, "tab:blue"),
            ("Wheel", ref.wheel, "tab:orange"),
            ("Best filter", filt, "tab:green"),
        ):
            _, arrays = compute_path_errors(series, ref.gt, samples, jump_threshold_m)
            progress = arrays["progress"]
            axes[0].plot(progress, arrays["x_error"], label=source_name, color=color)
            axes[1].plot(progress, arrays["y_error"], label=source_name, color=color)
            axes[2].plot(progress, arrays["z_error"], label=source_name, color=color)
            axes[3].plot(progress, arrays["yaw_error"], label=source_name, color=color)
        labels = ("x error [m]", "y error [m]", "z error [m]", "yaw error [rad]")
        for ax, label in zip(axes, labels):
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.3)
            ax.legend()
        axes[-1].set_xlabel("normalized path progress")
        fig.suptitle(f"{ref.name} best candidate path-only errors")
        fig.savefig(plots_dir / f"{ref.name}_best_errors.png", dpi=170)
        plt.close(fig)


def plot_convergence(output_dir: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    plots_dir = output_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    indices = [int(row["eval_index"]) for row in rows]
    scores = [float(row["train_score"]) for row in rows]
    best = np.minimum.accumulate(scores)
    fig, ax = plt.subplots(figsize=(9, 5), constrained_layout=True)
    ax.plot(indices, scores, "o-", label="candidate")
    ax.plot(indices, best, "k--", label="best so far")
    ax.set_xlabel("evaluation")
    ax.set_ylabel("aggregate training score")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.savefig(plots_dir / "optimization_convergence.png", dpi=170)
    plt.close(fig)


def write_markdown_summary(
    output_dir: Path,
    rows: list[dict[str, Any]],
    best_row: dict[str, Any] | None,
    validation_summary: dict[str, Any] | None,
) -> None:
    lines = ["# Bayesian filter optimization summary", ""]
    if best_row is None:
        lines.append("No completed evaluations.")
    else:
        lines.extend(
            [
                "## Best training candidate",
                "",
                f"- evaluation: `{best_row['eval_index']}`",
                f"- training score: `{float(best_row['train_score']):.6f}`",
                f"- mean XY RMSE: `{float(best_row['mean_xy_rmse_m']):.4f} m`",
                f"- mean FAST-LIO2 XY RMSE: `{float(best_row['mean_fastlio_xy_rmse_m']):.4f} m`",
                f"- mean path-length ratio: `{float(best_row['mean_path_length_ratio']):.4f}`",
                f"- bags beating FAST-LIO2: `{int(float(best_row['bags_beating_fastlio']))}`",
                "",
                "Parameters:",
                "",
                f"- `lidarG`: `{float(best_row['lidarG']):.8g}`",
                f"- `wheelGVx`: `{float(best_row['wheelGVx']):.8g}`",
                f"- `wheelGVy`: `{float(best_row['wheelGVy']):.8g}`",
                f"- `wheelGWz`: `{float(best_row['wheelGWz']):.8g}`",
                f"- `alpha_lidar`: `{float(best_row['alpha_lidar']):.8g}`",
                f"- `wheelVxScale`: `{float(best_row['wheelVxScale']):.8g}`",
                f"- `enableImu`: `{bool(int(float(best_row['enableImu'])))}`",
                f"- `enableWheel`: `{bool(int(float(best_row['enableWheel'])))}`",
                f"- `wheel_type_func`: `{int(float(best_row['wheel_type_func']))}`",
                f"- `wheelOffset`: `{float(best_row['wheelOffset']):.8g}`",
                f"- `gamma_vx`: `{float(best_row['gamma_vx']):.8g}`",
                f"- `gamma_omegaz`: `{float(best_row['gamma_omegaz']):.8g}`",
                f"- `delta_vx`: `{float(best_row['delta_vx']):.8g}`",
                f"- `delta_omegaz`: `{float(best_row['delta_omegaz']):.8g}`",
                "",
            ]
        )
    if validation_summary:
        lines.extend(["## Validation", ""])
        for bag, metrics in validation_summary.items():
            lines.append(
                f"- `{bag}`: filter XY RMSE `{metrics['xy_rmse_m']:.4f} m`, "
                f"FAST-LIO2 `{metrics['fastlio_xy_rmse_m']:.4f} m`, "
                f"score `{metrics['score']:.4f}`"
            )
        lines.append("")
    lines.extend(
        [
            "## Notes",
            "",
            "- The comparison ignores message time alignment and uses normalized path progress.",
            "- The optimizer records a unique live filter topic for each candidate and ignores bagged `/filter_odom`.",
            "- Re-run the best candidate at `--play-rate 1.0` before using gains as final robot parameters.",
            "",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def run_analyze(args: argparse.Namespace) -> None:
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    topics = TopicConfig(fastlio=args.fastlio_topic, wheel=args.wheel_topic, imu=args.imu_topic, gt=args.gt_topic)
    bag_paths = [Path(path).expanduser().resolve() for path in args.bag]
    references = load_reference_bags(bag_paths, topics, args.samples, args.jump_threshold)
    save_baseline_summary(references, output_dir)
    for ref in references:
        plot_baseline_paths(ref, output_dir / "plots", args.samples)
    save_json(
        output_dir / "analysis_config.json",
        {
            "bags": [str(path) for path in bag_paths],
            "topics": topics.__dict__,
            "samples": args.samples,
            "jump_threshold": args.jump_threshold,
        },
    )
    print(f"[done] baseline analysis written to {output_dir}", flush=True)


def summarize_eval(eval_index: int, candidate: Candidate, per_bag_metrics: dict[str, dict[str, float]]) -> dict[str, Any]:
    scores = [metrics["score"] for metrics in per_bag_metrics.values()]
    xy_rmse = [metrics["xy_rmse_m"] for metrics in per_bag_metrics.values()]
    fastlio_xy = [metrics["fastlio_xy_rmse_m"] for metrics in per_bag_metrics.values()]
    ratios = [metrics["path_length_ratio"] for metrics in per_bag_metrics.values()]
    beats = [metrics["beats_fastlio"] for metrics in per_bag_metrics.values()]
    row: dict[str, Any] = {
        "eval_index": eval_index,
        "train_score": aggregate_scores(scores),
        "mean_bag_score": float(np.mean(scores)) if scores else math.inf,
        "std_bag_score": float(np.std(scores)) if scores else math.inf,
        "mean_xy_rmse_m": float(np.mean(xy_rmse)) if xy_rmse else math.inf,
        "mean_fastlio_xy_rmse_m": float(np.mean(fastlio_xy)) if fastlio_xy else math.inf,
        "mean_path_length_ratio": float(np.mean(ratios)) if ratios else math.nan,
        "bags_beating_fastlio": float(np.sum(beats)) if beats else 0.0,
        "completed_bags": float(len(per_bag_metrics)),
        "pruned": 0.0,
    }
    row.update(candidate.csv_row())
    return row


def evaluate_candidate(
    candidate: Candidate,
    references: list[BagReference],
    eval_index: int,
    output_dir: Path,
    topics: TopicConfig,
    setup_files: list[str],
    domain_base: int,
    play_rate: float,
    workspace_root: Path,
    samples: int,
    jump_threshold_m: float,
    startup_wait_s: float,
    prune_score: float | None,
    prune_factor: float,
    early_stop_min_bags: int,
) -> tuple[dict[str, Any], dict[str, OdomSeries], dict[str, dict[str, float]]]:
    eval_dir = output_dir / f"eval_{eval_index:03d}"
    eval_dir.mkdir(parents=True, exist_ok=True)
    save_json(eval_dir / "candidate.json", candidate.csv_row())

    per_bag_metrics: dict[str, dict[str, float]] = {}
    filter_series_by_bag: dict[str, OdomSeries] = {}
    pruned = False
    for bag_index, ref in enumerate(references):
        domain = safe_ros_domain_id(domain_base, eval_index * 10 + bag_index)
        filter_bag = run_filter_on_bag(
            candidate,
            ref,
            eval_dir,
            eval_index,
            bag_index,
            topics,
            setup_files,
            domain,
            play_rate,
            workspace_root,
            startup_wait_s,
        )
        output_topic = f"/filter_odom_opt/e{eval_index:03d}/b{bag_index:02d}"
        metrics, _, series = evaluate_filter_bag(filter_bag, ref, output_topic, samples, jump_threshold_m)
        per_bag_metrics[ref.name] = metrics
        filter_series_by_bag[ref.name] = series
        save_json(eval_dir / f"{ref.name}_metrics.json", metrics)
        print(
            f"[eval {eval_index:03d}] {ref.name}: "
            f"xy_rmse={metrics['xy_rmse_m']:.4f}m "
            f"fastlio={metrics['fastlio_xy_rmse_m']:.4f}m "
            f"score={metrics['score']:.4f}",
            flush=True,
        )
        if prune_score is not None and len(per_bag_metrics) >= early_stop_min_bags:
            partial_score = aggregate_scores([item["score"] for item in per_bag_metrics.values()])
            if partial_score > prune_score * prune_factor:
                pruned = True
                print(
                    f"[eval {eval_index:03d}] pruned after {len(per_bag_metrics)} bag(s): "
                    f"partial_score={partial_score:.4f} best={prune_score:.4f}",
                    flush=True,
                )
                break

    summary = summarize_eval(eval_index, candidate, per_bag_metrics)
    if pruned:
        missing_bags = max(len(references) - len(per_bag_metrics), 0)
        summary["train_score"] = float(summary["train_score"]) + 10.0 * missing_bags
        summary["pruned"] = 1.0
    save_json(eval_dir / "summary.json", {"summary": summary, "per_bag": per_bag_metrics})
    return summary, filter_series_by_bag, per_bag_metrics


def validate_candidate(
    candidate: Candidate,
    references: list[BagReference],
    output_dir: Path,
    topics: TopicConfig,
    setup_files: list[str],
    domain_base: int,
    play_rate: float,
    workspace_root: Path,
    samples: int,
    jump_threshold_m: float,
    startup_wait_s: float,
) -> tuple[dict[str, OdomSeries], dict[str, dict[str, float]]]:
    if not references:
        return {}, {}
    validation_dir = output_dir / "validation_best"
    validation_dir.mkdir(parents=True, exist_ok=True)
    per_bag_metrics: dict[str, dict[str, float]] = {}
    filter_series_by_bag: dict[str, OdomSeries] = {}
    for bag_index, ref in enumerate(references):
        filter_bag = run_filter_on_bag(
            candidate,
            ref,
            validation_dir,
            999,
            bag_index,
            topics,
            setup_files,
            safe_ros_domain_id(domain_base, 900 + bag_index),
            play_rate,
            workspace_root,
            startup_wait_s,
        )
        output_topic = f"/filter_odom_opt/e999/b{bag_index:02d}"
        metrics, _, series = evaluate_filter_bag(filter_bag, ref, output_topic, samples, jump_threshold_m)
        per_bag_metrics[ref.name] = metrics
        filter_series_by_bag[ref.name] = series
        save_json(validation_dir / f"{ref.name}_metrics.json", metrics)
        print(
            f"[validation] {ref.name}: xy_rmse={metrics['xy_rmse_m']:.4f}m "
            f"fastlio={metrics['fastlio_xy_rmse_m']:.4f}m score={metrics['score']:.4f}",
            flush=True,
        )
    return filter_series_by_bag, per_bag_metrics


def run_optimize(args: argparse.Namespace) -> None:
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    workspace_root = Path(args.workspace_root).expanduser().resolve()
    topics = TopicConfig(fastlio=args.fastlio_topic, wheel=args.wheel_topic, imu=args.imu_topic, gt=args.gt_topic)
    setup_files = [str(Path(path).expanduser()) for path in args.setup]
    specs = search_space_for_wheel_mode(args.wheel_mode)

    train_paths = [Path(path).expanduser().resolve() for path in args.train_bag]
    validation_paths = [Path(path).expanduser().resolve() for path in args.validation_bag]
    train_refs = load_reference_bags(train_paths, topics, args.samples, args.jump_threshold)
    validation_refs = load_reference_bags(validation_paths, topics, args.samples, args.jump_threshold)
    save_baseline_summary(train_refs + validation_refs, output_dir)
    save_json(
        output_dir / "optimizer_config.json",
        {
            "train_bags": [str(path) for path in train_paths],
            "validation_bags": [str(path) for path in validation_paths],
            "topics": topics.__dict__,
            "samples": args.samples,
            "jump_threshold": args.jump_threshold,
            "play_rate": args.play_rate,
            "max_evals": args.max_evals,
            "init_evals": args.init_evals,
            "tune_imu": args.tune_imu,
            "wheel_mode": args.wheel_mode,
            "search_space": [spec.__dict__ for spec in specs],
        },
    )

    rng = random.Random(args.seed)
    seen: set[str] = set()
    x_rows: list[np.ndarray] = []
    y_rows: list[float] = []
    eval_rows: list[dict[str, Any]] = []
    best_summary: dict[str, Any] | None = None
    best_candidate: Candidate | None = None
    best_filter_series: dict[str, OdomSeries] = {}

    initial = initial_vectors(rng, args.init_evals, args.tune_imu, args.wheel_mode, specs)
    eval_csv = output_dir / "evaluations.csv"
    eval_jsonl = output_dir / "evaluations.jsonl"

    for eval_index in range(args.max_evals):
        if eval_index < len(initial):
            vector = initial[eval_index]
        else:
            vector = propose_next_vector(
                rng,
                np.vstack(x_rows),
                np.asarray(y_rows, dtype=float),
                args.tune_imu,
                args.wheel_mode,
                specs,
                args.acq_samples,
                seen,
            )
        candidate = unit_to_candidate(vector, args.tune_imu, args.wheel_mode, specs)
        key = candidate_key(candidate)
        if key in seen:
            for _ in range(100):
                vector = np.asarray([rng.random() for _ in range(len(specs) + (1 if args.tune_imu else 0))])
                if args.tune_imu:
                    vector[-1] = float(rng.choice([0, 1]))
                candidate = unit_to_candidate(vector, args.tune_imu, args.wheel_mode, specs)
                key = candidate_key(candidate)
                if key not in seen:
                    break
        seen.add(key)

        print(f"[eval {eval_index:03d}] candidate {candidate}", flush=True)
        summary, filter_series, per_bag_metrics = evaluate_candidate(
            candidate,
            train_refs,
            eval_index,
            output_dir,
            topics,
            setup_files,
            args.domain_base,
            args.play_rate,
            workspace_root,
            args.samples,
            args.jump_threshold,
            args.startup_wait,
            float(best_summary["train_score"]) if best_summary is not None and args.early_stop else None,
            args.early_stop_factor,
            args.early_stop_min_bags,
        )
        append_evaluation_csv(eval_csv, summary)
        with eval_jsonl.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps({"summary": summary, "per_bag": per_bag_metrics}, sort_keys=True) + "\n")
        eval_rows.append(summary)
        x_rows.append(candidate_to_unit(candidate, args.tune_imu, specs))
        y_rows.append(float(summary["train_score"]))
        if best_summary is None or float(summary["train_score"]) < float(best_summary["train_score"]):
            best_summary = summary
            best_candidate = candidate
            best_filter_series = filter_series
            save_json(output_dir / "best_candidate.json", {"summary": best_summary, "candidate": candidate.csv_row()})
            plot_best_candidate(output_dir, train_refs, best_filter_series, args.samples, args.jump_threshold)
        plot_convergence(output_dir, eval_rows)
        print(f"[eval {eval_index:03d}] aggregate_score={summary['train_score']:.6f}", flush=True)

    validation_series: dict[str, OdomSeries] = {}
    validation_metrics: dict[str, dict[str, float]] = {}
    if best_candidate is not None and validation_refs:
        validation_series, validation_metrics = validate_candidate(
            best_candidate,
            validation_refs,
            output_dir,
            topics,
            setup_files,
            args.domain_base,
            args.play_rate,
            workspace_root,
            args.samples,
            args.jump_threshold,
            args.startup_wait,
        )
        plot_best_candidate(output_dir, validation_refs, validation_series, args.samples, args.jump_threshold)
        save_json(output_dir / "validation_best.json", validation_metrics)

    write_markdown_summary(output_dir, eval_rows, best_summary, validation_metrics)
    print(f"[done] optimization written to {output_dir}", flush=True)


def candidate_from_args(args: argparse.Namespace) -> Candidate:
    return Candidate(
        lidarG=args.lidarG,
        wheelGVx=args.wheelGVx,
        wheelGVy=args.wheelGVy,
        wheelGWz=args.wheelGWz,
        alpha_lidar=args.alpha_lidar,
        wheelVxScale=args.wheelVxScale,
        enableImu=args.enable_imu,
        enableWheel=args.enable_wheel,
        wheel_type_func=args.wheel_type_func,
        wheelOffset=args.wheelOffset,
        gamma_vx=args.gamma_vx,
        gamma_omegaz=args.gamma_omegaz,
        delta_vx=args.delta_vx,
        delta_omegaz=args.delta_omegaz,
    )


def run_evaluate(args: argparse.Namespace) -> None:
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    workspace_root = Path(args.workspace_root).expanduser().resolve()
    topics = TopicConfig(fastlio=args.fastlio_topic, wheel=args.wheel_topic, imu=args.imu_topic, gt=args.gt_topic)
    setup_files = [str(Path(path).expanduser()) for path in args.setup]
    references = load_reference_bags(
        [Path(path).expanduser().resolve() for path in args.bag],
        topics,
        args.samples,
        args.jump_threshold,
    )
    candidate = candidate_from_args(args)
    summary, filter_series, per_bag_metrics = evaluate_candidate(
        candidate,
        references,
        0,
        output_dir,
        topics,
        setup_files,
        args.domain_base,
        args.play_rate,
        workspace_root,
        args.samples,
        args.jump_threshold,
        args.startup_wait,
        None,
        1.0,
        1,
    )
    save_baseline_summary(references, output_dir)
    save_json(output_dir / "candidate_metrics.json", {"summary": summary, "per_bag": per_bag_metrics})
    plot_best_candidate(output_dir, references, filter_series, args.samples, args.jump_threshold)
    write_markdown_summary(output_dir, [summary], summary, None)
    print(f"[done] candidate evaluation written to {output_dir}", flush=True)


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--fastlio-topic", default="/Odometry")
    parser.add_argument("--wheel-topic", default="/odom")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--gt-topic", default="/scout_mini/Odom")
    parser.add_argument("--samples", type=int, default=700)
    parser.add_argument("--jump-threshold", type=float, default=0.5)
    parser.add_argument("--output-dir", required=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline Bayesian optimization for adaptive_odom_filter.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    analyze = subparsers.add_parser("analyze", help="Analyze source odometries against ground-truth path geometry.")
    add_common_args(analyze)
    analyze.add_argument("--bag", action="append", required=True)

    optimize = subparsers.add_parser("optimize", help="Run filter-in-the-loop Bayesian optimization.")
    add_common_args(optimize)
    optimize.add_argument("--train-bag", action="append", required=True)
    optimize.add_argument("--validation-bag", action="append", default=[])
    optimize.add_argument("--workspace-root", default="/home/diogo/ros2_ws/src/adaptive_odom_filter")
    optimize.add_argument("--setup", action="append", default=list(DEFAULT_SOURCE_SETUP))
    optimize.add_argument("--max-evals", type=int, default=10)
    optimize.add_argument("--init-evals", type=int, default=5)
    optimize.add_argument("--acq-samples", type=int, default=6000)
    optimize.add_argument("--seed", type=int, default=7)
    optimize.add_argument("--domain-base", type=int, default=80)
    optimize.add_argument("--play-rate", type=float, default=1.0)
    optimize.add_argument("--startup-wait", type=float, default=5.0)
    optimize.add_argument("--wheel-mode", choices=("adaptive", "fixed", "none"), default="adaptive")
    optimize.add_argument("--tune-imu", action=argparse.BooleanOptionalAction, default=True)
    optimize.add_argument("--early-stop", action=argparse.BooleanOptionalAction, default=True)
    optimize.add_argument("--early-stop-factor", type=float, default=3.0)
    optimize.add_argument("--early-stop-min-bags", type=int, default=1)

    evaluate = subparsers.add_parser("evaluate", help="Run one explicit candidate on one or more bags.")
    add_common_args(evaluate)
    evaluate.add_argument("--bag", action="append", required=True)
    evaluate.add_argument("--workspace-root", default="/home/diogo/ros2_ws/src/adaptive_odom_filter")
    evaluate.add_argument("--setup", action="append", default=list(DEFAULT_SOURCE_SETUP))
    evaluate.add_argument("--domain-base", type=int, default=150)
    evaluate.add_argument("--play-rate", type=float, default=1.0)
    evaluate.add_argument("--startup-wait", type=float, default=5.0)
    evaluate.add_argument("--lidarG", type=float, required=True)
    evaluate.add_argument("--wheelGVx", type=float, required=True)
    evaluate.add_argument("--wheelGVy", type=float, required=True)
    evaluate.add_argument("--wheelGWz", type=float, required=True)
    evaluate.add_argument("--alpha-lidar", dest="alpha_lidar", type=float, required=True)
    evaluate.add_argument("--wheelVxScale", type=float, required=True)
    evaluate.add_argument("--wheel-type-func", type=int, choices=(0, 1), default=1)
    evaluate.add_argument("--wheelOffset", type=float, default=1.0e-5)
    evaluate.add_argument("--gamma-vx", dest="gamma_vx", type=float, default=0.05)
    evaluate.add_argument("--gamma-omegaz", dest="gamma_omegaz", type=float, default=0.01)
    evaluate.add_argument("--delta-vx", dest="delta_vx", type=float, default=0.0001)
    evaluate.add_argument("--delta-omegaz", dest="delta_omegaz", type=float, default=0.00001)
    evaluate.add_argument("--enable-imu", action=argparse.BooleanOptionalAction, default=False)
    evaluate.add_argument("--enable-wheel", action=argparse.BooleanOptionalAction, default=True)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.command == "analyze":
        run_analyze(args)
        return 0
    if args.command == "optimize":
        run_optimize(args)
        return 0
    if args.command == "evaluate":
        run_evaluate(args)
        return 0
    parser.error(f"unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
