#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, TextIO

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

VALID_PHASES = {"baseline", "active", "post"}


@dataclass
class PhaseStats:
    count: int = 0
    max_planar_speed: float = 0.0
    first_x: Optional[float] = None
    first_y: Optional[float] = None
    last_x: Optional[float] = None
    last_y: Optional[float] = None

    def update(self, position_x: float, position_y: float,
               planar_speed: float) -> None:
        if self.first_x is None:
            self.first_x = position_x
            self.first_y = position_y

        self.last_x = position_x
        self.last_y = position_y
        self.count += 1
        self.max_planar_speed = max(self.max_planar_speed, planar_speed)

    def delta_xy(self) -> float:
        if (self.first_x is None or self.first_y is None or
                self.last_x is None or self.last_y is None):
            return 0.0

        return math.hypot(
            self.last_x - self.first_x,
            self.last_y - self.first_y,
        )


class OdomCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("h2_dog_odom_capture")

        self.args = args
        self.phase_file = Path(args.phase_file)
        self.csv_path = Path(args.csv)
        self.summary_path = Path(args.summary)

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)

        self.csv_stream: TextIO = self.csv_path.open(
            "w", encoding="utf-8", newline=""
        )
        self.writer = csv.writer(self.csv_stream)
        self.writer.writerow(
            [
                "wall_time_ns",
                "monotonic_s",
                "ros_stamp_sec",
                "ros_stamp_nanosec",
                "phase",
                "position_x",
                "position_y",
                "position_z",
                "linear_x",
                "linear_y",
                "linear_z",
                "angular_z",
                "planar_speed",
            ]
        )

        self.stats = {
            "baseline": PhaseStats(),
            "active": PhaseStats(),
            "post": PhaseStats(),
        }

        self.sample_period_s = 1.0 / args.sample_hz
        self.last_sample_monotonic = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subscription = self.create_subscription(
            Odometry,
            args.topic,
            self.odom_callback,
            qos,
        )

    def read_phase(self) -> Optional[str]:
        try:
            phase = self.phase_file.read_text(encoding="utf-8").strip()
        except OSError:
            return None

        if phase not in VALID_PHASES:
            return None
        return phase

    def odom_callback(self, message: Odometry) -> None:
        monotonic_now = time.monotonic()
        if monotonic_now - self.last_sample_monotonic < self.sample_period_s:
            return

        phase = self.read_phase()
        if phase is None:
            return

        self.last_sample_monotonic = monotonic_now

        position = message.pose.pose.position
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        planar_speed = math.hypot(linear.x, linear.y)

        self.stats[phase].update(position.x, position.y, planar_speed)

        self.writer.writerow(
            [
                time.time_ns(),
                f"{monotonic_now:.9f}",
                message.header.stamp.sec,
                message.header.stamp.nanosec,
                phase,
                f"{position.x:.9f}",
                f"{position.y:.9f}",
                f"{position.z:.9f}",
                f"{linear.x:.9f}",
                f"{linear.y:.9f}",
                f"{linear.z:.9f}",
                f"{angular.z:.9f}",
                f"{planar_speed:.9f}",
            ]
        )

    def finish(self) -> None:
        self.csv_stream.flush()
        os.fsync(self.csv_stream.fileno())
        self.csv_stream.close()

        baseline = self.stats["baseline"]
        active = self.stats["active"]
        post = self.stats["post"]
        total_count = baseline.count + active.count + post.count

        required_phase_samples = max(10, int(self.args.sample_hz * 0.50))
        capture_ok = (
            baseline.count >= required_phase_samples
            and active.count >= required_phase_samples
            and post.count >= required_phase_samples
        )

        effective_speed_threshold = max(
            self.args.speed_threshold,
            baseline.max_planar_speed + self.args.speed_margin,
        )
        speed_response = active.max_planar_speed >= effective_speed_threshold

        active_delta_xy = active.delta_xy()
        position_response = active_delta_xy >= self.args.delta_threshold

        # Position delta is retained as diagnostic evidence, but it is not
        # sufficient by itself because long active windows can accumulate drift.
        odom_response_detected = capture_ok and speed_response

        fields = [
            ("odom_topic", self.args.topic),
            ("sample_hz", f"{self.args.sample_hz:.3f}"),
            ("odom_sample_count", str(total_count)),
            ("odom_baseline_sample_count", str(baseline.count)),
            ("odom_active_sample_count", str(active.count)),
            ("odom_post_sample_count", str(post.count)),
            (
                "odom_baseline_max_planar_speed",
                f"{baseline.max_planar_speed:.9f}",
            ),
            (
                "odom_active_max_planar_speed",
                f"{active.max_planar_speed:.9f}",
            ),
            (
                "odom_post_max_planar_speed",
                f"{post.max_planar_speed:.9f}",
            ),
            ("odom_active_delta_xy", f"{active_delta_xy:.9f}"),
            (
                "odom_effective_speed_threshold",
                f"{effective_speed_threshold:.9f}",
            ),
            ("odom_speed_response", "1" if speed_response else "0"),
            ("odom_position_response", "1" if position_response else "0"),
            ("odom_capture_ok", "1" if capture_ok else "0"),
            (
                "odom_response_detected",
                "1" if odom_response_detected else "0",
            ),
        ]

        temporary = self.summary_path.with_name(
            self.summary_path.name + ".tmp"
        )
        with temporary.open("w", encoding="utf-8") as stream:
            for key, value in fields:
                stream.write(f"{key}={value}\n")
            stream.flush()
            os.fsync(stream.fileno())

        os.replace(temporary, self.summary_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/dog_odom")
    parser.add_argument("--phase-file", required=True)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--summary", required=True)
    parser.add_argument("--sample-hz", type=float, default=50.0)
    parser.add_argument("--speed-threshold", type=float, default=0.02)
    parser.add_argument("--speed-margin", type=float, default=0.015)
    parser.add_argument("--delta-threshold", type=float, default=0.01)
    args = parser.parse_args()

    if not math.isfinite(args.sample_hz) or args.sample_hz <= 0.0:
        parser.error("--sample-hz must be positive")

    for name in ("speed_threshold", "speed_margin", "delta_threshold"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0.0:
            parser.error(
                f"--{name.replace('_', '-')} must be non-negative"
            )

    return args


def main() -> int:
    args = parse_args()
    stop_event = threading.Event()

    def request_stop(signum: int, frame: object) -> None:
        del signum
        del frame
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rclpy.init(args=None)
    node = OdomCapture(args)

    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.10)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.finish()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
