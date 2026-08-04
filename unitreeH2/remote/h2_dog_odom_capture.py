#!/usr/bin/env python3

# Stage 06E 的 /dog_odom 只读证据采集器。
#
# 数据链：原厂 ROS 2 /dog_odom -> 本节点 -> 分阶段 CSV 与 key=value summary。
# 本程序只创建 Odometry 订阅者，不发布 ROS/DDS 消息、不调用服务或运动 API。
# 唯一写入是调用者指定的项目日志文件；phase 文件由外层门禁脚本控制。

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

# 外层脚本只允许这三个阶段标签；未知/半写入内容会被当前样本跳过。
VALID_PHASES = {"baseline", "active", "post"}


@dataclass
# 一个采集阶段的样本数、最大平面速度和首末平面位置。
class PhaseStats:
    count: int = 0
    max_planar_speed: float = 0.0
    first_x: Optional[float] = None
    first_y: Optional[float] = None
    last_x: Optional[float] = None
    last_y: Optional[float] = None

    # 加入一帧有效样本，并保留该阶段首末位置与最大速度。
    def update(self, position_x: float, position_y: float,
               planar_speed: float) -> None:
        if self.first_x is None:
            self.first_x = position_x
            self.first_y = position_y

        self.last_x = position_x
        self.last_y = position_y
        self.count += 1
        self.max_planar_speed = max(self.max_planar_speed, planar_speed)

    # 返回该阶段首末位置的平面直线距离；样本不足时返回 0。
    def delta_xy(self) -> float:
        if (self.first_x is None or self.first_y is None or
                self.last_x is None or self.last_y is None):
            return 0.0

        return math.hypot(
            self.last_x - self.first_x,
            self.last_y - self.first_y,
        )


# 只读订阅并把 /dog_odom 按 phase 降采样写入证据文件。
class OdomCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        # 固定节点名用于 ROS 图谱识别；不创建 publisher/service/client。
        super().__init__("h2_dog_odom_capture")

        # 解析调用者指定的 phase、CSV 和 summary 路径。
        self.args = args
        self.phase_file = Path(args.phase_file)
        self.csv_path = Path(args.csv)
        self.summary_path = Path(args.summary)

        # 只创建目标文件的父目录，不修改其他 PC2/原厂工作区。
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)

        # CSV 从头创建，固定列包含墙钟、单调时间、ROS 时间、阶段、位姿和速度。
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

        # 三个阶段独立统计，避免基线噪声和运动后残留混入 active 指标。
        self.stats = {
            "baseline": PhaseStats(),
            "active": PhaseStats(),
            "post": PhaseStats(),
        }

        # sample_hz 只控制落盘降采样频率，不改变原厂 /dog_odom 发布频率。
        self.sample_period_s = 1.0 / args.sample_hz
        self.last_sample_monotonic = 0.0

        # 与已观察到的原厂 /dog_odom 契约匹配：Reliable、Volatile、
        # KeepLast 1000。这里只接收，不声明写端。
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 订阅话题默认 /dog_odom，可由显式参数覆盖用于离线/兼容性测试。
        self.subscription = self.create_subscription(
            Odometry,
            args.topic,
            self.odom_callback,
            qos,
        )

    # 读取外层脚本的阶段文件；缺失、I/O 错误或非法值均返回 None。
    def read_phase(self) -> Optional[str]:
        try:
            phase = self.phase_file.read_text(encoding="utf-8").strip()
        except OSError:
            return None

        if phase not in VALID_PHASES:
            return None
        return phase

    # 对活跃 phase 的消息限频、统计并写一行 CSV。
    def odom_callback(self, message: Odometry) -> None:
        # 单调时钟用于降采样，系统时间跳变不会导致突发重复采样。
        monotonic_now = time.monotonic()
        if monotonic_now - self.last_sample_monotonic < self.sample_period_s:
            return

        # phase 无效时宁可丢帧，也不把样本归入错误的 baseline/active/post。
        phase = self.read_phase()
        if phase is None:
            return

        self.last_sample_monotonic = monotonic_now

        # 只读取平移位置、线速度和 yaw 角速度；不修改消息。
        position = message.pose.pose.position
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        planar_speed = math.hypot(linear.x, linear.y)

        # 更新阶段聚合统计，并保留原始证据行（浮点固定九位）。
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

    # 落盘 CSV，计算门禁摘要，并以原子替换发布 summary。
    def finish(self) -> None:
        # 先 flush + fsync CSV，确保 summary 出现时对应逐帧证据已经持久化。
        self.csv_stream.flush()
        os.fsync(self.csv_stream.fileno())
        self.csv_stream.close()

        baseline = self.stats["baseline"]
        active = self.stats["active"]
        post = self.stats["post"]
        total_count = baseline.count + active.count + post.count

        # 每阶段至少需要 0.5 秒数据且不少于 10 帧，三阶段全部满足才完整。
        required_phase_samples = max(10, int(self.args.sample_hz * 0.50))
        capture_ok = (
            baseline.count >= required_phase_samples
            and active.count >= required_phase_samples
            and post.count >= required_phase_samples
        )

        # 响应速度阈值取固定下限与“基线最大噪声 + margin”的较大值，
        # 防止静止噪声被误认为机器人响应。
        effective_speed_threshold = max(
            self.args.speed_threshold,
            baseline.max_planar_speed + self.args.speed_margin,
        )
        speed_response = active.max_planar_speed >= effective_speed_threshold

        # 位置增量保留为诊断字段；它不能独立决定响应，因为长窗口会累积漂移。
        active_delta_xy = active.delta_xy()
        position_response = active_delta_xy >= self.args.delta_threshold

        # 成功响应要求采集完整且 active 最大速度越过动态阈值。
        odom_response_detected = capture_ok and speed_response

        # 固定 key=value 顺序与 08 门禁脚本的逐字段 schema 相匹配。
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

        # 先写同目录临时文件并 fsync，再 os.replace 原子替换正式 summary，
        # 外层脚本不会读到半行或部分字段。
        temporary = self.summary_path.with_name(
            self.summary_path.name + ".tmp"
        )
        with temporary.open("w", encoding="utf-8") as stream:
            for key, value in fields:
                stream.write(f"{key}={value}\n")
            stream.flush()
            os.fsync(stream.fileno())

        os.replace(temporary, self.summary_path)


# 解析采集路径、频率和响应阈值，并拒绝 NaN/Inf/负值。
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    # 输入话题和三类文件路径；phase/csv/summary 由 08 脚本为每次运行生成。
    parser.add_argument("--topic", default="/dog_odom")
    parser.add_argument("--phase-file", required=True)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--summary", required=True)
    # 降采样和响应阈值默认值服务于首次短脉冲证据，不控制机器人速度。
    parser.add_argument("--sample-hz", type=float, default=50.0)
    parser.add_argument("--speed-threshold", type=float, default=0.02)
    parser.add_argument("--speed-margin", type=float, default=0.015)
    parser.add_argument("--delta-threshold", type=float, default=0.01)
    args = parser.parse_args()

    # 频率必须为有限正数。
    if not math.isfinite(args.sample_hz) or args.sample_hz <= 0.0:
        parser.error("--sample-hz must be positive")

    # 三个阈值允许 0，但不能为负或非有限数。
    for name in ("speed_threshold", "speed_margin", "delta_threshold"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0.0:
            parser.error(
                f"--{name.replace('_', '-')} must be non-negative"
            )

    return args


# 运行有界于外部信号的 ROS 2 spin，并保证正常/异常退出都写摘要。
def main() -> int:
    args = parse_args()
    stop_event = threading.Event()

    # 信号处理器只设置线程事件，实际 I/O 清理由主循环完成。
    def request_stop(signum: int, frame: object) -> None:
        del signum
        del frame
        stop_event.set()

    # 08 脚本用 SIGINT 正常结束 post 采集，SIGTERM 也走相同安全清理路径。
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    # 使用调用进程已设置的 ROS/RMW/DDS 环境，不在程序内改 Domain 或网卡。
    rclpy.init(args=None)
    node = OdomCapture(args)

    try:
        # 0.10 秒 spin 超时保证收到停止事件后能及时退出。
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.10)
    except KeyboardInterrupt:
        pass
    finally:
        # 即使 spin 异常，也尽力先持久化证据，再销毁节点并关闭 rclpy。
        try:
            node.finish()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
