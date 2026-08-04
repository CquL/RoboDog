#!/usr/bin/env python3
"""Compare X30 ROS1 quadrangle messages with factory TCP gridmap frames."""

from __future__ import annotations

import argparse
import csv
import ipaddress
import json
import math
import statistics
import struct
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import BinaryIO, Iterable, Iterator, Sequence


OP_MSG_DATA = 0x02
OP_CHUNK = 0x05
OP_CONNECTION = 0x07

POINT_FIELD_INT8 = 1
POINT_FIELD_UINT8 = 2
POINT_FIELD_INT16 = 3
POINT_FIELD_UINT16 = 4
POINT_FIELD_INT32 = 5
POINT_FIELD_UINT32 = 6
POINT_FIELD_FLOAT32 = 7
POINT_FIELD_FLOAT64 = 8

POINT_FIELD_FORMATS = {
    POINT_FIELD_INT8: "b",
    POINT_FIELD_UINT8: "B",
    POINT_FIELD_INT16: "h",
    POINT_FIELD_UINT16: "H",
    POINT_FIELD_INT32: "i",
    POINT_FIELD_UINT32: "I",
    POINT_FIELD_FLOAT32: "f",
    POINT_FIELD_FLOAT64: "d",
}


@dataclass(frozen=True)
class BagConnection:
    connection_id: int
    topic: str
    message_type: str


@dataclass(frozen=True)
class BagMessage:
    connection: BagConnection
    record_time_ns: int
    data: bytes


@dataclass(frozen=True)
class PointCloudFrame:
    record_time_ns: int
    stamp_sec: int
    stamp_nsec: int
    frame_id: str
    width: int
    height: int
    points: tuple[tuple[float, float, float], ...]

    @property
    def stamp_ns(self) -> int:
        return self.stamp_sec * 1_000_000_000 + self.stamp_nsec


@dataclass(frozen=True)
class PoseFrame:
    record_time_ns: int
    stamp_sec: int
    stamp_nsec: int
    frame_id: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]

    @property
    def stamp_ns(self) -> int:
        return self.stamp_sec * 1_000_000_000 + self.stamp_nsec


@dataclass(frozen=True)
class MultiArrayDimension:
    label: str
    size: int
    stride: int


@dataclass(frozen=True)
class GridMapLayer:
    name: str
    dimensions: tuple[MultiArrayDimension, ...]
    data_offset: int
    values: tuple[float, ...]
    raw_data: bytes = b""


@dataclass(frozen=True)
class GridMapFrame:
    record_time_ns: int
    sequence: int
    stamp_sec: int
    stamp_nsec: int
    frame_id: str
    resolution: float
    length_x: float
    length_y: float
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    layers: tuple[GridMapLayer, ...]
    basic_layers: tuple[str, ...]
    outer_start_index: int
    inner_start_index: int

    @property
    def stamp_ns(self) -> int:
        return self.stamp_sec * 1_000_000_000 + self.stamp_nsec

    def layer(self, name: str) -> GridMapLayer:
        for layer in self.layers:
            if layer.name == name:
                return layer
        raise KeyError(f"GridMap has no layer {name!r}")


@dataclass(frozen=True)
class TransformStampedFrame:
    sequence: int
    stamp_sec: int
    stamp_nsec: int
    frame_id: str
    child_frame_id: str
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]

    @property
    def stamp_ns(self) -> int:
        return self.stamp_sec * 1_000_000_000 + self.stamp_nsec

    @property
    def normalized_frame_id(self) -> str:
        return normalize_frame_id(self.frame_id)

    @property
    def normalized_child_frame_id(self) -> str:
        return normalize_frame_id(self.child_frame_id)


@dataclass(frozen=True)
class TFMessageFrame:
    record_time_ns: int
    topic: str
    transforms: tuple[TransformStampedFrame, ...]


@dataclass(frozen=True)
class RosbagPlaneSegInputs:
    grid_maps: tuple[GridMapFrame, ...]
    quadrangles: tuple[PointCloudFrame, ...]
    look_poses: tuple[PoseFrame, ...]
    tf_messages: tuple[TFMessageFrame, ...]
    tf_static_messages: tuple[TFMessageFrame, ...]
    height_map_mode_values: Counter[int]
    height_map_mode_state_values: Counter[int]


@dataclass(frozen=True)
class TcpFrame:
    total_pack: int
    frame_len: int
    head_id: int
    layer_size: int
    points_size: int
    points: tuple[tuple[float, float, float], ...]
    time_count_ms: float
    source_stamp_sec: int
    source_stamp_nsec: int
    pack_time_sec: int
    pack_time_nsec: int
    end_id: int

    @property
    def source_stamp_ns(self) -> int:
        return self.source_stamp_sec * 1_000_000_000 + self.source_stamp_nsec

    @property
    def pack_stamp_ns(self) -> int:
        return self.pack_time_sec * 1_000_000_000 + self.pack_time_nsec


@dataclass(frozen=True)
class QuadrangleMetrics:
    area_m2: float
    z_range_m: float
    planarity_error_m: float | None
    duplicate_vertices: int
    degenerate: bool


class BinaryCursor:
    def __init__(self, data: bytes):
        self.data = data
        self.offset = 0

    def read(self, size: int) -> bytes:
        end = self.offset + size
        if end > len(self.data):
            raise ValueError(
                f"serialized message ended at {len(self.data)}, "
                f"need {size} bytes at offset {self.offset}"
            )
        value = self.data[self.offset:end]
        self.offset = end
        return value

    def unpack(self, fmt: str):
        size = struct.calcsize(fmt)
        return struct.unpack(fmt, self.read(size))

    def uint8(self) -> int:
        return self.unpack("<B")[0]

    def uint16(self) -> int:
        return self.unpack("<H")[0]

    def uint32(self) -> int:
        return self.unpack("<I")[0]

    def float32(self) -> float:
        return self.unpack("<f")[0]

    def float64(self) -> float:
        return self.unpack("<d")[0]

    def string(self) -> str:
        size = self.uint32()
        return self.read(size).decode("utf-8", errors="replace")


def read_exact(stream: BinaryIO, size: int) -> bytes:
    data = stream.read(size)
    if len(data) != size:
        raise EOFError(f"expected {size} bytes, received {len(data)}")
    return data


def parse_bag_header_fields(data: bytes) -> dict[str, bytes]:
    fields: dict[str, bytes] = {}
    offset = 0
    while offset < len(data):
        if offset + 4 > len(data):
            raise ValueError("truncated ROS bag header field length")
        field_len = struct.unpack_from("<I", data, offset)[0]
        offset += 4
        field = data[offset : offset + field_len]
        if len(field) != field_len:
            raise ValueError("truncated ROS bag header field")
        offset += field_len
        key, separator, value = field.partition(b"=")
        if not separator:
            raise ValueError("ROS bag header field has no '=' separator")
        fields[key.decode("ascii")] = value
    return fields


def bag_uint8(fields: dict[str, bytes], key: str) -> int:
    value = fields[key]
    if len(value) != 1:
        raise ValueError(f"bag field {key!r} is not uint8")
    return value[0]


def bag_uint32(fields: dict[str, bytes], key: str) -> int:
    return struct.unpack("<I", fields[key])[0]


def bag_time_ns(fields: dict[str, bytes], key: str) -> int:
    sec, nsec = struct.unpack("<II", fields[key])
    return sec * 1_000_000_000 + nsec


def iter_bag_records(stream: BinaryIO, limit: int | None = None):
    start = stream.tell()
    while limit is None or stream.tell() - start < limit:
        consumed = stream.tell() - start
        if limit is not None and consumed == limit:
            return
        raw_header_len = stream.read(4)
        if not raw_header_len:
            return
        if len(raw_header_len) != 4:
            raise EOFError("truncated ROS bag record header length")
        header_len = struct.unpack("<I", raw_header_len)[0]
        header = parse_bag_header_fields(read_exact(stream, header_len))
        data_len = struct.unpack("<I", read_exact(stream, 4))[0]
        data = read_exact(stream, data_len)
        yield header, data

    if stream.tell() - start != limit:
        raise ValueError("ROS bag chunk record exceeded declared size")


def parse_connection(header: dict[str, bytes], data: bytes) -> BagConnection:
    connection_header = parse_bag_header_fields(data)
    return BagConnection(
        connection_id=bag_uint32(header, "conn"),
        topic=header["topic"].decode("utf-8", errors="replace"),
        message_type=connection_header["type"].decode("utf-8", errors="replace"),
    )


def parse_pointcloud2(data: bytes, record_time_ns: int) -> PointCloudFrame:
    cursor = BinaryCursor(data)
    cursor.uint32()  # Header.seq
    stamp_sec = cursor.uint32()
    stamp_nsec = cursor.uint32()
    frame_id = cursor.string()
    height = cursor.uint32()
    width = cursor.uint32()

    fields = []
    for _ in range(cursor.uint32()):
        fields.append(
            {
                "name": cursor.string(),
                "offset": cursor.uint32(),
                "datatype": cursor.uint8(),
                "count": cursor.uint32(),
            }
        )

    is_bigendian = bool(cursor.uint8())
    point_step = cursor.uint32()
    row_step = cursor.uint32()
    point_data = cursor.read(cursor.uint32())
    cursor.uint8()  # is_dense

    if row_step and len(point_data) < row_step * height:
        raise ValueError("PointCloud2 data is shorter than row_step * height")

    by_name = {field["name"]: field for field in fields}
    missing = {"x", "y", "z"} - by_name.keys()
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {sorted(missing)}")

    endian = ">" if is_bigendian else "<"
    points = []
    for row in range(height):
        row_offset = row * row_step
        for column in range(width):
            point_offset = row_offset + column * point_step
            coordinates = []
            for name in ("x", "y", "z"):
                field = by_name[name]
                fmt = POINT_FIELD_FORMATS.get(field["datatype"])
                if fmt is None or field["count"] < 1:
                    raise ValueError(f"unsupported PointCloud2 field {name}: {field}")
                coordinates.append(
                    float(
                        struct.unpack_from(
                            endian + fmt,
                            point_data,
                            point_offset + field["offset"],
                        )[0]
                    )
                )
            points.append(tuple(coordinates))

    return PointCloudFrame(
        record_time_ns=record_time_ns,
        stamp_sec=stamp_sec,
        stamp_nsec=stamp_nsec,
        frame_id=frame_id,
        width=width,
        height=height,
        points=tuple(points),
    )


def parse_pose_stamped(data: bytes, record_time_ns: int) -> PoseFrame:
    cursor = BinaryCursor(data)
    cursor.uint32()  # Header.seq
    stamp_sec = cursor.uint32()
    stamp_nsec = cursor.uint32()
    frame_id = cursor.string()
    position = cursor.unpack("<ddd")
    orientation = cursor.unpack("<dddd")
    if cursor.offset != len(data):
        raise ValueError("PoseStamped payload has trailing bytes")
    return PoseFrame(
        record_time_ns=record_time_ns,
        stamp_sec=stamp_sec,
        stamp_nsec=stamp_nsec,
        frame_id=frame_id,
        position=position,
        orientation=orientation,
    )


def normalize_frame_id(frame_id: str) -> str:
    """Normalize a TF frame id without changing any non-prefix characters."""
    return frame_id.lstrip("/")


def parse_transform_stamped(cursor: BinaryCursor) -> TransformStampedFrame:
    sequence = cursor.uint32()
    stamp_sec = cursor.uint32()
    stamp_nsec = cursor.uint32()
    frame_id = cursor.string()
    child_frame_id = cursor.string()
    translation = cursor.unpack("<ddd")
    rotation = cursor.unpack("<dddd")
    return TransformStampedFrame(
        sequence=sequence,
        stamp_sec=stamp_sec,
        stamp_nsec=stamp_nsec,
        frame_id=frame_id,
        child_frame_id=child_frame_id,
        translation=translation,
        rotation=rotation,
    )


def parse_tf_message(
    data: bytes,
    record_time_ns: int,
    topic: str = "/tf",
) -> TFMessageFrame:
    cursor = BinaryCursor(data)
    transforms = tuple(
        parse_transform_stamped(cursor) for _ in range(cursor.uint32())
    )
    if cursor.offset != len(data):
        raise ValueError(
            f"TFMessage payload has {len(data) - cursor.offset} trailing bytes"
        )
    return TFMessageFrame(
        record_time_ns=record_time_ns,
        topic=topic,
        transforms=transforms,
    )


def parse_string_array(cursor: BinaryCursor) -> tuple[str, ...]:
    return tuple(cursor.string() for _ in range(cursor.uint32()))


def parse_float32_multi_array(
    cursor: BinaryCursor,
    layer_name: str,
) -> GridMapLayer:
    dimensions = tuple(
        MultiArrayDimension(
            label=cursor.string(),
            size=cursor.uint32(),
            stride=cursor.uint32(),
        )
        for _ in range(cursor.uint32())
    )
    data_offset = cursor.uint32()
    value_count = cursor.uint32()
    raw_data = cursor.read(value_count * 4)
    values = struct.unpack(f"<{value_count}f", raw_data) if value_count else ()
    return GridMapLayer(
        name=layer_name,
        dimensions=dimensions,
        data_offset=data_offset,
        values=values,
        raw_data=raw_data,
    )


def parse_grid_map(data: bytes, record_time_ns: int) -> GridMapFrame:
    cursor = BinaryCursor(data)
    sequence = cursor.uint32()
    stamp_sec = cursor.uint32()
    stamp_nsec = cursor.uint32()
    frame_id = cursor.string()
    resolution = cursor.float64()
    length_x = cursor.float64()
    length_y = cursor.float64()
    position = cursor.unpack("<ddd")
    orientation = cursor.unpack("<dddd")
    layer_names = parse_string_array(cursor)
    basic_layers = parse_string_array(cursor)

    layer_count = cursor.uint32()
    if layer_count != len(layer_names):
        raise ValueError(
            "GridMap layer name/data count mismatch: "
            f"names={len(layer_names)}, data={layer_count}"
        )
    layers = tuple(
        parse_float32_multi_array(cursor, layer_name)
        for layer_name in layer_names
    )
    outer_start_index = cursor.uint16()
    inner_start_index = cursor.uint16()
    if cursor.offset != len(data):
        raise ValueError(
            f"GridMap payload has {len(data) - cursor.offset} trailing bytes"
        )

    return GridMapFrame(
        record_time_ns=record_time_ns,
        sequence=sequence,
        stamp_sec=stamp_sec,
        stamp_nsec=stamp_nsec,
        frame_id=frame_id,
        resolution=resolution,
        length_x=length_x,
        length_y=length_y,
        position=position,
        orientation=orientation,
        layers=layers,
        basic_layers=basic_layers,
        outer_start_index=outer_start_index,
        inner_start_index=inner_start_index,
    )


def iter_rosbag_messages(bag_path: Path) -> Iterator[BagMessage]:
    """Yield ROS message records while opening and traversing a bag only once."""
    from io import BytesIO

    connections: dict[int, BagConnection] = {}

    def process_record(
        header: dict[str, bytes], data: bytes
    ) -> BagMessage | None:
        operation = bag_uint8(header, "op")
        if operation == OP_CONNECTION:
            connection = parse_connection(header, data)
            connections[connection.connection_id] = connection
            return None
        if operation != OP_MSG_DATA:
            return None

        connection_id = bag_uint32(header, "conn")
        connection = connections.get(connection_id)
        if connection is None:
            raise ValueError(f"message uses unknown connection {connection_id}")
        return BagMessage(
            connection=connection,
            record_time_ns=bag_time_ns(header, "time"),
            data=data,
        )

    with bag_path.open("rb") as stream:
        version = stream.readline()
        if version != b"#ROSBAG V2.0\n":
            raise ValueError(f"unsupported ROS bag version header: {version!r}")

        for header, data in iter_bag_records(stream):
            operation = bag_uint8(header, "op")
            if operation == OP_CHUNK:
                compression = header["compression"].decode("ascii")
                if compression != "none":
                    raise ValueError(
                        f"compressed ROS bag chunks are unsupported: {compression}"
                    )
                declared_size = bag_uint32(header, "size")
                if declared_size != len(data):
                    raise ValueError(
                        f"chunk size mismatch: header={declared_size}, data={len(data)}"
                    )
                records = iter_bag_records(BytesIO(data), limit=len(data))
            else:
                records = ((header, data),)

            for nested_header, nested_data in records:
                message = process_record(nested_header, nested_data)
                if message is not None:
                    yield message


def require_message_type(message: BagMessage, expected_type: str) -> None:
    if message.connection.message_type != expected_type:
        raise ValueError(
            f"topic {message.connection.topic!r} has unexpected type "
            f"{message.connection.message_type!r}; expected {expected_type!r}"
        )


def parse_int32_message(message: BagMessage) -> int:
    require_message_type(message, "std_msgs/Int32")
    if len(message.data) != 4:
        raise ValueError(f"{message.connection.topic} payload is not int32")
    return struct.unpack("<i", message.data)[0]


def read_rosbag_plane_seg_inputs(
    bag_path: Path,
    grid_map_topic: str = "/deeprobotics_local_height_map_mid360/height_map",
    quadrangles_topic: str = "/plane_seg/quadrangels",
    look_pose_topic: str = "/plane_seg/look_pose",
    height_map_mode_topic: str = "/height_map_mode",
    height_map_mode_state_topic: str = "/height_map_mode_state",
    tf_topic: str = "/tf",
    tf_static_topic: str = "/tf_static",
) -> RosbagPlaneSegInputs:
    """Read all plane-seg fixture inputs in one sequential bag traversal."""
    grid_maps: list[GridMapFrame] = []
    quadrangles: list[PointCloudFrame] = []
    look_poses: list[PoseFrame] = []
    tf_messages: list[TFMessageFrame] = []
    tf_static_messages: list[TFMessageFrame] = []
    height_map_mode_values: Counter[int] = Counter()
    height_map_mode_state_values: Counter[int] = Counter()

    for message in iter_rosbag_messages(bag_path):
        topic = message.connection.topic
        if topic == grid_map_topic:
            require_message_type(message, "grid_map_msgs/GridMap")
            grid_maps.append(parse_grid_map(message.data, message.record_time_ns))
        elif topic == quadrangles_topic:
            require_message_type(message, "sensor_msgs/PointCloud2")
            quadrangles.append(
                parse_pointcloud2(message.data, message.record_time_ns)
            )
        elif topic == look_pose_topic:
            require_message_type(message, "geometry_msgs/PoseStamped")
            look_poses.append(
                parse_pose_stamped(message.data, message.record_time_ns)
            )
        elif topic == height_map_mode_topic:
            height_map_mode_values[parse_int32_message(message)] += 1
        elif topic == height_map_mode_state_topic:
            height_map_mode_state_values[parse_int32_message(message)] += 1
        elif topic == tf_topic:
            require_message_type(message, "tf2_msgs/TFMessage")
            tf_messages.append(
                parse_tf_message(message.data, message.record_time_ns, topic)
            )
        elif topic == tf_static_topic:
            require_message_type(message, "tf2_msgs/TFMessage")
            tf_static_messages.append(
                parse_tf_message(message.data, message.record_time_ns, topic)
            )

    grid_maps.sort(key=lambda frame: (frame.stamp_ns, frame.record_time_ns))
    quadrangles.sort(key=lambda frame: (frame.stamp_ns, frame.record_time_ns))
    look_poses.sort(key=lambda frame: (frame.stamp_ns, frame.record_time_ns))
    tf_messages.sort(key=lambda message: message.record_time_ns)
    tf_static_messages.sort(key=lambda message: message.record_time_ns)
    return RosbagPlaneSegInputs(
        grid_maps=tuple(grid_maps),
        quadrangles=tuple(quadrangles),
        look_poses=tuple(look_poses),
        tf_messages=tuple(tf_messages),
        tf_static_messages=tuple(tf_static_messages),
        height_map_mode_values=height_map_mode_values,
        height_map_mode_state_values=height_map_mode_state_values,
    )


def read_rosbag_grid_maps(
    bag_path: Path,
    topic: str = "/deeprobotics_local_height_map_mid360/height_map",
) -> list[GridMapFrame]:
    frames = []
    for message in iter_rosbag_messages(bag_path):
        if message.connection.topic != topic:
            continue
        require_message_type(message, "grid_map_msgs/GridMap")
        frames.append(parse_grid_map(message.data, message.record_time_ns))
    frames.sort(key=lambda frame: (frame.stamp_ns, frame.record_time_ns))
    return frames


def read_rosbag_quadrangles(
    bag_path: Path,
    topic: str = "/plane_seg/quadrangels",
) -> tuple[list[PointCloudFrame], Counter[int], list[PoseFrame]]:
    frames: list[PointCloudFrame] = []
    mode_values: Counter[int] = Counter()
    look_poses: list[PoseFrame] = []

    for message in iter_rosbag_messages(bag_path):
        message_topic = message.connection.topic
        if message_topic == topic:
            require_message_type(message, "sensor_msgs/PointCloud2")
            frames.append(parse_pointcloud2(message.data, message.record_time_ns))
        elif message_topic == "/height_map_mode_state":
            mode_values[parse_int32_message(message)] += 1
        elif message_topic == "/plane_seg/look_pose":
            require_message_type(message, "geometry_msgs/PoseStamped")
            look_poses.append(
                parse_pose_stamped(message.data, message.record_time_ns)
            )

    frames.sort(key=lambda frame: (frame.stamp_ns, frame.record_time_ns))
    look_poses.sort(key=lambda frame: (frame.stamp_ns, frame.record_time_ns))
    return frames, mode_values, look_poses


def parse_pcap_tcp_payloads(
    pcap_path: Path,
    source_ip: str = "192.168.1.105",
    destination_ip: str = "192.168.1.103",
    destination_port: int = 49999,
) -> list[tuple[int, bytes]]:
    magic_formats = {
        b"\xd4\xc3\xb2\xa1": ("<", 1_000),
        b"\xa1\xb2\xc3\xd4": (">", 1_000),
        b"\x4d\x3c\xb2\xa1": ("<", 1),
        b"\xa1\xb2\x3c\x4d": (">", 1),
    }
    expected_source = ipaddress.ip_address(source_ip).packed
    expected_destination = ipaddress.ip_address(destination_ip).packed
    payloads: list[tuple[int, bytes]] = []

    with pcap_path.open("rb") as stream:
        magic = read_exact(stream, 4)
        if magic not in magic_formats:
            raise ValueError(f"unsupported pcap magic: {magic.hex()}")
        endian, fraction_to_ns = magic_formats[magic]
        global_rest = read_exact(stream, 20)
        _, _, _, _, _, network = struct.unpack(endian + "HHIIII", global_rest)
        if network != 1:
            raise ValueError(f"pcap link type {network} is not Ethernet")

        while True:
            packet_header = stream.read(16)
            if not packet_header:
                break
            if len(packet_header) != 16:
                raise EOFError("truncated pcap packet header")
            ts_sec, ts_fraction, included_len, _ = struct.unpack(
                endian + "IIII", packet_header
            )
            packet = read_exact(stream, included_len)
            timestamp_ns = ts_sec * 1_000_000_000 + ts_fraction * fraction_to_ns

            if len(packet) < 14:
                continue
            ethernet_type = struct.unpack_from("!H", packet, 12)[0]
            network_offset = 14
            if ethernet_type == 0x8100 and len(packet) >= 18:
                ethernet_type = struct.unpack_from("!H", packet, 16)[0]
                network_offset = 18
            if ethernet_type != 0x0800 or len(packet) < network_offset + 20:
                continue

            ip_header = packet[network_offset:]
            if ip_header[0] >> 4 != 4 or ip_header[9] != 6:
                continue
            ip_header_len = (ip_header[0] & 0x0F) * 4
            if ip_header_len < 20 or len(ip_header) < ip_header_len + 20:
                continue
            if ip_header[12:16] != expected_source:
                continue
            if ip_header[16:20] != expected_destination:
                continue

            total_length = struct.unpack_from("!H", ip_header, 2)[0]
            tcp = ip_header[ip_header_len:total_length]
            if len(tcp) < 20:
                continue
            source_port, dest_port, sequence = struct.unpack_from("!HHI", tcp, 0)
            del source_port
            if dest_port != destination_port:
                continue
            tcp_header_len = (tcp[12] >> 4) * 4
            if tcp_header_len < 20 or len(tcp) < tcp_header_len:
                continue
            payload = tcp[tcp_header_len:]
            if payload:
                payloads.append((sequence, bytes(payload)))

    return payloads


def reassemble_tcp_payload(payloads: Iterable[tuple[int, bytes]]) -> bytes:
    ordered = sorted(payloads, key=lambda item: item[0])
    if not ordered:
        return b""
    stream = bytearray()
    expected_sequence = ordered[0][0]
    for sequence, payload in ordered:
        end_sequence = sequence + len(payload)
        if end_sequence <= expected_sequence:
            continue
        if sequence > expected_sequence:
            raise ValueError(
                f"TCP capture has a gap of {sequence - expected_sequence} bytes"
            )
        overlap = max(0, expected_sequence - sequence)
        stream.extend(payload[overlap:])
        expected_sequence = end_sequence
    return bytes(stream)


def decode_factory_tcp_stream(data: bytes) -> list[TcpFrame]:
    frames: list[TcpFrame] = []
    offset = 0
    while offset < len(data):
        if len(data) - offset < 8:
            raise ValueError(f"truncated TCP frame prefix at offset {offset}")
        total_pack = struct.unpack_from("<I", data, offset)[0]
        frame_len = struct.unpack_from("<I", data, offset + 4)[0]
        if total_pack != 1:
            raise ValueError(f"unsupported total_pack={total_pack} at offset {offset}")
        if frame_len < 40 or offset + 4 + frame_len > len(data):
            raise ValueError(f"invalid frame_len={frame_len} at offset {offset}")

        frame = data[offset + 4 : offset + 4 + frame_len]
        parsed_len, head_id, layer_size, points_size = struct.unpack_from(
            "<IIII", frame, 0
        )
        if parsed_len != frame_len:
            raise ValueError("frame length field changed while parsing")
        if points_size % 12 or frame_len != 40 + points_size:
            raise ValueError(
                f"invalid points_size={points_size} for frame_len={frame_len}"
            )

        point_count = points_size // 12
        points = tuple(
            struct.unpack_from("<fff", frame, 16 + index * 12)
            for index in range(point_count)
        )
        tail_offset = 16 + points_size
        (
            time_count_ms,
            source_stamp_sec,
            source_stamp_nsec,
            pack_time_sec,
            pack_time_nsec,
            end_id,
        ) = struct.unpack_from("<fIIIII", frame, tail_offset)
        frames.append(
            TcpFrame(
                total_pack=total_pack,
                frame_len=frame_len,
                head_id=head_id,
                layer_size=layer_size,
                points_size=points_size,
                points=points,
                time_count_ms=time_count_ms,
                source_stamp_sec=source_stamp_sec,
                source_stamp_nsec=source_stamp_nsec,
                pack_time_sec=pack_time_sec,
                pack_time_nsec=pack_time_nsec,
                end_id=end_id,
            )
        )
        offset += 4 + frame_len
    return frames


def vector_subtract(a: Sequence[float], b: Sequence[float]):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a: Sequence[float], b: Sequence[float]):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(vector: Sequence[float]) -> float:
    return math.sqrt(sum(component * component for component in vector))


def triangle_area(a, b, c) -> float:
    return 0.5 * norm(cross(vector_subtract(b, a), vector_subtract(c, a)))


def quadrangle_metrics(points: Sequence[Sequence[float]]) -> QuadrangleMetrics:
    if len(points) != 4:
        raise ValueError("quadrangle metrics require exactly four vertices")
    area = triangle_area(points[0], points[1], points[2]) + triangle_area(
        points[0], points[2], points[3]
    )
    z_values = [point[2] for point in points]
    duplicates = sum(
        1
        for left in range(4)
        for right in range(left + 1, 4)
        if norm(vector_subtract(points[left], points[right])) <= 1e-6
    )

    normal = None
    origin = points[0]
    for first in range(1, 3):
        for second in range(first + 1, 4):
            candidate = cross(
                vector_subtract(points[first], origin),
                vector_subtract(points[second], origin),
            )
            candidate_norm = norm(candidate)
            if candidate_norm > 1e-9:
                normal = tuple(component / candidate_norm for component in candidate)
                break
        if normal is not None:
            break

    planarity_error = None
    if normal is not None:
        planarity_error = max(
            abs(
                sum(
                    normal[index] * (point[index] - origin[index])
                    for index in range(3)
                )
            )
            for point in points
        )

    return QuadrangleMetrics(
        area_m2=area,
        z_range_m=max(z_values) - min(z_values),
        planarity_error_m=planarity_error,
        duplicate_vertices=duplicates,
        degenerate=duplicates > 0 or area <= 1e-6 or normal is None,
    )


def points_match(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
    tolerance: float = 1e-6,
) -> tuple[bool, float]:
    if len(left) != len(right):
        return False, math.inf
    max_error = 0.0
    for left_point, right_point in zip(left, right):
        for left_value, right_value in zip(left_point, right_point):
            if math.isnan(left_value) and math.isnan(right_value):
                continue
            error = abs(left_value - right_value)
            max_error = max(max_error, error)
            if error > tolerance:
                return False, max_error
    return True, max_error


def counter_to_dict(counter: Counter[int]) -> dict[str, int]:
    return {str(key): counter[key] for key in sorted(counter)}


def numeric_summary(values: Sequence[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {
        "minimum": min(values),
        "median": statistics.median(values),
        "mean": statistics.fmean(values),
        "maximum": max(values),
    }


def analyze(bag_path: Path, pcap_path: Path, output_dir: Path) -> dict:
    bag_frames, mode_values, look_poses = read_rosbag_quadrangles(bag_path)
    tcp_payloads = parse_pcap_tcp_payloads(pcap_path)
    tcp_stream = reassemble_tcp_payload(tcp_payloads)
    tcp_frames = decode_factory_tcp_stream(tcp_stream)

    output_dir.mkdir(parents=True, exist_ok=True)
    tcp_by_stamp = {frame.source_stamp_ns: frame for frame in tcp_frames}
    bag_by_stamp = {frame.stamp_ns: frame for frame in bag_frames}

    matched = 0
    exact_points = 0
    mismatched_points = 0
    max_point_error = 0.0
    frame_rows = []
    group_rows = []
    total_groups = 0
    degenerate_groups = 0
    divisible_by_four_frames = 0
    frames_with_one_degenerate_group = 0
    first_group_degenerate_frames = 0
    first_group_last_vertices_equal_frames = 0
    nonfirst_degenerate_groups = 0
    first_group_code_candidates: Counter[int] = Counter()

    for frame_index, frame in enumerate(bag_frames):
        metrics = []
        if len(frame.points) % 4 == 0:
            divisible_by_four_frames += 1
            for group_index in range(len(frame.points) // 4):
                vertices = frame.points[group_index * 4 : group_index * 4 + 4]
                group_metric = quadrangle_metrics(vertices)
                metrics.append(group_metric)
                total_groups += 1
                degenerate_groups += int(group_metric.degenerate)
                row = {
                    "stamp_sec": frame.stamp_sec,
                    "stamp_nsec": frame.stamp_nsec,
                    "frame_index": frame_index,
                    "group_index": group_index,
                    **asdict(group_metric),
                }
                for vertex_index, vertex in enumerate(vertices):
                    row[f"p{vertex_index}_x"] = vertex[0]
                    row[f"p{vertex_index}_y"] = vertex[1]
                    row[f"p{vertex_index}_z"] = vertex[2]
                group_rows.append(row)

                if group_index == 0:
                    first_group_degenerate_frames += int(group_metric.degenerate)
                    first_group_last_vertices_equal_frames += int(
                        vertices[2] == vertices[3]
                    )
                    candidate = round(vertices[0][0] - 0.001)
                    if abs(vertices[0][0] - (candidate + 0.001)) < 1e-4:
                        first_group_code_candidates[candidate] += 1
                else:
                    nonfirst_degenerate_groups += int(group_metric.degenerate)

        frame_degenerate_count = sum(metric.degenerate for metric in metrics)
        frames_with_one_degenerate_group += int(frame_degenerate_count == 1)

        tcp_frame = tcp_by_stamp.get(frame.stamp_ns)
        tcp_match = tcp_frame is not None
        point_match = False
        point_error = None
        if tcp_match:
            matched += 1
            point_match, point_error = points_match(frame.points, tcp_frame.points)
            max_point_error = max(max_point_error, point_error)
            if point_match:
                exact_points += 1
            else:
                mismatched_points += 1

        frame_rows.append(
            {
                "stamp_sec": frame.stamp_sec,
                "stamp_nsec": frame.stamp_nsec,
                "record_time_ns": frame.record_time_ns,
                "frame_id": frame.frame_id,
                "width": frame.width,
                "height": frame.height,
                "point_count": len(frame.points),
                "group_count": len(metrics),
                "degenerate_group_count": frame_degenerate_count,
                "tcp_frame_found": tcp_match,
                "tcp_points_match": point_match,
                "max_point_error": point_error,
            }
        )

    def write_csv(path: Path, rows: list[dict]) -> None:
        if not rows:
            path.write_text("", encoding="utf-8")
            return
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    write_csv(output_dir / "quadrangle_frames.csv", frame_rows)
    write_csv(output_dir / "quadrangle_groups.csv", group_rows)

    look_pose_rows = [
        {
            "stamp_sec": pose.stamp_sec,
            "stamp_nsec": pose.stamp_nsec,
            "record_time_ns": pose.record_time_ns,
            "frame_id": pose.frame_id,
            "position_x": pose.position[0],
            "position_y": pose.position[1],
            "position_z": pose.position[2],
            "orientation_x": pose.orientation[0],
            "orientation_y": pose.orientation[1],
            "orientation_z": pose.orientation[2],
            "orientation_w": pose.orientation[3],
        }
        for pose in look_poses
    ]
    write_csv(output_dir / "look_pose.csv", look_pose_rows)

    tcp_rows = [
        {
            "source_stamp_sec": frame.source_stamp_sec,
            "source_stamp_nsec": frame.source_stamp_nsec,
            "pack_time_sec": frame.pack_time_sec,
            "pack_time_nsec": frame.pack_time_nsec,
            "pack_latency_ms": (
                frame.pack_stamp_ns - frame.source_stamp_ns
            ) / 1_000_000.0,
            "time_count_ms": frame.time_count_ms,
            "point_count": len(frame.points),
            "frame_len": frame.frame_len,
            "head_id": frame.head_id,
            "layer_size": frame.layer_size,
            "end_id": frame.end_id,
        }
        for frame in tcp_frames
    ]
    write_csv(output_dir / "tcp_frames.csv", tcp_rows)

    bag_widths = Counter(len(frame.points) for frame in bag_frames)
    tcp_widths = Counter(len(frame.points) for frame in tcp_frames)
    ordered_tcp_frames = sorted(tcp_frames, key=lambda frame: frame.source_stamp_ns)
    source_intervals_ms = [
        (right.source_stamp_ns - left.source_stamp_ns) / 1_000_000.0
        for left, right in zip(ordered_tcp_frames, ordered_tcp_frames[1:])
    ]
    pack_latencies_ms = [
        (frame.pack_stamp_ns - frame.source_stamp_ns) / 1_000_000.0
        for frame in tcp_frames
    ]
    summary = {
        "inputs": {
            "bag": str(bag_path.resolve()),
            "pcap": str(pcap_path.resolve()),
        },
        "bag": {
            "quadrangle_frames": len(bag_frames),
            "frame_ids": dict(Counter(frame.frame_id for frame in bag_frames)),
            "point_count_distribution": counter_to_dict(bag_widths),
            "height_map_mode_state_distribution": counter_to_dict(mode_values),
            "timestamps_unique": len(bag_by_stamp) == len(bag_frames),
            "look_pose_frames": len(look_poses),
            "look_pose_frame_ids": dict(
                Counter(pose.frame_id for pose in look_poses)
            ),
            "look_pose_zero_stamp_frames": sum(
                pose.stamp_ns == 0 for pose in look_poses
            ),
            "look_pose_unique_positions": len(
                {pose.position for pose in look_poses}
            ),
            "look_pose_unique_orientations": len(
                {pose.orientation for pose in look_poses}
            ),
            "look_pose_exact_stamp_matches": len(
                set(bag_by_stamp).intersection(pose.stamp_ns for pose in look_poses)
            ),
        },
        "tcp": {
            "payload_segments": len(tcp_payloads),
            "reassembled_bytes": len(tcp_stream),
            "frames": len(tcp_frames),
            "point_count_distribution": counter_to_dict(tcp_widths),
            "head_ids": counter_to_dict(Counter(frame.head_id for frame in tcp_frames)),
            "layer_sizes": counter_to_dict(
                Counter(frame.layer_size for frame in tcp_frames)
            ),
            "end_ids": counter_to_dict(Counter(frame.end_id for frame in tcp_frames)),
            "timestamps_unique": len(tcp_by_stamp) == len(tcp_frames),
            "source_interval_ms": numeric_summary(source_intervals_ms),
            "pack_latency_ms": numeric_summary(pack_latencies_ms),
            "time_count_ms": numeric_summary(
                [frame.time_count_ms for frame in tcp_frames]
            ),
            "wire_format": {
                "endianness": "little",
                "stream_prefix": "uint32 total_pack",
                "frame": (
                    "uint32 frame_len, uint32 head_id, uint32 layer_size, "
                    "uint32 points_size, float32 xyz[], float32 time_count_ms, "
                    "uint32 source_sec, uint32 source_nsec, uint32 pack_sec, "
                    "uint32 pack_nsec, uint32 end_id"
                ),
            },
        },
        "comparison": {
            "matched_timestamps": matched,
            "bag_frames_without_tcp_match": len(bag_frames) - matched,
            "tcp_frames_outside_bag_window": len(tcp_frames) - matched,
            "frames_with_matching_points": exact_points,
            "frames_with_mismatching_points": mismatched_points,
            "maximum_coordinate_error": max_point_error,
        },
        "geometry": {
            "groups_of_four": total_groups,
            "degenerate_groups": degenerate_groups,
            "nondegenerate_groups": total_groups - degenerate_groups,
            "frames_divisible_by_four": divisible_by_four_frames,
            "frames_with_exactly_one_degenerate_group": (
                frames_with_one_degenerate_group
            ),
            "frames_with_degenerate_first_group": first_group_degenerate_frames,
            "frames_with_equal_p2_p3_in_first_group": (
                first_group_last_vertices_equal_frames
            ),
            "degenerate_groups_after_first_group": nonfirst_degenerate_groups,
            "first_group_p0_x_minus_0_001_candidates": counter_to_dict(
                first_group_code_candidates
            ),
            "first_group_semantics": "unknown; do not treat it as a support plane",
        },
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True, help="ROS1 bag path")
    parser.add_argument("--pcap", type=Path, required=True, help="TCP pcap path")
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Directory for JSON and CSV analysis output",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    summary = analyze(args.bag, args.pcap, args.output)
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    comparison = summary["comparison"]
    if comparison["bag_frames_without_tcp_match"]:
        return 2
    if comparison["frames_with_mismatching_points"]:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
