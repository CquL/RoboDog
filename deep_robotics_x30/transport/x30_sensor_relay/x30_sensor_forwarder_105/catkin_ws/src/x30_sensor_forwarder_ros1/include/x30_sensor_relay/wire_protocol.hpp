#pragma once

// 105 上的 ROS1 写入端与 106 上的 ROS2 读取端共用此应用层帧格式。
// 本链路只转发传感器消息，不用于机器人控制或传感器配置。
//
// 每个 TCP 帧均采用小端序，并包含以下固定 32 字节帧头：
//   magic[4], version:u16, stream:u16, sequence:u64, source_stamp_ns:u64,
//   payload_size:u32, reserved:u32.
// 帧头后紧跟各数据流的 payload。TCP 保证字节顺序与可靠传输；接收端发布
// ROS2 数据前还会独立校验 magic、version、stream、来源 IP 和声明长度。
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace x30_sensor_relay {

constexpr std::array<std::uint8_t, 4> kMagic = {{'X', '3', '0', 'R'}};
constexpr std::uint16_t kProtocolVersion = 1;
constexpr std::size_t kFrameHeaderSize = 32;
constexpr std::uint32_t kDefaultMaxPayloadBytes = 64U * 1024U * 1024U;

enum class StreamId : std::uint16_t {
  kPointCloud = 1,
  kImu = 2,
  kOdometry = 3,
};

struct FrameHeader {
  // sequence 在各数据流内独立计数，用于发现丢帧和重连间隙。
  // source_stamp_ns 保留 ROS1 消息时间，而不是使用接收时间。
  std::uint16_t version{0};
  StreamId stream{StreamId::kPointCloud};
  std::uint64_t sequence{0};
  std::uint64_t source_stamp_ns{0};
  std::uint32_t payload_size{0};
  std::uint32_t reserved{0};
};

// 以跨平台的小端序写入基础类型。消息序列化不直接复制 C++ 结构体，
// 避免结构体填充和本机字节序破坏稳定的线上协议。
class ByteWriter {
 public:
  explicit ByteWriter(std::size_t reserve_bytes = 0) {
    data_.reserve(reserve_bytes);
  }

  void writeU8(std::uint8_t value) {
    data_.push_back(value);
  }

  void writeU16(std::uint16_t value) {
    for (std::size_t i = 0; i < 2; ++i) {
      data_.push_back(static_cast<std::uint8_t>((value >> (8U * i)) & 0xffU));
    }
  }

  void writeU32(std::uint32_t value) {
    for (std::size_t i = 0; i < 4; ++i) {
      data_.push_back(static_cast<std::uint8_t>((value >> (8U * i)) & 0xffU));
    }
  }

  void writeU64(std::uint64_t value) {
    for (std::size_t i = 0; i < 8; ++i) {
      data_.push_back(static_cast<std::uint8_t>((value >> (8U * i)) & 0xffU));
    }
  }

  void writeDouble(double value) {
    static_assert(sizeof(double) == sizeof(std::uint64_t),
                  "The relay protocol requires IEEE-754 64-bit doubles.");
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    writeU64(bits);
  }

  void writeString(const std::string& value) {
    if (value.size() > std::numeric_limits<std::uint32_t>::max()) {
      writeU32(0);
      return;
    }
    writeU32(static_cast<std::uint32_t>(value.size()));
    writeBytes(reinterpret_cast<const std::uint8_t*>(value.data()), value.size());
  }

  void writeBytes(const std::uint8_t* bytes, std::size_t size) {
    if (bytes == nullptr || size == 0) {
      return;
    }
    data_.insert(data_.end(), bytes, bytes + size);
  }

  const std::vector<std::uint8_t>& data() const {
    return data_;
  }

  std::vector<std::uint8_t> take() {
    return std::move(data_);
  }

 private:
  std::vector<std::uint8_t> data_;
};

// 带边界检查的解码器，供合同测试和帧头校验使用。任一读取失败时，
// 调用方应拒绝整个帧。
class ByteReader {
 public:
  ByteReader(const std::uint8_t* bytes, std::size_t size)
      : bytes_(bytes), size_(size) {}

  bool readU8(std::uint8_t& value) {
    if (!has(1)) {
      return false;
    }
    value = bytes_[offset_++];
    return true;
  }

  bool readU16(std::uint16_t& value) {
    std::uint64_t decoded = 0;
    if (!readUnsigned(2, decoded)) {
      return false;
    }
    value = static_cast<std::uint16_t>(decoded);
    return true;
  }

  bool readU32(std::uint32_t& value) {
    std::uint64_t decoded = 0;
    if (!readUnsigned(4, decoded)) {
      return false;
    }
    value = static_cast<std::uint32_t>(decoded);
    return true;
  }

  bool readU64(std::uint64_t& value) {
    return readUnsigned(8, value);
  }

  bool readDouble(double& value) {
    std::uint64_t bits = 0;
    if (!readU64(bits)) {
      return false;
    }
    std::memcpy(&value, &bits, sizeof(value));
    return true;
  }

  bool readString(std::string& value, std::uint32_t max_size = 4096U) {
    std::uint32_t size = 0;
    if (!readU32(size) || size > max_size || !has(size)) {
      return false;
    }
    value.assign(reinterpret_cast<const char*>(bytes_ + offset_), size);
    offset_ += size;
    return true;
  }

  bool readBytes(std::vector<std::uint8_t>& value, std::uint32_t size) {
    if (!has(size)) {
      return false;
    }
    value.assign(bytes_ + offset_, bytes_ + offset_ + size);
    offset_ += size;
    return true;
  }

  std::size_t remaining() const {
    return size_ - offset_;
  }

 private:
  bool has(std::size_t requested) const {
    return requested <= size_ - offset_;
  }

  bool readUnsigned(std::size_t byte_count, std::uint64_t& value) {
    if (!has(byte_count)) {
      return false;
    }
    value = 0;
    for (std::size_t i = 0; i < byte_count; ++i) {
      value |= static_cast<std::uint64_t>(bytes_[offset_++]) << (8U * i);
    }
    return true;
  }

  const std::uint8_t* bytes_{nullptr};
  std::size_t size_{0};
  std::size_t offset_{0};
};

// 统一构造帧头，确保三路传感器流使用完全一致的字段顺序和 reserved 规则。
inline void appendFrameHeader(ByteWriter& writer,
                              StreamId stream,
                              std::uint64_t sequence,
                              std::uint64_t source_stamp_ns,
                              std::uint32_t payload_size) {
  writer.writeBytes(kMagic.data(), kMagic.size());
  writer.writeU16(kProtocolVersion);
  writer.writeU16(static_cast<std::uint16_t>(stream));
  writer.writeU64(sequence);
  writer.writeU64(source_stamp_ns);
  writer.writeU32(payload_size);
  writer.writeU32(0);
}

// 生成供 send(2) 使用的连续缓冲区。payload 过大时直接拒绝，
// 不允许截断帧头中的 uint32 长度。
inline std::vector<std::uint8_t> buildFrame(StreamId stream,
                                            std::uint64_t sequence,
                                            std::uint64_t source_stamp_ns,
                                            const std::vector<std::uint8_t>& payload) {
  if (payload.size() > std::numeric_limits<std::uint32_t>::max()) {
    return {};
  }
  ByteWriter writer(kFrameHeaderSize + payload.size());
  appendFrameHeader(writer, stream, sequence, source_stamp_ns,
                    static_cast<std::uint32_t>(payload.size()));
  writer.writeBytes(payload.data(), payload.size());
  return writer.take();
}

// 当前只接受 version 1 且 reserved 为零的帧。后续格式变更必须在
// 105 和 106 两端显式同步。
inline bool parseFrameHeader(const std::uint8_t* bytes,
                             std::size_t size,
                             FrameHeader& header) {
  if (bytes == nullptr || size != kFrameHeaderSize ||
      std::memcmp(bytes, kMagic.data(), kMagic.size()) != 0) {
    return false;
  }

  ByteReader reader(bytes + kMagic.size(), size - kMagic.size());
  std::uint16_t stream = 0;
  if (!reader.readU16(header.version) || !reader.readU16(stream) ||
      !reader.readU64(header.sequence) ||
      !reader.readU64(header.source_stamp_ns) ||
      !reader.readU32(header.payload_size) ||
      !reader.readU32(header.reserved) || reader.remaining() != 0) {
    return false;
  }
  header.stream = static_cast<StreamId>(stream);
  return header.version == kProtocolVersion && header.reserved == 0U;
}

}  // namespace x30_sensor_relay
