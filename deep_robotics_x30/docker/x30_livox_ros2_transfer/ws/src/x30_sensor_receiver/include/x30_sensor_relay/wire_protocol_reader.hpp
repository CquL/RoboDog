#pragma once

// 105 上的 ROS1 转发端与 106 上的 ROS2 接收端共用此小端序帧合同。
// 固定 32 字节帧头如下：
//   magic[4], version:u16, stream:u16, sequence:u64, source_stamp_ns:u64,
//   payload_size:u32, reserved:u32.
// payload 会跨越主机与容器信任边界，因此所有读取都进行边界检查。
// 本协议只传输复制的传感器数据。
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace x30_sensor_relay {

constexpr std::array<std::uint8_t, 4> kMagic = {{'X', '3', '0', 'R'}};
constexpr std::uint16_t kProtocolVersion = 1;
constexpr std::size_t kFrameHeaderSize = 32;

enum class StreamId : std::uint16_t {
  kPointCloud = 1,
  kImu = 2,
  kOdometry = 3,
};

struct FrameHeader {
  // sequence 在各数据流内独立计数，仅用于观测，不用于消息计时。
  // source_stamp_ns 用于重建 ROS2 header 时间戳。
  std::uint16_t version{0};
  StreamId stream{StreamId::kPointCloud};
  std::uint64_t sequence{0};
  std::uint64_t source_stamp_ns{0};
  std::uint32_t payload_size{0};
  std::uint32_t reserved{0};
};

// 用游标解码不可信的帧与 payload 字节。每次移动游标前都检查剩余容量。
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

// 拒绝未知 magic/version 和非零 reserved 字段。对应 TCP 监听器还会校验
// stream 身份与 payload 长度上限。
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
