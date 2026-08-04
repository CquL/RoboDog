// 接收端字节合同测试。使用固定字节模拟 105 写入端发出的帧，
// 不依赖 ROS、socket 或运行中的容器。
#include <array>
#include <cstdint>

#include "x30_sensor_relay/wire_protocol_reader.hpp"

int main() {
  // 合法小端序 IMU 帧头：sequence 7、源时间戳 42 ns、payload 16 字节。
  std::array<std::uint8_t, x30_sensor_relay::kFrameHeaderSize> bytes{};
  bytes[0] = 'X';
  bytes[1] = '3';
  bytes[2] = '0';
  bytes[3] = 'R';
  bytes[4] = 1;
  bytes[6] = 2;
  bytes[8] = 7;
  bytes[16] = 42;
  bytes[24] = 16;

  x30_sensor_relay::FrameHeader header;
  if (!x30_sensor_relay::parseFrameHeader(bytes.data(), bytes.size(),
                                           header) ||
      header.version != 1 ||
      header.stream != x30_sensor_relay::StreamId::kImu ||
      header.sequence != 7 ||
      header.source_stamp_ns != 42 ||
      header.payload_size != 16) {
    return 1;
  }

  // reserved 字节用于保护格式扩展，在 v1 中必须保持为零。
  bytes[28] = 1;
  if (x30_sensor_relay::parseFrameHeader(bytes.data(), bytes.size(),
                                          header)) {
    return 2;
  }
  bytes[28] = 0;

  // 错误 magic 不能被识别为转发帧。
  bytes[0] = 'B';
  if (x30_sensor_relay::parseFrameHeader(bytes.data(), bytes.size(),
                                          header)) {
    return 3;
  }
  return 0;
}
