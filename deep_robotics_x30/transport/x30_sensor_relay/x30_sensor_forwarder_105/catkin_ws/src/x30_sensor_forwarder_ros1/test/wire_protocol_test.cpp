// 写入端字节合同测试。使用固定预期偏移，确保字段顺序或字节序被意外修改时
// 能在部署前失败。
#include <array>
#include <cstdint>
#include <vector>

#include "x30_sensor_relay/wire_protocol.hpp"

int main() {
  const std::vector<std::uint8_t> payload = {0x10, 0x20, 0x30, 0x40};
  const auto frame = x30_sensor_relay::buildFrame(
      x30_sensor_relay::StreamId::kPointCloud, 42U, 123456789U, payload);

  if (frame.size() !=
      x30_sensor_relay::kFrameHeaderSize + payload.size()) {
    return 1;
  }

  // 预期字节按小端序编码 magic、version、stream、sequence、源时间戳、
  // payload 长度和全零 reserved 字段。
  std::array<std::uint8_t, x30_sensor_relay::kFrameHeaderSize>
      expected_header{};
  expected_header[0] = 'X';
  expected_header[1] = '3';
  expected_header[2] = '0';
  expected_header[3] = 'R';
  expected_header[4] = 1;
  expected_header[6] = 1;
  expected_header[8] = 42;
  expected_header[16] = 0x15;
  expected_header[17] = 0xcd;
  expected_header[18] = 0x5b;
  expected_header[19] = 0x07;
  expected_header[24] = 4;
  for (std::size_t index = 0; index < expected_header.size(); ++index) {
    if (frame[index] != expected_header[index]) {
      return 2;
    }
  }

  // 解析生成的帧头，确认写入端与共享解析器一致。
  x30_sensor_relay::FrameHeader header;
  if (!x30_sensor_relay::parseFrameHeader(
          frame.data(), x30_sensor_relay::kFrameHeaderSize, header) ||
      header.version != x30_sensor_relay::kProtocolVersion ||
      header.stream != x30_sensor_relay::StreamId::kPointCloud ||
      header.sequence != 42U ||
      header.source_stamp_ns != 123456789U ||
      header.payload_size != payload.size() ||
      header.reserved != 0U) {
    return 3;
  }

  for (std::size_t index = 0; index < payload.size(); ++index) {
    if (frame[x30_sensor_relay::kFrameHeaderSize + index] !=
        payload[index]) {
      return 4;
    }
  }

  return 0;
}
