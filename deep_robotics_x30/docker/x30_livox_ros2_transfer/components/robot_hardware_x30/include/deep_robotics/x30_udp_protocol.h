#ifndef X30_UDP_PROTOCOL_H
#define X30_UDP_PROTOCOL_H

// X30 厂商 UDP 协议的字节级辅助函数。
//
// 本头文件不依赖 socket 或机器人状态，确保报文编码确定且可在开发机直接测试。

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace x30_udp_protocol {

// 高字节选择原厂命令源，低 24 位后缀由遥控和导航动作命令共用。
constexpr uint32_t kRemoteCommandPrefix = 0x21000000;
constexpr uint32_t kNavigationCommandPrefix = 0x31000000;

// 直接物理速度使用三条独立的 12 字节命令，值为带符号 SI 单位乘 1000。
// 导航速度使用一条 36 字节命令，以 float64 保存三个 SI 单位分量。
constexpr uint32_t kPhysicalVxCode = 0x00000123;
constexpr uint32_t kPhysicalVyCode = 0x00000124;
constexpr uint32_t kPhysicalYawCode = 0x00000122;
constexpr uint32_t kNavigationVelocityCode = 0x00000150;

// 步态常量是后缀而非完整命令码，运行时由 makeMotionCommandCode 添加命令源前缀。
constexpr uint32_t kGaitWalkSuffix = 0x00010300;
constexpr uint32_t kGaitStairsSuffix = 0x00010405;
constexpr uint32_t kGaitLWalkSuffix = 0x00010420;
constexpr uint32_t kGaitMountainSuffix = 0x00010421;

// 普通楼梯的说明书限幅比 X30 通用项目限幅更严格；HAL 仅在发送该步态请求后启用。
constexpr double kGaitStairsMaxVx = 0.3;
constexpr double kGaitStairsMaxVy = 0.2;
constexpr double kGaitStairsMaxOmega = 0.8;

// 简单报文布局：
//   [0..3] 命令 u32，[4..7] 数值 i32，[8..11] 保留 u32
// 导航速度报文布局：
//   [0..11] 命令/数据长度/类型头，[12..35] vx/vy/omega float64
// 所有多字节字段均为小端序，与原厂发送端一致。
constexpr std::size_t kCommandHeaderSize = 12;
constexpr std::size_t kNavigationVelocityDataSize = 24;
constexpr std::size_t kNavigationVelocityPacketSize =
    kCommandHeaderSize + kNavigationVelocityDataSize;

using SimplePacket = std::array<uint8_t, kCommandHeaderSize>;
using NavigationVelocityPacket =
    std::array<uint8_t, kNavigationVelocityPacketSize>;

// 掩码防止异常后缀覆盖已选择的命令源字节。
constexpr uint32_t makeMotionCommandCode(uint32_t prefix, uint32_t suffix)
{
    return (prefix & 0xff000000u) | (suffix & 0x00ffffffu);
}

// 显式逐字节写入，避免同一源码在 x86_64 开发机和 X30 上构建时依赖主机字节序
// 或内存对齐。
inline void writeUint32LittleEndian(uint8_t *destination, uint32_t value)
{
    destination[0] = static_cast<uint8_t>(value & 0xffu);
    destination[1] = static_cast<uint8_t>((value >> 8u) & 0xffu);
    destination[2] = static_cast<uint8_t>((value >> 16u) & 0xffu);
    destination[3] = static_cast<uint8_t>((value >> 24u) & 0xffu);
}

inline void writeUint64LittleEndian(uint8_t *destination, uint64_t value)
{
    for (std::size_t index = 0; index < sizeof(value); ++index) {
        destination[index] = static_cast<uint8_t>((value >> (index * 8u)) & 0xffu);
    }
}

// 序列化 double 前保留其精确 IEEE-754 位模式。
inline void writeDoubleLittleEndian(uint8_t *destination, double value)
{
    static_assert(sizeof(double) == sizeof(uint64_t),
                  "X30 navigation velocity packets require 64-bit doubles.");

    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    writeUint64LittleEndian(destination, bits);
}

// 构造动作、控制轴和物理速度共用的 12 字节命令帧。
inline SimplePacket makeSimplePacket(uint32_t code, int32_t value)
{
    SimplePacket packet{};
    writeUint32LittleEndian(packet.data(), code);
    writeUint32LittleEndian(packet.data() + 4,
                            static_cast<uint32_t>(value));
    writeUint32LittleEndian(packet.data() + 8, 0u);
    return packet;
}

// 对齐原厂发送端转换方式，包括向零截断以及转为 int32 前的饱和处理。
inline int32_t physicalVelocityToMilli(double velocity)
{
    if (!std::isfinite(velocity)) {
        return 0;
    }

    const double scaled = velocity * 1000.0;
    if (scaled >= static_cast<double>(std::numeric_limits<int32_t>::max())) {
        return std::numeric_limits<int32_t>::max();
    }
    if (scaled <= static_cast<double>(std::numeric_limits<int32_t>::min())) {
        return std::numeric_limits<int32_t>::min();
    }

    // 原厂 udp_sender 使用 cvttsd2si，即向零截断。
    return static_cast<int32_t>(scaled);
}

// 为感知主机的导航速度输入构造命令 0x150。
inline NavigationVelocityPacket makeNavigationVelocityPacket(
    double vx,
    double vy,
    double omega)
{
    NavigationVelocityPacket packet{};
    writeUint32LittleEndian(packet.data(), kNavigationVelocityCode);
    writeUint32LittleEndian(packet.data() + 4,
                            static_cast<uint32_t>(kNavigationVelocityDataSize));
    writeUint32LittleEndian(packet.data() + 8, 1u);
    writeDoubleLittleEndian(packet.data() + 12, vx);
    writeDoubleLittleEndian(packet.data() + 20, vy);
    writeDoubleLittleEndian(packet.data() + 28, omega);
    return packet;
}

} // namespace x30_udp_protocol

#endif
