#ifndef X30_UDP_PROTOCOL_H
#define X30_UDP_PROTOCOL_H

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace x30_udp_protocol {

constexpr uint32_t kRemoteCommandPrefix = 0x21000000;
constexpr uint32_t kNavigationCommandPrefix = 0x31000000;

constexpr uint32_t kPhysicalVxCode = 0x00000123;
constexpr uint32_t kPhysicalVyCode = 0x00000124;
constexpr uint32_t kPhysicalYawCode = 0x00000122;
constexpr uint32_t kNavigationVelocityCode = 0x00000150;

constexpr uint32_t kGaitWalkSuffix = 0x00010300;
constexpr uint32_t kGaitLWalkSuffix = 0x00010420;
constexpr uint32_t kGaitMountainSuffix = 0x00010421;

constexpr std::size_t kCommandHeaderSize = 12;
constexpr std::size_t kNavigationVelocityDataSize = 24;
constexpr std::size_t kNavigationVelocityPacketSize =
    kCommandHeaderSize + kNavigationVelocityDataSize;

using SimplePacket = std::array<uint8_t, kCommandHeaderSize>;
using NavigationVelocityPacket =
    std::array<uint8_t, kNavigationVelocityPacketSize>;

constexpr uint32_t makeMotionCommandCode(uint32_t prefix, uint32_t suffix)
{
    return (prefix & 0xff000000u) | (suffix & 0x00ffffffu);
}

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

inline void writeDoubleLittleEndian(uint8_t *destination, double value)
{
    static_assert(sizeof(double) == sizeof(uint64_t),
                  "X30 navigation velocity packets require 64-bit doubles.");

    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    writeUint64LittleEndian(destination, bits);
}

inline SimplePacket makeSimplePacket(uint32_t code, int32_t value)
{
    SimplePacket packet{};
    writeUint32LittleEndian(packet.data(), code);
    writeUint32LittleEndian(packet.data() + 4,
                            static_cast<uint32_t>(value));
    writeUint32LittleEndian(packet.data() + 8, 0u);
    return packet;
}

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

    // The factory udp_sender uses cvttsd2si, which truncates toward zero.
    return static_cast<int32_t>(scaled);
}

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
