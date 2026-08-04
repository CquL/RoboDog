#include "deep_robotics/x30_udp_protocol.h"

#include <array>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdint>
#include <iostream>

// 已恢复 X30 UDP 协议的离线字节契约测试。
// 测试不打开 socket、不加载机器人配置，也不与机器人通信。
int main()
{
    using namespace x30_udp_protocol;

    // Physical 速度遵循厂家发送端乘以 1000 后截断的规则。
    assert(physicalVelocityToMilli(0.2) == 200);
    assert(physicalVelocityToMilli(-0.1239) == -123);

    // 校验正负方向的完整 12 字节简单帧，
    // 包括 little-endian 二进制补码表示。
    const SimplePacket positive = makeSimplePacket(kPhysicalVxCode, 200);
    const SimplePacket expected_positive{
        0x23, 0x01, 0x00, 0x00,
        0xc8, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };
    assert(positive == expected_positive);

    const SimplePacket negative = makeSimplePacket(kPhysicalVxCode, -200);
    const SimplePacket expected_negative{
        0x23, 0x01, 0x00, 0x00,
        0x38, 0xff, 0xff, 0xff,
        0x00, 0x00, 0x00, 0x00,
    };
    assert(negative == expected_negative);

    // 前缀组合只能改变来源字节，并保留已恢复的 24-bit 步态后缀。
    constexpr uint32_t remote_mountain = makeMotionCommandCode(
        kRemoteCommandPrefix, kGaitMountainSuffix);
    constexpr uint32_t navigation_mountain = makeMotionCommandCode(
        kNavigationCommandPrefix, kGaitMountainSuffix);
    constexpr uint32_t navigation_walk = makeMotionCommandCode(
        kNavigationCommandPrefix, kGaitWalkSuffix);
    constexpr uint32_t remote_stairs = makeMotionCommandCode(
        kRemoteCommandPrefix, kGaitStairsSuffix);
    constexpr uint32_t navigation_stairs = makeMotionCommandCode(
        kNavigationCommandPrefix, kGaitStairsSuffix);
    constexpr uint32_t navigation_l_walk = makeMotionCommandCode(
        kNavigationCommandPrefix, kGaitLWalkSuffix);
    static_assert(remote_mountain == 0x21010421u);
    static_assert(navigation_mountain == 0x31010421u);
    static_assert(navigation_walk == 0x31010300u);
    static_assert(remote_stairs == 0x21010405u);
    static_assert(navigation_stairs == 0x31010405u);
    static_assert(navigation_l_walk == 0x31010420u);
    static_assert(kGaitStairsMaxVx == 0.3);
    static_assert(kGaitStairsMaxVy == 0.2);
    static_assert(kGaitStairsMaxOmega == 0.8);

    // 代表性步态帧固定线上命令字节和保留字段，
    // 不依赖 HAL 或 socket 行为。
    const SimplePacket mountain = makeSimplePacket(navigation_mountain, 0);
    const SimplePacket expected_mountain{
        0x21, 0x04, 0x01, 0x31,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };
    assert(mountain == expected_mountain);

    const SimplePacket l_walk = makeSimplePacket(navigation_l_walk, 0);
    const SimplePacket expected_l_walk{
        0x20, 0x04, 0x01, 0x31,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };
    assert(l_walk == expected_l_walk);

    const SimplePacket stairs = makeSimplePacket(navigation_stairs, 0);
    const SimplePacket expected_stairs{
        0x05, 0x04, 0x01, 0x31,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };
    assert(stairs == expected_stairs);

    // 命令 0x150 由 12 字节头部和三个 IEEE-754 float64 值组成。
    // 比较全部 36 字节可发现尺寸、类型、顺序和 endian 漂移。
    const NavigationVelocityPacket navigation =
        makeNavigationVelocityPacket(0.2, -0.1, 0.3);
    const NavigationVelocityPacket expected_navigation{
        0x50, 0x01, 0x00, 0x00,
        0x18, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x00, 0x00,
        0x9a, 0x99, 0x99, 0x99, 0x99, 0x99, 0xc9, 0x3f,
        0x9a, 0x99, 0x99, 0x99, 0x99, 0x99, 0xb9, 0xbf,
        0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0xd3, 0x3f,
    };
    assert(navigation == expected_navigation);

    std::cout << "X30 UDP protocol packet tests passed." << std::endl;
    return 0;
}
