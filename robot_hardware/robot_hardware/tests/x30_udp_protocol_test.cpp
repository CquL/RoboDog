#include "deep_robotics/x30_udp_protocol.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>

int main()
{
    using namespace x30_udp_protocol;

    assert(physicalVelocityToMilli(0.2) == 200);
    assert(physicalVelocityToMilli(-0.1239) == -123);

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

    constexpr uint32_t remote_mountain = makeMotionCommandCode(
        kRemoteCommandPrefix, kGaitMountainSuffix);
    constexpr uint32_t navigation_mountain = makeMotionCommandCode(
        kNavigationCommandPrefix, kGaitMountainSuffix);
    constexpr uint32_t navigation_walk = makeMotionCommandCode(
        kNavigationCommandPrefix, kGaitWalkSuffix);
    constexpr uint32_t navigation_l_walk = makeMotionCommandCode(
        kNavigationCommandPrefix, kGaitLWalkSuffix);
    static_assert(remote_mountain == 0x21010421u);
    static_assert(navigation_mountain == 0x31010421u);
    static_assert(navigation_walk == 0x31010300u);
    static_assert(navigation_l_walk == 0x31010420u);

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
