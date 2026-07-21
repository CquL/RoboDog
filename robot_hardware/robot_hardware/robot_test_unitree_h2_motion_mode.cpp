#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

int main(int argc, char *argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <network-interface>"
                  << std::endl;
        return 64;
    }

    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

        unitree::robot::b2::MotionSwitcherClient client;
        client.SetTimeout(3.0f);
        client.Init();

        std::string form;
        std::string name;
        const int32_t ret = client.CheckMode(form, name);

        std::cout << "H2_MOTION_SWITCHER_CHECK"
                  << " ret=" << ret
                  << " form=" << form
                  << " name=" << name << std::endl;

        if (ret != 0) {
            return 65;
        }

        // Read-only diagnostic. Never call SelectMode() or ReleaseMode().
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "H2_MOTION_SWITCHER_CHECK_EXCEPTION " << error.what()
                  << std::endl;
        return 66;
    } catch (...) {
        std::cerr << "H2_MOTION_SWITCHER_CHECK_UNKNOWN_EXCEPTION" << std::endl;
        return 67;
    }
}
