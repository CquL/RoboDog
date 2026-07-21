#ifndef UNITREE_H2_TEST_FAKE_LOCO_CLIENT_HPP
#define UNITREE_H2_TEST_FAKE_LOCO_CLIENT_HPP

#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace unitree {
namespace robot {
namespace test {

struct H2LocoRecorder {
    inline static int domain_id = -1;
    inline static std::string interface_name;
    inline static std::atomic<int> channel_init_calls{0};
    inline static std::atomic<int> loco_init_calls{0};
    inline static std::atomic<int> timeout_calls{0};
    inline static std::atomic<int> get_fsm_calls{0};
    inline static std::atomic<int> get_fsm_mode_calls{0};
    inline static std::atomic<int> get_available_fsm_calls{0};
    inline static std::atomic<int> set_velocity_calls{0};
    inline static std::atomic<int> stop_move_attempts{0};
    inline static std::atomic<int> stop_move_calls{0};
    inline static std::atomic<int> stand_up_calls{0};
    inline static std::atomic<int> start_calls{0};
    inline static std::atomic<int> damp_calls{0};
    inline static std::atomic<int> squat_calls{0};
    inline static std::atomic<int> sit_calls{0};

    inline static std::atomic<float> last_timeout_s{0.0f};
    inline static std::atomic<float> last_vx{0.0f};
    inline static std::atomic<float> last_vy{0.0f};
    inline static std::atomic<float> last_omega{0.0f};
    inline static std::atomic<float> last_duration_s{0.0f};

    inline static std::atomic<int32_t> get_fsm_result{0};
    inline static std::atomic<int> fsm_id{601};
    inline static std::atomic<int> fsm_mode{2};
    inline static std::atomic<int32_t> get_fsm_mode_result{0};
    inline static std::atomic<int32_t> get_available_fsm_result{0};
    inline static std::vector<int> available_fsm_ids{1, 2, 3, 4, 601};
    inline static std::vector<std::string> available_fsm_names{
        "damp", "squat", "sit", "stand_up", "start"};
    inline static std::atomic<int32_t> set_velocity_result{0};
    inline static std::atomic<int32_t> stop_move_result{0};
    inline static std::atomic<int32_t> stand_up_result{0};
    inline static std::atomic<int32_t> start_result{0};
    inline static std::atomic<int32_t> damp_result{0};
    inline static std::atomic<int32_t> squat_result{0};
    inline static std::atomic<int32_t> sit_result{0};

    inline static std::atomic<bool> throw_on_channel_init{false};
    inline static std::atomic<bool> throw_on_loco_init{false};
    inline static std::atomic<bool> throw_on_set_timeout{false};
    inline static std::atomic<bool> throw_on_get_fsm{false};
    inline static std::atomic<bool> throw_on_get_fsm_mode{false};
    inline static std::atomic<bool> throw_on_get_available_fsm{false};
    inline static std::atomic<bool> throw_on_set_velocity{false};
    inline static std::atomic<bool> throw_on_stop_move{false};
    inline static std::atomic<bool> throw_on_action{false};

    static void Reset()
    {
        domain_id = -1;
        interface_name.clear();
        channel_init_calls = 0;
        loco_init_calls = 0;
        timeout_calls = 0;
        get_fsm_calls = 0;
        get_fsm_mode_calls = 0;
        get_available_fsm_calls = 0;
        set_velocity_calls = 0;
        stop_move_attempts = 0;
        stop_move_calls = 0;
        stand_up_calls = 0;
        start_calls = 0;
        damp_calls = 0;
        squat_calls = 0;
        sit_calls = 0;
        last_timeout_s = 0.0f;
        last_vx = 0.0f;
        last_vy = 0.0f;
        last_omega = 0.0f;
        last_duration_s = 0.0f;
        get_fsm_result = 0;
        fsm_id = 601;
        fsm_mode = 2;
        get_fsm_mode_result = 0;
        get_available_fsm_result = 0;
        available_fsm_ids = {1, 2, 3, 4, 601};
        available_fsm_names = {"damp", "squat", "sit", "stand_up", "start"};
        set_velocity_result = 0;
        stop_move_result = 0;
        stand_up_result = 0;
        start_result = 0;
        damp_result = 0;
        squat_result = 0;
        sit_result = 0;
        throw_on_channel_init = false;
        throw_on_loco_init = false;
        throw_on_set_timeout = false;
        throw_on_get_fsm = false;
        throw_on_get_fsm_mode = false;
        throw_on_get_available_fsm = false;
        throw_on_set_velocity = false;
        throw_on_stop_move = false;
        throw_on_action = false;
    }
};

} // namespace test

class ChannelFactory {
public:
    static ChannelFactory *Instance()
    {
        static ChannelFactory instance;
        return &instance;
    }

    void Init(int domain_id, const std::string &interface_name)
    {
        if (test::H2LocoRecorder::throw_on_channel_init) {
            throw std::runtime_error("fake ChannelFactory initialization failure");
        }
        test::H2LocoRecorder::domain_id = domain_id;
        test::H2LocoRecorder::interface_name = interface_name;
        ++test::H2LocoRecorder::channel_init_calls;
    }
};

namespace h2 {

class LocoClient {
public:
    void Init()
    {
        if (test::H2LocoRecorder::throw_on_loco_init) {
            throw std::runtime_error("fake LocoClient initialization failure");
        }
        ++test::H2LocoRecorder::loco_init_calls;
    }

    void SetTimeout(float timeout_s)
    {
        if (test::H2LocoRecorder::throw_on_set_timeout) {
            throw std::runtime_error("fake SetTimeout failure");
        }
        test::H2LocoRecorder::last_timeout_s = timeout_s;
        ++test::H2LocoRecorder::timeout_calls;
    }

    int32_t GetFsmId(int &fsm_id)
    {
        if (test::H2LocoRecorder::throw_on_get_fsm) {
            throw std::runtime_error("fake GetFsmId failure");
        }
        ++test::H2LocoRecorder::get_fsm_calls;
        fsm_id = test::H2LocoRecorder::fsm_id;
        return test::H2LocoRecorder::get_fsm_result;
    }

    int32_t GetFsmMode(int &fsm_mode)
    {
        if (test::H2LocoRecorder::throw_on_get_fsm_mode) {
            throw std::runtime_error("fake GetFsmMode failure");
        }
        ++test::H2LocoRecorder::get_fsm_mode_calls;
        fsm_mode = test::H2LocoRecorder::fsm_mode;
        return test::H2LocoRecorder::get_fsm_mode_result;
    }

    int32_t GetAvailableFsmIds(std::vector<int> &ids,
                               std::vector<std::string> &names)
    {
        if (test::H2LocoRecorder::throw_on_get_available_fsm) {
            throw std::runtime_error("fake GetAvailableFsmIds failure");
        }
        ++test::H2LocoRecorder::get_available_fsm_calls;
        ids = test::H2LocoRecorder::available_fsm_ids;
        names = test::H2LocoRecorder::available_fsm_names;
        return test::H2LocoRecorder::get_available_fsm_result;
    }

    int32_t SetVelocity(float vx, float vy, float omega, float duration_s)
    {
        test::H2LocoRecorder::last_vx = vx;
        test::H2LocoRecorder::last_vy = vy;
        test::H2LocoRecorder::last_omega = omega;
        test::H2LocoRecorder::last_duration_s = duration_s;
        ++test::H2LocoRecorder::set_velocity_calls;
        // Record before throwing to model the safety-critical ambiguous case:
        // the robot may have accepted the request even though the client did
        // not receive a successful response.
        if (test::H2LocoRecorder::throw_on_set_velocity) {
            throw std::runtime_error("fake SetVelocity post-delivery failure");
        }
        return test::H2LocoRecorder::set_velocity_result;
    }

    int32_t StopMove()
    {
        ++test::H2LocoRecorder::stop_move_attempts;
        if (test::H2LocoRecorder::throw_on_stop_move) {
            throw std::runtime_error("fake StopMove failure");
        }
        ++test::H2LocoRecorder::stop_move_calls;
        return test::H2LocoRecorder::stop_move_result;
    }

    int32_t StandUp() { return RecordAction(test::H2LocoRecorder::stand_up_calls,
                                            test::H2LocoRecorder::stand_up_result); }
    int32_t Start() { return RecordAction(test::H2LocoRecorder::start_calls,
                                          test::H2LocoRecorder::start_result); }
    int32_t Damp() { return RecordAction(test::H2LocoRecorder::damp_calls,
                                         test::H2LocoRecorder::damp_result); }
    int32_t Squat() { return RecordAction(test::H2LocoRecorder::squat_calls,
                                          test::H2LocoRecorder::squat_result); }
    int32_t Sit() { return RecordAction(test::H2LocoRecorder::sit_calls,
                                        test::H2LocoRecorder::sit_result); }

private:
    static int32_t RecordAction(std::atomic<int> &calls,
                                const std::atomic<int32_t> &result)
    {
        if (test::H2LocoRecorder::throw_on_action) {
            throw std::runtime_error("fake action failure");
        }
        ++calls;
        return result.load();
    }
};

} // namespace h2
} // namespace robot
} // namespace unitree

#endif
