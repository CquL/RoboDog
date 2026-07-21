#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/MainBoardState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace {

using BmsState = unitree_hg::msg::dds_::BmsState_;
using ImuState = unitree_hg::msg::dds_::IMUState_;
using LowState = unitree_hg::msg::dds_::LowState_;
using MainBoardState = unitree_hg::msg::dds_::MainBoardState_;
using unitree::robot::ChannelSubscriber;

using SteadyClock = std::chrono::steady_clock;

std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             SteadyClock::now().time_since_epoch())
      .count();
}

struct TopicStats {
  explicit TopicStats(std::string channel_name)
      : channel(std::move(channel_name)) {}

  std::string channel;
  std::atomic<std::uint64_t> samples{0};
  std::atomic<std::int64_t> first_ns{0};
  std::atomic<std::int64_t> last_ns{0};
  std::atomic<std::uint64_t> crc_ok{0};
  std::atomic<std::uint64_t> crc_fail{0};
  std::mutex sample_mutex;
  std::string first_sample;
  std::string last_sample;
};

std::uint64_t MarkSample(TopicStats &stats) {
  const auto now = NowNs();
  const auto sequence = stats.samples.fetch_add(1) + 1;
  if (sequence == 1) {
    stats.first_ns.store(now);
  }
  stats.last_ns.store(now);
  return sequence;
}

void SaveSample(TopicStats &stats, std::uint64_t sequence,
                const std::string &sample) {
  std::lock_guard<std::mutex> lock(stats.sample_mutex);
  if (sequence == 1) {
    stats.first_sample = sample;
  }
  stats.last_sample = sample;
}

bool ShouldFormat(std::uint64_t sequence) {
  return sequence == 1 || (sequence % 100 == 0);
}

std::uint32_t Crc32Core(const std::uint32_t *words, std::uint32_t length) {
  constexpr std::uint32_t polynomial = 0x04c11db7;
  std::uint32_t crc = 0xffffffff;
  for (std::uint32_t i = 0; i < length; ++i) {
    std::uint32_t value = words[i];
    for (std::uint32_t bit = 0; bit < 32; ++bit) {
      const bool top_bit = (crc & 0x80000000U) != 0;
      crc <<= 1U;
      if ((value & 0x80000000U) != 0) {
        crc ^= polynomial;
      }
      if (top_bit) {
        crc ^= polynomial;
      }
      value <<= 1U;
    }
  }
  return crc;
}

std::string FormatImu(const ImuState &message) {
  const auto &q = message.quaternion();
  const auto &gyro = message.gyroscope();
  const auto &acc = message.accelerometer();
  const auto &rpy = message.rpy();
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "q=[" << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3]
      << "] gyro=[" << gyro[0] << ',' << gyro[1] << ',' << gyro[2]
      << "] acc=[" << acc[0] << ',' << acc[1] << ',' << acc[2]
      << "] rpy=[" << rpy[0] << ',' << rpy[1] << ',' << rpy[2]
      << "] temperature_raw=" << message.temperature();
  return out.str();
}

void HandleLowState(const void *data, TopicStats &stats) {
  const auto &message = *static_cast<const LowState *>(data);
  const auto sequence = MarkSample(stats);

  static_assert(sizeof(LowState) % sizeof(std::uint32_t) == 0,
                "LowState layout must be word aligned for vendor CRC");
  std::array<std::uint32_t,
             sizeof(LowState) / sizeof(std::uint32_t)> words{};
  std::memcpy(words.data(), &message, sizeof(message));
  const bool crc_valid =
      Crc32Core(words.data(), static_cast<std::uint32_t>(words.size() - 1U)) ==
      message.crc();
  if (crc_valid) {
    stats.crc_ok.fetch_add(1);
  } else {
    stats.crc_fail.fetch_add(1);
  }

  if (!ShouldFormat(sequence)) {
    return;
  }

  const auto &motor0 = message.motor_state()[0];
  std::ostringstream out;
  out << "tick=" << message.tick()
      << " mode_pr=" << static_cast<unsigned int>(message.mode_pr())
      << " mode_machine=" << static_cast<unsigned int>(message.mode_machine())
      << " crc=0x" << std::hex << message.crc() << std::dec
      << " crc_valid=" << (crc_valid ? 1 : 0)
      << " motor0_q=" << motor0.q()
      << " motor0_dq=" << motor0.dq()
      << " motor0_tau_est=" << motor0.tau_est()
      << " motor0_state=" << motor0.motorstate()
      << " pelvis_" << FormatImu(message.imu_state());
  SaveSample(stats, sequence, out.str());
}

void HandleImu(const void *data, TopicStats &stats) {
  const auto &message = *static_cast<const ImuState *>(data);
  const auto sequence = MarkSample(stats);
  if (ShouldFormat(sequence)) {
    SaveSample(stats, sequence, FormatImu(message));
  }
}

void HandleBms(const void *data, TopicStats &stats) {
  const auto &message = *static_cast<const BmsState *>(data);
  const auto sequence = MarkSample(stats);
  if (!ShouldFormat(sequence)) {
    return;
  }
  const auto &voltage = message.bmsvoltage();
  const auto &temperature = message.temperature();
  std::ostringstream out;
  out << "soc_raw=" << static_cast<unsigned int>(message.soc())
      << " soh_raw=" << static_cast<unsigned int>(message.soh())
      << " current_raw=" << message.current()
      << " voltage_raw=[" << voltage[0] << ',' << voltage[1] << ','
      << voltage[2] << "] temperature0_raw=" << temperature[0]
      << " cycle=" << message.cycle();
  SaveSample(stats, sequence, out.str());
}

void HandleMainBoard(const void *data, TopicStats &stats) {
  const auto &message = *static_cast<const MainBoardState *>(data);
  const auto sequence = MarkSample(stats);
  if (!ShouldFormat(sequence)) {
    return;
  }
  const auto &fan = message.fan_state();
  const auto &temperature = message.temperature();
  const auto &value = message.value();
  const auto &state = message.state();
  std::ostringstream out;
  out << "fan0_raw=" << fan[0]
      << " temperature0_raw=" << temperature[0]
      << " value0_raw=" << value[0]
      << " state0_raw=" << state[0];
  SaveSample(stats, sequence, out.str());
}

void PrintStats(TopicStats &stats) {
  const auto count = stats.samples.load();
  const auto first = stats.first_ns.load();
  const auto last = stats.last_ns.load();
  double rate_hz = 0.0;
  if (count > 1 && last > first) {
    rate_hz = static_cast<double>(count - 1) * 1.0e9 /
              static_cast<double>(last - first);
  }

  std::string first_sample;
  std::string last_sample;
  {
    std::lock_guard<std::mutex> lock(stats.sample_mutex);
    first_sample = stats.first_sample;
    last_sample = stats.last_sample;
  }

  std::cout << "TOPIC channel=" << stats.channel << " samples=" << count
            << " rate_hz=" << std::fixed << std::setprecision(3) << rate_hz
            << " crc_ok=" << stats.crc_ok.load()
            << " crc_fail=" << stats.crc_fail.load() << '\n';
  if (!first_sample.empty()) {
    std::cout << "FIRST channel=" << stats.channel << ' ' << first_sample << '\n';
  }
  if (!last_sample.empty()) {
    std::cout << "LAST channel=" << stats.channel << ' ' << last_sample << '\n';
  }
}

int ParseInteger(const std::string &value, const std::string &name) {
  std::size_t used = 0;
  const int parsed = std::stoi(value, &used);
  if (used != value.size()) {
    throw std::invalid_argument("invalid " + name + ": " + value);
  }
  return parsed;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    std::string interface_name = "eth0";
    int domain_id = 0;
    int duration_seconds = 15;

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--interface" && i + 1 < argc) {
        interface_name = argv[++i];
      } else if (arg == "--domain" && i + 1 < argc) {
        domain_id = ParseInteger(argv[++i], "domain");
      } else if (arg == "--seconds" && i + 1 < argc) {
        duration_seconds = ParseInteger(argv[++i], "seconds");
      } else {
        throw std::invalid_argument("unknown or incomplete argument: " + arg);
      }
    }

    if (interface_name.empty() || domain_id < 0 || domain_id > 232 ||
        duration_seconds < 1 || duration_seconds > 60) {
      throw std::invalid_argument("argument outside the accepted safety bounds");
    }

    std::cout << "PROBE_MODE=SUBSCRIBE_ONLY\n"
              << "DOMAIN_ID=" << domain_id << '\n'
              << "INTERFACE=" << interface_name << '\n'
              << "DURATION_SECONDS=" << duration_seconds << '\n'
              << "SPORTMODE_STATE=OMITTED_PC2_HG_HEADER_MISSING\n";

    unitree::robot::ChannelFactory::Instance()->Init(domain_id, interface_name);

    TopicStats lowstate_raw_stats("rt/lowstate_raw");
    TopicStats lowstate_stats("rt/lowstate");
    TopicStats lf_lowstate_stats("rt/lf/lowstate");
    TopicStats secondary_imu_stats("rt/secondary_imu");
    TopicStats lf_secondary_imu_stats("rt/lf/secondary_imu");
    TopicStats bms_stats("rt/lf/bmsstate");
    TopicStats mainboard_stats("rt/lf/mainboardstate");

    auto lowstate_raw = std::make_shared<ChannelSubscriber<LowState>>(
        lowstate_raw_stats.channel);
    auto lowstate = std::make_shared<ChannelSubscriber<LowState>>(
        lowstate_stats.channel);
    auto lf_lowstate = std::make_shared<ChannelSubscriber<LowState>>(
        lf_lowstate_stats.channel);
    auto secondary_imu = std::make_shared<ChannelSubscriber<ImuState>>(
        secondary_imu_stats.channel);
    auto lf_secondary_imu = std::make_shared<ChannelSubscriber<ImuState>>(
        lf_secondary_imu_stats.channel);
    auto bms = std::make_shared<ChannelSubscriber<BmsState>>(bms_stats.channel);
    auto mainboard = std::make_shared<ChannelSubscriber<MainBoardState>>(
        mainboard_stats.channel);

    lowstate_raw->InitChannel(
        [&lowstate_raw_stats](const void *data) {
          HandleLowState(data, lowstate_raw_stats);
        },
        1);
    lowstate->InitChannel(
        [&lowstate_stats](const void *data) {
          HandleLowState(data, lowstate_stats);
        },
        1);
    lf_lowstate->InitChannel(
        [&lf_lowstate_stats](const void *data) {
          HandleLowState(data, lf_lowstate_stats);
        },
        1);
    secondary_imu->InitChannel(
        [&secondary_imu_stats](const void *data) {
          HandleImu(data, secondary_imu_stats);
        },
        1);
    lf_secondary_imu->InitChannel(
        [&lf_secondary_imu_stats](const void *data) {
          HandleImu(data, lf_secondary_imu_stats);
        },
        1);
    bms->InitChannel(
        [&bms_stats](const void *data) { HandleBms(data, bms_stats); }, 1);
    mainboard->InitChannel(
        [&mainboard_stats](const void *data) {
          HandleMainBoard(data, mainboard_stats);
        },
        1);

    std::this_thread::sleep_for(std::chrono::seconds(duration_seconds));

    lowstate_raw->CloseChannel();
    lowstate->CloseChannel();
    lf_lowstate->CloseChannel();
    secondary_imu->CloseChannel();
    lf_secondary_imu->CloseChannel();
    bms->CloseChannel();
    mainboard->CloseChannel();

    PrintStats(lowstate_raw_stats);
    PrintStats(lowstate_stats);
    PrintStats(lf_lowstate_stats);
    PrintStats(secondary_imu_stats);
    PrintStats(lf_secondary_imu_stats);
    PrintStats(bms_stats);
    PrintStats(mainboard_stats);

    unitree::robot::ChannelFactory::Instance()->Release();

    const bool have_lowstate =
        lowstate_raw_stats.samples.load() > 0 ||
        lowstate_stats.samples.load() > 0 ||
        lf_lowstate_stats.samples.load() > 0;
    const bool have_imu = secondary_imu_stats.samples.load() > 0 ||
                          lf_secondary_imu_stats.samples.load() > 0;
    const bool have_bms = bms_stats.samples.load() > 0;
    const bool have_mainboard = mainboard_stats.samples.load() > 0;

    if (have_lowstate && have_imu && have_bms && have_mainboard) {
      std::cout << "H2_HG_SUBSCRIBE_ONLY_PROBE_OK\n";
      return EXIT_SUCCESS;
    }

    std::cout << "H2_HG_SUBSCRIBE_ONLY_PROBE_INCOMPLETE\n";
    return 3;
  } catch (const std::exception &error) {
    std::cerr << "PROBE_EXCEPTION=" << error.what() << '\n';
    return 2;
  }
}
