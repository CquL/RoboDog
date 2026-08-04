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

// H2 HG 原生 DDS 状态只读探针。
//
// 数据流：
//   PC1/HG 状态发布者 -> SDK2 ChannelSubscriber -> 频率/CRC/首末样本统计
//   -> stdout（由外层脚本保存为日志）。
//
// 安全边界：本程序不包含 DDS 发送端、运动控制客户端、请求消息或任何
// 控制话题；只在有界时间内接收状态，随后关闭所有订阅并释放 DDS。
namespace {

using BmsState = unitree_hg::msg::dds_::BmsState_;
using ImuState = unitree_hg::msg::dds_::IMUState_;
using LowState = unitree_hg::msg::dds_::LowState_;
using MainBoardState = unitree_hg::msg::dds_::MainBoardState_;
using unitree::robot::ChannelSubscriber;

using SteadyClock = std::chrono::steady_clock;

// 使用单调时钟计算样本频率，不受系统时间校准或 NTP 跳变影响。
std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             SteadyClock::now().time_since_epoch())
      .count();
}

// 每个 DDS 话题的线程安全统计：
// - 原子字段供高频回调更新计数/时间/CRC；
// - 互斥锁只保护较低频率格式化的首末样本文本。
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

// 记录一帧到达并返回从 1 开始的本地序号。
std::uint64_t MarkSample(TopicStats &stats) {
  const auto now = NowNs();
  const auto sequence = stats.samples.fetch_add(1) + 1;
  if (sequence == 1) {
    stats.first_ns.store(now);
  }
  stats.last_ns.store(now);
  return sequence;
}

// 保存第一帧和最近一次被格式化的样本；不持有原厂消息对象的引用。
void SaveSample(TopicStats &stats, std::uint64_t sequence,
                const std::string &sample) {
  std::lock_guard<std::mutex> lock(stats.sample_mutex);
  if (sequence == 1) {
    stats.first_sample = sample;
  }
  stats.last_sample = sample;
}

// 只格式化首帧和每 100 帧，降低高频 LowState/IMU 对 CPU 和日志的影响。
bool ShouldFormat(std::uint64_t sequence) {
  return sequence == 1 || (sequence % 100 == 0);
}

// 按宇树 LowState 契约实现 32 位字 CRC 核心；只用于校验接收数据完整性，
// 不修改消息，也不向 DDS 回写校验结果。
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

// 把 IMU 关键原始字段格式化为单行日志；不做单位换算或坐标变换，
// `_raw` 后缀提醒维护者这些值仍需以厂商 IDL/文档解释。
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

// LowState 回调：计数、验证末尾 CRC，并抽取 motor0 与骨盆 IMU 作为样例。
// 完整消息只在回调栈内读取，不缓存指针。
void HandleLowState(const void *data, TopicStats &stats) {
  const auto &message = *static_cast<const LowState *>(data);
  const auto sequence = MarkSample(stats);

  // 厂商 CRC 覆盖除最后 crc 字段外的所有 32 位字；先 memcpy 到对齐数组，
  // 避免直接把 DDS 对象按 uint32_t 指针读取造成未对齐访问。
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

  // 仅输出第一个电机作为链路活性样例，不把它解释为控制目标或故障结论。
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

// 独立 secondary_imu 回调：高频只计数，按采样策略保存人可读快照。
void HandleImu(const void *data, TopicStats &stats) {
  const auto &message = *static_cast<const ImuState *>(data);
  const auto sequence = MarkSample(stats);
  if (ShouldFormat(sequence)) {
    SaveSample(stats, sequence, FormatImu(message));
  }
}

// BMS 回调：记录 SOC/SOH、电流、电压、温度和循环次数的原始值。
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

// MainBoard 回调：记录首元素作为状态流样例；不对数组语义作未经证实推断。
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

// 计算观察窗口内平均接收频率并输出稳定 TOPIC/FIRST/LAST 日志行。
// count 小于 2 时保持 0 Hz，避免除零或制造无依据频率。
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

// 严格解析整数字符串；存在尾随字符即抛异常，由 main 统一返回错误码 2。
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
    // 默认绑定 H2 PC2 的 eth0、DDS Domain 0，观察 15 秒。
    std::string interface_name = "eth0";
    int domain_id = 0;
    int duration_seconds = 15;

    // 仅接受显式的网卡、Domain 和持续时间参数，未知/不完整参数立即拒绝。
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

    // Domain 遵循 DDS 有效范围；持续时间限制为 1~60 秒，保证探针有界。
    if (interface_name.empty() || domain_id < 0 || domain_id > 232 ||
        duration_seconds < 1 || duration_seconds > 60) {
      throw std::invalid_argument("argument outside the accepted safety bounds");
    }

    // 在日志头声明只订阅模式和实际参数；SportModeState 因交付 PC2 缺少
    // 对应 HG 头文件而明确省略，避免把“未检查”误报为“无数据”。
    std::cout << "PROBE_MODE=SUBSCRIBE_ONLY\n"
              << "DOMAIN_ID=" << domain_id << '\n'
              << "INTERFACE=" << interface_name << '\n'
              << "DURATION_SECONDS=" << duration_seconds << '\n'
              << "SPORTMODE_STATE=OMITTED_PC2_HG_HEADER_MISSING\n";

    // 初始化 SDK2 DDS 工厂；本程序之后只构造 ChannelSubscriber。
    unitree::robot::ChannelFactory::Instance()->Init(domain_id, interface_name);

    // 为七个候选状态通道分别维护统计，便于确认实际活跃命名变体。
    TopicStats lowstate_raw_stats("rt/lowstate_raw");
    TopicStats lowstate_stats("rt/lowstate");
    TopicStats lf_lowstate_stats("rt/lf/lowstate");
    TopicStats secondary_imu_stats("rt/secondary_imu");
    TopicStats lf_secondary_imu_stats("rt/lf/secondary_imu");
    TopicStats bms_stats("rt/lf/bmsstate");
    TopicStats mainboard_stats("rt/lf/mainboardstate");

    // 订阅类型与话题一一对应：三路 LowState、两路 IMU、BMS 和 MainBoard。
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

    // 每个 receive channel 队列深度为 1，只关心最新状态，避免探针处理落后
    // 时累计历史数据；回调只更新本话题统计。
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

    // 固定观察窗口内主线程只等待，不发布、调用 RPC 或改变机器人状态。
    std::this_thread::sleep_for(std::chrono::seconds(duration_seconds));

    // 先关闭所有 receive channel，再读取最终统计，防止打印期间计数变化。
    lowstate_raw->CloseChannel();
    lowstate->CloseChannel();
    lf_lowstate->CloseChannel();
    secondary_imu->CloseChannel();
    lf_secondary_imu->CloseChannel();
    bms->CloseChannel();
    mainboard->CloseChannel();

    // 按稳定顺序输出各话题统计，方便脚本和人工跨次比较。
    PrintStats(lowstate_raw_stats);
    PrintStats(lowstate_stats);
    PrintStats(lf_lowstate_stats);
    PrintStats(secondary_imu_stats);
    PrintStats(lf_secondary_imu_stats);
    PrintStats(bms_stats);
    PrintStats(mainboard_stats);

    // 所有订阅关闭后释放全局 DDS 工厂，确保进程退出前无残留 participant。
    unitree::robot::ChannelFactory::Instance()->Release();

    // 验收按“状态类别”判断：LowState/IMU 可由任一候选命名提供，
    // BMS/MainBoard 则要求对应低频通道确实收到至少一帧。
    const bool have_lowstate =
        lowstate_raw_stats.samples.load() > 0 ||
        lowstate_stats.samples.load() > 0 ||
        lf_lowstate_stats.samples.load() > 0;
    const bool have_imu = secondary_imu_stats.samples.load() > 0 ||
                          lf_secondary_imu_stats.samples.load() > 0;
    const bool have_bms = bms_stats.samples.load() > 0;
    const bool have_mainboard = mainboard_stats.samples.load() > 0;

    // 全部四类状态存在返回 0；缺类返回 3，供外层 Stage 05 门禁阻止推进。
    if (have_lowstate && have_imu && have_bms && have_mainboard) {
      std::cout << "H2_HG_SUBSCRIBE_ONLY_PROBE_OK\n";
      return EXIT_SUCCESS;
    }

    std::cout << "H2_HG_SUBSCRIBE_ONLY_PROBE_INCOMPLETE\n";
    return 3;
  } catch (const std::exception &error) {
    // 参数、DDS 初始化或运行异常统一输出稳定键并返回 2。
    std::cerr << "PROBE_EXCEPTION=" << error.what() << '\n';
    return 2;
  }
}
