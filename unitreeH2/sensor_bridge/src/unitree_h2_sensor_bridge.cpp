#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "unitree_h2_sensor_bridge/imu_conversion.hpp"

// H2 只读 IMU 桥接节点。
//
// 数据链：
//   PC1/传感器 -> HG DDS rt/lowstate、rt/secondary_imu
//   -> 本节点 -> ROS 2 /h2/imu/pelvis、/h2/imu/torso
//
// 本节点只创建 DDS 订阅者和 ROS 2 发布者，不创建 LocoClient、不打开
// /api/sport 写通道，也不调用 RobotHardwareInterface 的运动接口。
namespace {

using VendorImuState = unitree_hg::msg::dds_::IMUState_;
using VendorLowState = unitree_hg::msg::dds_::LowState_;
using unitree::robot::ChannelSubscriber;

// 读取环境变量；未设置或为空字符串时使用回退值。ROS 参数随后仍可覆盖它。
std::string EnvironmentOr(const char *name, const std::string &fallback) {
  const char *value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

// 读取整数环境变量并进行完整字符串解析；尾随字符会被视为配置错误，
// 由 main 的异常处理统一打印并返回非零退出码。
int EnvironmentIntegerOr(const char *name, int fallback) {
  const std::string value = EnvironmentOr(name, "");
  if (value.empty()) {
    return fallback;
  }
  std::size_t used = 0;
  const int parsed = std::stoi(value, &used);
  if (used != value.size()) {
    throw std::invalid_argument(std::string("invalid ") + name + ": " + value);
  }
  return parsed;
}

// DDS 回调与 ROS 定时器之间共享的“最新一帧”缓存。
// sequence 用来确保每个 DDS 样本最多发布一次，valid 区分尚未收到首帧。
struct LatestImu {
  VendorImuState message;
  builtin_interfaces::msg::Time received_stamp;
  std::uint64_t sequence{0};
  bool valid{false};
};

class UnitreeH2SensorBridge final : public rclcpp::Node {
 public:
  UnitreeH2SensorBridge()
      : Node("unitree_h2_sensor_bridge") {
    // DDS 绑定参数：默认使用 H2 PC2 与 PC1 通信的 eth0、Domain 0。
    // 环境变量适合容器部署，ROS 参数文件适合固定产品配置。
    dds_interface_ = declare_parameter<std::string>(
        "dds_interface", EnvironmentOr("H2_DDS_INTERFACE", "eth0"));
    dds_domain_ = declare_parameter<int>(
        "dds_domain", EnvironmentIntegerOr("H2_DDS_DOMAIN", 0));

    // 原厂 DDS 输入契约：骨盆 IMU 嵌在 LowState 中，躯干 IMU 是独立消息。
    pelvis_dds_topic_ =
        declare_parameter<std::string>("pelvis_dds_topic", "rt/lowstate");
    torso_dds_topic_ = declare_parameter<std::string>(
        "torso_dds_topic", "rt/secondary_imu");

    // Docker 内统一的 ROS 2 输出话题及 frame_id；上层算法只依赖这些名称，
    // 不直接耦合宇树 HG 消息类型。
    pelvis_ros_topic_ =
        declare_parameter<std::string>("pelvis_ros_topic", "/h2/imu/pelvis");
    torso_ros_topic_ =
        declare_parameter<std::string>("torso_ros_topic", "/h2/imu/torso");
    pelvis_frame_id_ =
        declare_parameter<std::string>("pelvis_frame_id", "h2_pelvis_imu");
    torso_frame_id_ =
        declare_parameter<std::string>("torso_frame_id", "h2_torso_imu");

    // publish_rate_hz 是检查最新缓存的频率；stale_timeout_ms 限制输入最大
    // 允许年龄，避免 DDS 断流后重复发布旧姿态。
    publish_rate_hz_ =
        declare_parameter<double>("publish_rate_hz", 200.0);
    stale_timeout_ms_ =
        declare_parameter<int>("stale_timeout_ms", 100);

    ValidateParameters();

    // IMU 是高频、允许偶发丢帧的传感器流，使用 ROS 2 SensorDataQoS，
    // 队列深度 5 用于吸收短时调度抖动。
    const auto qos = rclcpp::SensorDataQoS().keep_last(5);
    pelvis_publisher_ =
        create_publisher<sensor_msgs::msg::Imu>(pelvis_ros_topic_, qos);
    torso_publisher_ =
        create_publisher<sensor_msgs::msg::Imu>(torso_ros_topic_, qos);

    // SDK2 ChannelFactory 仅初始化 DDS 通信域和网卡；此处没有控制客户端。
    unitree::robot::ChannelFactory::Instance()->Init(
        dds_domain_, dds_interface_);
    channel_factory_initialized_ = true;

    pelvis_subscriber_ =
        std::make_shared<ChannelSubscriber<VendorLowState>>(pelvis_dds_topic_);
    torso_subscriber_ =
        std::make_shared<ChannelSubscriber<VendorImuState>>(torso_dds_topic_);

    // DDS 回调只复制最新数据并记录接收时间，不在 DDS 线程中执行 ROS 发布，
    // 从而缩短回调临界路径并隔离两套中间件的线程模型。
    pelvis_subscriber_->InitChannel(
        [this](const void *data) {
          const auto &low_state =
              *static_cast<const VendorLowState *>(data);
          StoreSample(low_state.imu_state(), pelvis_sample_);
        },
        1);

    // secondary_imu 本身就是 HG IMUState，无需从复合状态消息取子字段。
    torso_subscriber_->InitChannel(
        [this](const void *data) {
          const auto &imu = *static_cast<const VendorImuState *>(data);
          StoreSample(imu, torso_sample_);
        },
        1);

    // ROS 定时器按照配置频率检查两路缓存，并只发布尚未发布的新鲜样本。
    const auto timer_period = std::chrono::duration_cast<
        std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ = create_wall_timer(
        timer_period, [this]() { PublishLatest(); });

    RCLCPP_INFO(
        get_logger(),
        "H2_IMU_BRIDGE_READY dds_interface=%s dds_domain=%d "
        "pelvis_dds=%s torso_dds=%s pelvis_ros=%s torso_ros=%s "
        "publish_rate_hz=%.3f stale_timeout_ms=%d",
        dds_interface_.c_str(), dds_domain_, pelvis_dds_topic_.c_str(),
        torso_dds_topic_.c_str(), pelvis_ros_topic_.c_str(),
        torso_ros_topic_.c_str(), publish_rate_hz_, stale_timeout_ms_);
  }

  ~UnitreeH2SensorBridge() override {
    // 按“停止定时器 -> 关闭 DDS 通道 -> 释放全局 ChannelFactory”的顺序
    // 清理，避免析构过程中仍有回调访问已释放成员。
    publish_timer_.reset();
    if (pelvis_subscriber_) {
      pelvis_subscriber_->CloseChannel();
    }
    if (torso_subscriber_) {
      torso_subscriber_->CloseChannel();
    }
    pelvis_subscriber_.reset();
    torso_subscriber_.reset();
    if (channel_factory_initialized_) {
      unitree::robot::ChannelFactory::Instance()->Release();
    }
  }

 private:
  // 启动前集中校验参数，配置不合法时快速失败，不留下半初始化 DDS 通道。
  void ValidateParameters() const {
    if (dds_interface_.empty()) {
      throw std::invalid_argument("dds_interface must not be empty");
    }
    if (dds_domain_ < 0 || dds_domain_ > 232) {
      throw std::invalid_argument("dds_domain must be in [0, 232]");
    }
    if (pelvis_dds_topic_.empty() || torso_dds_topic_.empty() ||
        pelvis_ros_topic_.empty() || torso_ros_topic_.empty() ||
        pelvis_frame_id_.empty() || torso_frame_id_.empty()) {
      throw std::invalid_argument("topic names and frame IDs must not be empty");
    }
    if (publish_rate_hz_ < 1.0 || publish_rate_hz_ > 1000.0) {
      throw std::invalid_argument("publish_rate_hz must be in [1, 1000]");
    }
    if (stale_timeout_ms_ < 10 || stale_timeout_ms_ > 5000) {
      throw std::invalid_argument("stale_timeout_ms must be in [10, 5000]");
    }
  }

  // DDS 回调入口：以同一把互斥锁保护两路缓存，时间戳取本节点 ROS 时钟。
  void StoreSample(const VendorImuState &message, LatestImu &sample) {
    const auto stamp = static_cast<builtin_interfaces::msg::Time>(
        get_clock()->now());
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample.message = message;
    sample.received_stamp = stamp;
    ++sample.sequence;
    sample.valid = true;
  }

  // 为发布线程制作一致性快照；调用方在解锁后完成转换和发布。
  bool ReadSample(const LatestImu &source, LatestImu &destination) {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    if (!source.valid) {
      return false;
    }
    destination = source;
    return true;
  }

  // 只允许接收时间未倒退且年龄未超过 stale_timeout_ms 的样本通过。
  bool IsFresh(const LatestImu &sample) {
    const rclcpp::Time received(sample.received_stamp, get_clock()->get_clock_type());
    const auto age = get_clock()->now() - received;
    return age.nanoseconds() >= 0 &&
           age.nanoseconds() <=
               static_cast<std::int64_t>(stale_timeout_ms_) * 1000000LL;
  }

  // 发布单路 IMU：依次执行去重、时效检查、数值转换和 ROS 发布。
  // 告警使用节流输出，持续断流或坏数据不会淹没容器日志。
  void PublishOne(
      const LatestImu &sample,
      std::uint64_t &last_published_sequence,
      const std::string &frame_id,
      const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr &publisher,
      const char *source_name) {
    if (sample.sequence == last_published_sequence) {
      return;
    }
    if (!IsFresh(sample)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "H2_IMU_STALE source=%s timeout_ms=%d",
          source_name, stale_timeout_ms_);
      return;
    }

    sensor_msgs::msg::Imu output;
    if (!unitree_h2_sensor_bridge::ConvertImu(
            sample.message, sample.received_stamp, frame_id, output)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "H2_IMU_INVALID source=%s", source_name);
      return;
    }
    publisher->publish(output);
    last_published_sequence = sample.sequence;
  }

  // 定时器回调：分别处理骨盆和躯干缓存；任一路尚无首帧时输出等待状态。
  void PublishLatest() {
    LatestImu pelvis;
    LatestImu torso;
    const bool have_pelvis = ReadSample(pelvis_sample_, pelvis);
    const bool have_torso = ReadSample(torso_sample_, torso);

    if (have_pelvis) {
      PublishOne(
          pelvis, pelvis_last_published_sequence_, pelvis_frame_id_,
          pelvis_publisher_, "pelvis");
    }
    if (have_torso) {
      PublishOne(
          torso, torso_last_published_sequence_, torso_frame_id_,
          torso_publisher_, "torso");
    }
    if (!have_pelvis || !have_torso) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "H2_IMU_WAITING pelvis=%d torso=%d",
          have_pelvis ? 1 : 0, have_torso ? 1 : 0);
    }
  }

  // 配置项：DDS 输入绑定、ROS 输出契约以及时效策略。
  std::string dds_interface_;
  int dds_domain_{0};
  std::string pelvis_dds_topic_;
  std::string torso_dds_topic_;
  std::string pelvis_ros_topic_;
  std::string torso_ros_topic_;
  std::string pelvis_frame_id_;
  std::string torso_frame_id_;
  double publish_rate_hz_{200.0};
  int stale_timeout_ms_{100};

  // 通信资源：SDK2 DDS 订阅者与 ROS 2 发布者。
  bool channel_factory_initialized_{false};
  std::shared_ptr<ChannelSubscriber<VendorLowState>> pelvis_subscriber_;
  std::shared_ptr<ChannelSubscriber<VendorImuState>> torso_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pelvis_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr torso_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // 跨线程样本状态：DDS 回调写入，ROS 定时器读取。
  std::mutex sample_mutex_;
  LatestImu pelvis_sample_;
  LatestImu torso_sample_;
  std::uint64_t pelvis_last_published_sequence_{0};
  std::uint64_t torso_last_published_sequence_{0};
};

}  // namespace

int main(int argc, char **argv) {
  // 所有配置/初始化异常在进程边界转换成稳定的日志键和值和退出码 2，
  // 便于 Docker health/audit 脚本判定失败；正常收到关闭信号则返回 0。
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<UnitreeH2SensorBridge>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "H2_IMU_BRIDGE_EXCEPTION=%s\n", error.what());
    rclcpp::shutdown();
    return 2;
  }
}
