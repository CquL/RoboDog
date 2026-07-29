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

namespace {

using VendorImuState = unitree_hg::msg::dds_::IMUState_;
using VendorLowState = unitree_hg::msg::dds_::LowState_;
using unitree::robot::ChannelSubscriber;

std::string EnvironmentOr(const char *name, const std::string &fallback) {
  const char *value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

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
    dds_interface_ = declare_parameter<std::string>(
        "dds_interface", EnvironmentOr("H2_DDS_INTERFACE", "eth0"));
    dds_domain_ = declare_parameter<int>(
        "dds_domain", EnvironmentIntegerOr("H2_DDS_DOMAIN", 0));
    pelvis_dds_topic_ =
        declare_parameter<std::string>("pelvis_dds_topic", "rt/lowstate");
    torso_dds_topic_ = declare_parameter<std::string>(
        "torso_dds_topic", "rt/secondary_imu");
    pelvis_ros_topic_ =
        declare_parameter<std::string>("pelvis_ros_topic", "/h2/imu/pelvis");
    torso_ros_topic_ =
        declare_parameter<std::string>("torso_ros_topic", "/h2/imu/torso");
    pelvis_frame_id_ =
        declare_parameter<std::string>("pelvis_frame_id", "h2_pelvis_imu");
    torso_frame_id_ =
        declare_parameter<std::string>("torso_frame_id", "h2_torso_imu");
    publish_rate_hz_ =
        declare_parameter<double>("publish_rate_hz", 200.0);
    stale_timeout_ms_ =
        declare_parameter<int>("stale_timeout_ms", 100);

    ValidateParameters();

    const auto qos = rclcpp::SensorDataQoS().keep_last(5);
    pelvis_publisher_ =
        create_publisher<sensor_msgs::msg::Imu>(pelvis_ros_topic_, qos);
    torso_publisher_ =
        create_publisher<sensor_msgs::msg::Imu>(torso_ros_topic_, qos);

    unitree::robot::ChannelFactory::Instance()->Init(
        dds_domain_, dds_interface_);
    channel_factory_initialized_ = true;

    pelvis_subscriber_ =
        std::make_shared<ChannelSubscriber<VendorLowState>>(pelvis_dds_topic_);
    torso_subscriber_ =
        std::make_shared<ChannelSubscriber<VendorImuState>>(torso_dds_topic_);

    pelvis_subscriber_->InitChannel(
        [this](const void *data) {
          const auto &low_state =
              *static_cast<const VendorLowState *>(data);
          StoreSample(low_state.imu_state(), pelvis_sample_);
        },
        1);
    torso_subscriber_->InitChannel(
        [this](const void *data) {
          const auto &imu = *static_cast<const VendorImuState *>(data);
          StoreSample(imu, torso_sample_);
        },
        1);

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

  void StoreSample(const VendorImuState &message, LatestImu &sample) {
    const auto stamp = static_cast<builtin_interfaces::msg::Time>(
        get_clock()->now());
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample.message = message;
    sample.received_stamp = stamp;
    ++sample.sequence;
    sample.valid = true;
  }

  bool ReadSample(const LatestImu &source, LatestImu &destination) {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    if (!source.valid) {
      return false;
    }
    destination = source;
    return true;
  }

  bool IsFresh(const LatestImu &sample) {
    const rclcpp::Time received(sample.received_stamp, get_clock()->get_clock_type());
    const auto age = get_clock()->now() - received;
    return age.nanoseconds() >= 0 &&
           age.nanoseconds() <=
               static_cast<std::int64_t>(stale_timeout_ms_) * 1000000LL;
  }

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

  bool channel_factory_initialized_{false};
  std::shared_ptr<ChannelSubscriber<VendorLowState>> pelvis_subscriber_;
  std::shared_ptr<ChannelSubscriber<VendorImuState>> torso_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pelvis_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr torso_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex sample_mutex_;
  LatestImu pelvis_sample_;
  LatestImu torso_sample_;
  std::uint64_t pelvis_last_published_sequence_{0};
  std::uint64_t torso_last_published_sequence_{0};
};

}  // namespace

int main(int argc, char **argv) {
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
