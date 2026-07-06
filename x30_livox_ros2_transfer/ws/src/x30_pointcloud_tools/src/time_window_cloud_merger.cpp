#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class TimeWindowCloudMerger : public rclcpp::Node {
 public:
  TimeWindowCloudMerger() : Node("time_window_cloud_merger") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/x30/points_merged");
    output_frame_ = declare_parameter<std::string>("output_frame", "lidar_link");
    window_ms_ = declare_parameter<double>("window_ms", 100.0);
    min_clouds_ = declare_parameter<int>("min_clouds", 1);

    if (window_ms_ < 20.0) {
      RCLCPP_WARN(get_logger(), "window_ms %.2f is too small, using 20 ms", window_ms_);
      window_ms_ = 20.0;
    }
    if (min_clouds_ < 1) {
      min_clouds_ = 1;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(64)).reliable().durability_volatile();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          buffer_.push_back(*msg);
        });

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

    auto period = std::chrono::duration<double, std::milli>(window_ms_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&TimeWindowCloudMerger::publishWindow, this));

    RCLCPP_INFO(get_logger(),
                "merging %s into %s every %.1f ms, output_frame=%s, min_clouds=%d",
                input_topic_.c_str(), output_topic_.c_str(), window_ms_,
                output_frame_.c_str(), min_clouds_);
  }

 private:
  void publishWindow() {
    std::vector<sensor_msgs::msg::PointCloud2> clouds;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      clouds.swap(buffer_);
    }

    if (clouds.size() < static_cast<size_t>(min_clouds_)) {
      return;
    }

    sensor_msgs::msg::PointCloud2 merged;
    if (!mergeClouds(clouds, merged)) {
      return;
    }

    pub_->publish(merged);

    published_count_++;
    total_input_count_ += clouds.size();
    total_point_count_ += merged.width;
    auto now = steady_clock_.now();
    if ((now - last_log_time_).seconds() > 2.0) {
      const double avg_inputs =
          published_count_ == 0 ? 0.0 : static_cast<double>(total_input_count_) / published_count_;
      const double avg_points =
          published_count_ == 0 ? 0.0 : static_cast<double>(total_point_count_) / published_count_;
      RCLCPP_INFO(get_logger(),
                  "published %lu merged clouds, last window inputs=%zu points=%u, avg inputs=%.2f avg points=%.0f",
                  published_count_, clouds.size(), merged.width, avg_inputs, avg_points);
      last_log_time_ = now;
    }
  }

  bool mergeClouds(const std::vector<sensor_msgs::msg::PointCloud2>& clouds,
                   sensor_msgs::msg::PointCloud2& merged) {
    const auto* base = selectTemplate(clouds);
    if (base == nullptr) {
      return false;
    }

    merged = *base;
    merged.header.stamp = clouds.back().header.stamp;
    merged.header.frame_id = output_frame_.empty() ? base->header.frame_id : output_frame_;
    merged.height = 1;
    merged.width = 0;
    merged.row_step = 0;
    merged.is_dense = true;
    merged.data.clear();

    size_t total_bytes = 0;
    uint32_t total_points = 0;
    bool skipped = false;
    for (const auto& cloud : clouds) {
      if (!compatible(*base, cloud)) {
        skipped = true;
        continue;
      }
      const uint32_t points = pointCount(cloud);
      total_points += points;
      total_bytes += points * cloud.point_step;
      merged.is_dense = merged.is_dense && cloud.is_dense;
    }

    if (total_points == 0) {
      return false;
    }

    merged.data.reserve(total_bytes);
    for (const auto& cloud : clouds) {
      if (!compatible(*base, cloud)) {
        continue;
      }
      const uint32_t points = pointCount(cloud);
      const size_t bytes = static_cast<size_t>(points) * cloud.point_step;
      merged.data.insert(merged.data.end(), cloud.data.begin(), cloud.data.begin() + bytes);
    }

    merged.width = total_points;
    merged.row_step = merged.width * merged.point_step;

    if (skipped) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "skipped incompatible cloud(s) in merge window");
    }
    return true;
  }

  const sensor_msgs::msg::PointCloud2* selectTemplate(
      const std::vector<sensor_msgs::msg::PointCloud2>& clouds) const {
    for (const auto& cloud : clouds) {
      if (!cloud.data.empty() && cloud.point_step > 0) {
        return &cloud;
      }
    }
    return nullptr;
  }

  static uint32_t pointCount(const sensor_msgs::msg::PointCloud2& cloud) {
    if (cloud.point_step == 0) {
      return 0;
    }
    const uint32_t by_layout = cloud.height == 0 ? cloud.width : cloud.width * cloud.height;
    const uint32_t by_bytes = static_cast<uint32_t>(cloud.data.size() / cloud.point_step);
    return std::min(by_layout, by_bytes);
  }

  static bool compatible(const sensor_msgs::msg::PointCloud2& a,
                         const sensor_msgs::msg::PointCloud2& b) {
    if (a.point_step != b.point_step || a.fields.size() != b.fields.size()) {
      return false;
    }
    for (size_t i = 0; i < a.fields.size(); ++i) {
      const auto& af = a.fields[i];
      const auto& bf = b.fields[i];
      if (af.name != bf.name || af.offset != bf.offset || af.datatype != bf.datatype ||
          af.count != bf.count) {
        return false;
      }
    }
    return true;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double window_ms_{100.0};
  int min_clouds_{1};

  std::mutex mutex_;
  std::vector<sensor_msgs::msg::PointCloud2> buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_log_time_{0, 0, RCL_STEADY_TIME};
  uint64_t published_count_{0};
  uint64_t total_input_count_{0};
  uint64_t total_point_count_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeWindowCloudMerger>());
  rclcpp::shutdown();
  return 0;
}
