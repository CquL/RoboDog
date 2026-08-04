// X30 被动传感器链的 105 侧转发节点。
//
// 数据流：
//   原厂 ROS1 Topic -> 各数据流有界队列 -> TCP 分帧 -> 106
//
// 本节点只订阅，不发布 ROS1 Topic、不调用服务、不配置 Livox/IMU、
// 不停止原厂进程，也不发送机器人命令。各数据流使用独立工作线程，
// 避免缓慢或正在重连的 TCP 流阻塞其他传感器的 ROS 回调。
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "x30_sensor_relay/wire_protocol.hpp"

namespace {

using x30_sensor_relay::ByteWriter;
using x30_sensor_relay::StreamId;

struct SerializedMessage {
  // 原始 ROS 时间戳写入帧头，payload 保存其余 ROS 消息字段。
  std::uint64_t source_stamp_ns{0};
  std::vector<std::uint8_t> payload;
};

std::uint64_t stampToNanoseconds(const ros::Time& stamp) {
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
         static_cast<std::uint64_t>(stamp.nsec);
}

SerializedMessage serializePointCloud(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  // 原样保留 PointCloud2 字段结构和原始字节数组。本链路不解析、融合、
  // 滤波、坐标变换或重新标记单点时间。
  ByteWriter writer(msg->data.size() + 256U);
  writer.writeU32(msg->header.seq);
  writer.writeString(msg->header.frame_id);
  writer.writeU32(msg->height);
  writer.writeU32(msg->width);
  writer.writeU32(static_cast<std::uint32_t>(msg->fields.size()));
  for (const auto& field : msg->fields) {
    writer.writeString(field.name);
    writer.writeU32(field.offset);
    writer.writeU8(field.datatype);
    writer.writeU32(field.count);
  }
  writer.writeU8(msg->is_bigendian ? 1U : 0U);
  writer.writeU32(msg->point_step);
  writer.writeU32(msg->row_step);
  writer.writeU8(msg->is_dense ? 1U : 0U);
  writer.writeU32(static_cast<std::uint32_t>(msg->data.size()));
  writer.writeBytes(msg->data.data(), msg->data.size());
  return {stampToNanoseconds(msg->header.stamp), writer.take()};
}

void writeVector3(ByteWriter& writer, const geometry_msgs::Vector3& value) {
  writer.writeDouble(value.x);
  writer.writeDouble(value.y);
  writer.writeDouble(value.z);
}

template <typename ArrayType>
void writeDoubleArray(ByteWriter& writer, const ArrayType& values) {
  for (const auto value : values) {
    writer.writeDouble(value);
  }
}

SerializedMessage serializeImu(const sensor_msgs::ImuConstPtr& msg) {
  // 字段顺序与 sensor_msgs/Imu 一致，使 ROS2 接收端可无损重建姿态、
  // 运动向量及全部协方差矩阵。
  ByteWriter writer(384U);
  writer.writeU32(msg->header.seq);
  writer.writeString(msg->header.frame_id);
  writer.writeDouble(msg->orientation.x);
  writer.writeDouble(msg->orientation.y);
  writer.writeDouble(msg->orientation.z);
  writer.writeDouble(msg->orientation.w);
  writeDoubleArray(writer, msg->orientation_covariance);
  writeVector3(writer, msg->angular_velocity);
  writeDoubleArray(writer, msg->angular_velocity_covariance);
  writeVector3(writer, msg->linear_acceleration);
  writeDoubleArray(writer, msg->linear_acceleration_covariance);
  return {stampToNanoseconds(msg->header.stamp), writer.take()};
}

SerializedMessage serializeOdometry(const nav_msgs::OdometryConstPtr& msg) {
  // 保留两个 frame ID、pose/twist 和协方差数组。本步骤仅复制传输，
  // 不进行里程计计算或坐标系转换。
  ByteWriter writer(768U);
  writer.writeU32(msg->header.seq);
  writer.writeString(msg->header.frame_id);
  writer.writeString(msg->child_frame_id);

  writer.writeDouble(msg->pose.pose.position.x);
  writer.writeDouble(msg->pose.pose.position.y);
  writer.writeDouble(msg->pose.pose.position.z);
  writer.writeDouble(msg->pose.pose.orientation.x);
  writer.writeDouble(msg->pose.pose.orientation.y);
  writer.writeDouble(msg->pose.pose.orientation.z);
  writer.writeDouble(msg->pose.pose.orientation.w);
  writeDoubleArray(writer, msg->pose.covariance);

  writeVector3(writer, msg->twist.twist.linear);
  writeVector3(writer, msg->twist.twist.angular);
  writeDoubleArray(writer, msg->twist.covariance);
  return {stampToNanoseconds(msg->header.stamp), writer.take()};
}

class TcpClient {
 public:
  TcpClient(std::string host,
            int port,
            int connect_timeout_ms,
            int send_timeout_ms)
      : host_(std::move(host)),
        port_(port),
        connect_timeout_ms_(connect_timeout_ms),
        send_timeout_ms_(send_timeout_ms) {}

  ~TcpClient() {
    closeSocket();
  }

  bool sendPacket(const std::vector<std::uint8_t>& packet) {
    // send(2) 可能只写入部分帧，因此持续发送，直到完整帧头和 payload
    // 全部写入，或 socket 失败。
    if (!ensureConnected()) {
      return false;
    }

    std::size_t offset = 0;
    while (offset < packet.size()) {
      const ssize_t sent =
          ::send(fd_, packet.data() + offset, packet.size() - offset,
                 MSG_NOSIGNAL);
      if (sent > 0) {
        offset += static_cast<std::size_t>(sent);
        continue;
      }
      if (sent < 0 && errno == EINTR) {
        continue;
      }
      closeSocket();
      return false;
    }
    return true;
  }

  bool connected() const {
    return connected_.load();
  }

 private:
  bool ensureConnected() {
    if (fd_ >= 0) {
      return true;
    }

    // 有时限的非阻塞连接可防止 106 不可达时永久卡住数据流线程。
    struct addrinfo hints {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    struct addrinfo* addresses = nullptr;
    const std::string port_text = std::to_string(port_);
    if (::getaddrinfo(host_.c_str(), port_text.c_str(), &hints, &addresses) !=
        0) {
      return false;
    }

    bool connected = false;
    for (auto* address = addresses; address != nullptr;
         address = address->ai_next) {
      const int candidate =
          ::socket(address->ai_family, address->ai_socktype,
                   address->ai_protocol);
      if (candidate < 0) {
        continue;
      }

      const int original_flags = ::fcntl(candidate, F_GETFL, 0);
      if (original_flags < 0 ||
          ::fcntl(candidate, F_SETFL, original_flags | O_NONBLOCK) < 0) {
        ::close(candidate);
        continue;
      }

      int result =
          ::connect(candidate, address->ai_addr, address->ai_addrlen);
      if (result < 0 && errno == EINPROGRESS) {
        struct pollfd poll_fd {};
        poll_fd.fd = candidate;
        poll_fd.events = POLLOUT;
        result = ::poll(&poll_fd, 1, connect_timeout_ms_);
        if (result > 0) {
          int socket_error = 0;
          socklen_t error_size = sizeof(socket_error);
          if (::getsockopt(candidate, SOL_SOCKET, SO_ERROR, &socket_error,
                           &error_size) != 0 ||
              socket_error != 0) {
            result = -1;
          } else {
            result = 0;
          }
        } else {
          result = -1;
        }
      }

      if (result == 0 &&
          ::fcntl(candidate, F_SETFL, original_flags) == 0) {
        configureConnectedSocket(candidate);
        fd_ = candidate;
        connected_.store(true);
        connected = true;
        break;
      }
      ::close(candidate);
    }

    ::freeaddrinfo(addresses);
    return connected;
  }

  void configureConnectedSocket(int fd) const {
    // 传感器链优先保证低延迟而非批量发送。Keepalive 用于发现失联对端，
    // SO_SNDTIMEO 限制阻塞写入时间。
    const int enabled = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
    ::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled));

    struct timeval send_timeout {};
    send_timeout.tv_sec = send_timeout_ms_ / 1000;
    send_timeout.tv_usec = (send_timeout_ms_ % 1000) * 1000;
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout,
                 sizeof(send_timeout));
  }

  void closeSocket() {
    connected_.store(false);
    if (fd_ >= 0) {
      ::shutdown(fd_, SHUT_RDWR);
      ::close(fd_);
      fd_ = -1;
    }
  }

  std::string host_;
  int port_{0};
  int connect_timeout_ms_{500};
  int send_timeout_ms_{1000};
  int fd_{-1};
  std::atomic<bool> connected_{false};
};

// 每路 ROS 数据拥有独立队列、TCP 连接、sequence 计数器和工作线程。
// ROS 回调只把共享消息指针入队，序列化和网络阻塞不会占用 ROS 回调线程。
template <typename MessagePtr>
class StreamSender {
 public:
  using Serializer =
      std::function<SerializedMessage(const MessagePtr&)>;

  StreamSender(std::string name,
               std::string host,
               int port,
               StreamId stream,
               std::size_t queue_depth,
               bool latest_only,
               int connect_timeout_ms,
               int send_timeout_ms,
               int reconnect_delay_ms,
               Serializer serializer)
      : name_(std::move(name)),
        host_(std::move(host)),
        port_(port),
        stream_(stream),
        queue_depth_(std::max<std::size_t>(1U, queue_depth)),
        latest_only_(latest_only),
        reconnect_delay_ms_(std::max(10, reconnect_delay_ms)),
        serializer_(std::move(serializer)),
        client_(host_, port_, connect_timeout_ms, send_timeout_ms) {}

  ~StreamSender() {
    stop();
  }

  void start() {
    running_.store(true);
    worker_ = std::thread(&StreamSender::run, this);
  }

  void stop() {
    running_.store(false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      dropped_.fetch_add(queue_.size());
      queue_.clear();
    }
    condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void enqueue(const MessagePtr& message) {
    received_.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // 大点云采用 latest_only：发生网络背压后旧点云已失去实时价值。
      // IMU 和里程计采用有界 FIFO，吸收短时突发，同时限制内存和延迟。
      if (latest_only_ && !queue_.empty()) {
        dropped_.fetch_add(queue_.size());
        queue_.clear();
      }
      while (queue_.size() >= queue_depth_) {
        queue_.pop_front();
        dropped_.fetch_add(1);
      }
      queue_.push_back(message);
    }
    condition_.notify_one();
  }

  std::uint64_t received() const {
    return received_.load();
  }

  std::uint64_t sent() const {
    return sent_.load();
  }

  std::uint64_t dropped() const {
    return dropped_.load();
  }

  std::uint64_t failures() const {
    return failures_.load();
  }

  bool connected() const {
    return client_.connected();
  }

 private:
  void run() {
    while (running_.load()) {
      MessagePtr message;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {
          return !running_.load() || !queue_.empty();
        });
        if (!running_.load()) {
          break;
        }
        message = queue_.front();
        queue_.pop_front();
      }

      const auto serialized = serializer_(message);
      const auto packet = x30_sensor_relay::buildFrame(
          stream_, sequence_++, serialized.source_stamp_ns,
          serialized.payload);
      if (packet.empty() || !client_.sendPacket(packet)) {
        failures_.fetch_add(1);
        // 断线期间积压的数据已经过期，因此清空队列、短暂等待，
        // 并在发送下一条消息时重新连接。
        clearStaleQueue();
        std::this_thread::sleep_for(
            std::chrono::milliseconds(reconnect_delay_ms_));
        continue;
      }
      sent_.fetch_add(1);
    }
  }

  void clearStaleQueue() {
    std::lock_guard<std::mutex> lock(mutex_);
    dropped_.fetch_add(queue_.size());
    queue_.clear();
  }

  std::string name_;
  std::string host_;
  int port_{0};
  StreamId stream_{StreamId::kPointCloud};
  std::size_t queue_depth_{1};
  bool latest_only_{false};
  int reconnect_delay_ms_{250};
  Serializer serializer_;
  TcpClient client_;

  std::atomic<bool> running_{false};
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<MessagePtr> queue_;
  std::uint64_t sequence_{0};

  std::atomic<std::uint64_t> received_{0};
  std::atomic<std::uint64_t> sent_{0};
  std::atomic<std::uint64_t> dropped_{0};
  std::atomic<std::uint64_t> failures_{0};
};

// 管理三路只读 ROS1 订阅，并把每个源 Topic 映射到独立 TCP 流。
// 端口和队列策略必须与 ROS2 launch 文件及跨主机合同测试保持一致。
class X30SensorForwarder {
 public:
  X30SensorForwarder() : private_node_("~") {
    private_node_.param<std::string>("receiver_host", receiver_host_,
                                     "192.168.1.106");
    private_node_.param<std::string>("point_cloud_topic", point_cloud_topic_,
                                     "/lidar_points");
    private_node_.param<std::string>("imu_topic", imu_topic_, "/imu/data");
    private_node_.param<std::string>("odometry_topic", odometry_topic_,
                                     "/leg_odom");

    private_node_.param("point_cloud_port", point_cloud_port_, 56110);
    private_node_.param("imu_port", imu_port_, 56111);
    private_node_.param("odometry_port", odometry_port_, 56112);
    private_node_.param("point_cloud_queue_depth", point_cloud_queue_depth_, 1);
    private_node_.param("imu_queue_depth", imu_queue_depth_, 100);
    private_node_.param("odometry_queue_depth", odometry_queue_depth_, 100);
    private_node_.param("connect_timeout_ms", connect_timeout_ms_, 500);
    private_node_.param("send_timeout_ms", send_timeout_ms_, 1000);
    private_node_.param("reconnect_delay_ms", reconnect_delay_ms_, 250);
    private_node_.param("status_period_s", status_period_s_, 5.0);

    point_cloud_sender_.reset(
        new PointCloudSender("point_cloud", receiver_host_, point_cloud_port_,
                             StreamId::kPointCloud,
                             static_cast<std::size_t>(
                                 std::max(1, point_cloud_queue_depth_)),
                             true, connect_timeout_ms_, send_timeout_ms_,
                             reconnect_delay_ms_, serializePointCloud));
    imu_sender_.reset(
        new ImuSender("imu", receiver_host_, imu_port_, StreamId::kImu,
                      static_cast<std::size_t>(std::max(1, imu_queue_depth_)),
                      false, connect_timeout_ms_, send_timeout_ms_,
                      reconnect_delay_ms_, serializeImu));
    odometry_sender_.reset(new OdometrySender(
        "odometry", receiver_host_, odometry_port_, StreamId::kOdometry,
        static_cast<std::size_t>(std::max(1, odometry_queue_depth_)), false,
        connect_timeout_ms_, send_timeout_ms_, reconnect_delay_ms_,
        serializeOdometry));

    point_cloud_sender_->start();
    imu_sender_->start();
    odometry_sender_->start();

    // ROS 回调只负责入队；ROS1 订阅启用 TCP_NODELAY，避免在本链路队列前
    // 再增加一层批量发送延迟。
    const auto transport_hints = ros::TransportHints().tcpNoDelay(true);
    point_cloud_subscription_ = node_.subscribe<sensor_msgs::PointCloud2>(
        point_cloud_topic_, 1, &X30SensorForwarder::onPointCloud, this,
        transport_hints);
    imu_subscription_ = node_.subscribe<sensor_msgs::Imu>(
        imu_topic_, std::max(10, imu_queue_depth_),
        &X30SensorForwarder::onImu, this, transport_hints);
    odometry_subscription_ = node_.subscribe<nav_msgs::Odometry>(
        odometry_topic_, std::max(10, odometry_queue_depth_),
        &X30SensorForwarder::onOdometry, this, transport_hints);

    status_timer_ = node_.createTimer(
        ros::Duration(std::max(1.0, status_period_s_)),
        &X30SensorForwarder::reportStatus, this);

    ROS_INFO_STREAM(
        "[x30_sensor_forwarder] read-only relay started; destination="
        << receiver_host_ << ", point_cloud=" << point_cloud_topic_ << ":"
        << point_cloud_port_ << ", imu=" << imu_topic_ << ":" << imu_port_
        << ", odometry=" << odometry_topic_ << ":" << odometry_port_);
  }

  ~X30SensorForwarder() {
    // 先停止接收新的 ROS 回调，再结束工作线程。stop() 会清空队列并
    // 确定性地 join 各线程。
    point_cloud_subscription_.shutdown();
    imu_subscription_.shutdown();
    odometry_subscription_.shutdown();
    if (point_cloud_sender_) {
      point_cloud_sender_->stop();
    }
    if (imu_sender_) {
      imu_sender_->stop();
    }
    if (odometry_sender_) {
      odometry_sender_->stop();
    }
  }

 private:
  using PointCloudSender =
      StreamSender<sensor_msgs::PointCloud2ConstPtr>;
  using ImuSender = StreamSender<sensor_msgs::ImuConstPtr>;
  using OdometrySender = StreamSender<nav_msgs::OdometryConstPtr>;

  void onPointCloud(const sensor_msgs::PointCloud2ConstPtr& message) {
    point_cloud_sender_->enqueue(message);
  }

  void onImu(const sensor_msgs::ImuConstPtr& message) {
    imu_sender_->enqueue(message);
  }

  void onOdometry(const nav_msgs::OdometryConstPtr& message) {
    odometry_sender_->enqueue(message);
  }

  template <typename Sender>
  void logSender(const std::string& name, const Sender& sender) const {
    ROS_INFO_STREAM("[x30_sensor_forwarder] " << name
                    << " connected=" << (sender.connected() ? "yes" : "no")
                    << " received=" << sender.received()
                    << " sent=" << sender.sent()
                    << " dropped=" << sender.dropped()
                    << " failures=" << sender.failures());
  }

  void reportStatus(const ros::TimerEvent&) {
    logSender("point_cloud", *point_cloud_sender_);
    logSender("imu", *imu_sender_);
    logSender("odometry", *odometry_sender_);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber point_cloud_subscription_;
  ros::Subscriber imu_subscription_;
  ros::Subscriber odometry_subscription_;
  ros::Timer status_timer_;

  std::string receiver_host_;
  std::string point_cloud_topic_;
  std::string imu_topic_;
  std::string odometry_topic_;
  int point_cloud_port_{56110};
  int imu_port_{56111};
  int odometry_port_{56112};
  int point_cloud_queue_depth_{1};
  int imu_queue_depth_{100};
  int odometry_queue_depth_{100};
  int connect_timeout_ms_{500};
  int send_timeout_ms_{1000};
  int reconnect_delay_ms_{250};
  double status_period_s_{5.0};

  std::unique_ptr<PointCloudSender> point_cloud_sender_;
  std::unique_ptr<ImuSender> imu_sender_;
  std::unique_ptr<OdometrySender> odometry_sender_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "x30_sensor_forwarder");
  X30SensorForwarder forwarder;
  // 三个回调线程让点云、IMU 和里程计订阅独立推进；网络 I/O 仍由各自的
  // StreamSender 工作线程执行。
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
