// 105-to-106 被动传感器接收节点。校验指定来源主机发送的 TCP 帧并发布
// ROS2 消息；不打开传感器设备，也不发送机器人命令。
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include "x30_sensor_relay/wire_protocol_reader.hpp"

namespace {

using x30_sensor_relay::ByteReader;
using x30_sensor_relay::FrameHeader;
using x30_sensor_relay::StreamId;

class TcpStreamServer {
 public:
  using PayloadHandler =
      std::function<bool(const FrameHeader&, const std::vector<std::uint8_t>&)>;

  TcpStreamServer(rclcpp::Logger logger,
                  std::string name,
                  std::string bind_address,
                  std::string allowed_source_ip,
                  int port,
                  StreamId expected_stream,
                  std::uint32_t max_payload_bytes,
                  PayloadHandler handler)
      : logger_(std::move(logger)),
        name_(std::move(name)),
        bind_address_(std::move(bind_address)),
        allowed_source_ip_(std::move(allowed_source_ip)),
        port_(port),
        expected_stream_(expected_stream),
        max_payload_bytes_(max_payload_bytes),
        handler_(std::move(handler)) {}

  ~TcpStreamServer() {
    stop();
  }

  bool start() {
    if (running_.exchange(true)) {
      return startup_state_.load() == 1;
    }
    startup_state_.store(0);
    worker_ = std::thread(&TcpStreamServer::run, this);

    // 端口绑定失败时必须终止构造。最多等待一秒，获取监听线程启动结果。
    for (int attempt = 0; attempt < 100; ++attempt) {
      const int state = startup_state_.load();
      if (state != 0) {
        return state == 1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_ERROR(logger_, "[%s] listener startup timed out", name_.c_str());
    startup_state_.store(-1);
    stop();
    return false;
  }

  void stop() {
    // 关闭两个描述符可解除 accept(2) 或 recv(2) 阻塞，使工作线程无需取消
    // 或分离即可正常 join。
    running_.store(false);
    closeAtomicSocket(client_fd_);
    closeAtomicSocket(listen_fd_);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  std::uint64_t frames() const {
    return frames_.load();
  }

  std::uint64_t invalidFrames() const {
    return invalid_frames_.load();
  }

  std::uint64_t sequenceGaps() const {
    return sequence_gaps_.load();
  }

  std::uint64_t connections() const {
    return connections_.load();
  }

  bool connected() const {
    return client_fd_.load() >= 0;
  }

 private:
  static void closeAtomicSocket(std::atomic<int>& socket_holder) {
    const int fd = socket_holder.exchange(-1);
    if (fd >= 0) {
      ::shutdown(fd, SHUT_RDWR);
      ::close(fd);
    }
  }

  void run() {
    // 每个 server 实例只管理一路数据和一个端口，使大点云帧与 IMU、
    // 里程计 TCP 字节流相互隔离。
    const int server_fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_fd < 0) {
      RCLCPP_ERROR(logger_, "[%s] socket creation failed: %s", name_.c_str(),
                   std::strerror(errno));
      startup_state_.store(-1);
      running_.store(false);
      return;
    }
    listen_fd_.store(server_fd);

    const int enabled = 1;
    ::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &enabled,
                 sizeof(enabled));

    struct sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port_));
    if (::inet_pton(AF_INET, bind_address_.c_str(), &address.sin_addr) != 1) {
      RCLCPP_ERROR(logger_, "[%s] invalid bind address: %s", name_.c_str(),
                   bind_address_.c_str());
      closeOwnedSocket(listen_fd_, server_fd);
      startup_state_.store(-1);
      running_.store(false);
      return;
    }

    if (::bind(server_fd, reinterpret_cast<struct sockaddr*>(&address),
               sizeof(address)) != 0) {
      RCLCPP_ERROR(logger_, "[%s] bind %s:%d failed: %s", name_.c_str(),
                   bind_address_.c_str(), port_, std::strerror(errno));
      closeOwnedSocket(listen_fd_, server_fd);
      startup_state_.store(-1);
      running_.store(false);
      return;
    }

    if (::listen(server_fd, 2) != 0) {
      RCLCPP_ERROR(logger_, "[%s] listen failed: %s", name_.c_str(),
                   std::strerror(errno));
      closeOwnedSocket(listen_fd_, server_fd);
      startup_state_.store(-1);
      running_.store(false);
      return;
    }

    RCLCPP_INFO(logger_, "[%s] listening on %s:%d; allowed source=%s",
                name_.c_str(), bind_address_.c_str(), port_,
                allowed_source_ip_.empty() ? "any"
                                           : allowed_source_ip_.c_str());
    startup_state_.store(1);

    while (running_.load()) {
      struct sockaddr_in peer {};
      socklen_t peer_size = sizeof(peer);
      const int client =
          ::accept(server_fd, reinterpret_cast<struct sockaddr*>(&peer),
                   &peer_size);
      if (client < 0) {
        if (!running_.load()) {
          break;
        }
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_WARN(logger_, "[%s] accept failed: %s", name_.c_str(),
                    std::strerror(errno));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      char peer_ip[INET_ADDRSTRLEN] = {};
      ::inet_ntop(AF_INET, &peer.sin_addr, peer_ip, sizeof(peer_ip));
      // host 网络会在 106 上暴露监听端口。限制对端为 105，防止其他局域网
      // 客户端注入 ROS2 传感器数据。
      if (!allowed_source_ip_.empty() &&
          allowed_source_ip_ != std::string(peer_ip)) {
        RCLCPP_WARN(logger_, "[%s] rejecting unexpected source %s",
                    name_.c_str(), peer_ip);
        ::shutdown(client, SHUT_RDWR);
        ::close(client);
        continue;
      }

      configureClient(client);
      client_fd_.store(client);
      connections_.fetch_add(1);
      // 每次重连开启新的 sequence 观测窗口。间隙只在单次连接内统计，
      // 重连次数单独报告。
      have_sequence_ = false;
      RCLCPP_INFO(logger_, "[%s] connected from %s", name_.c_str(), peer_ip);

      handleClient(client);
      closeOwnedSocket(client_fd_, client);
      if (running_.load()) {
        RCLCPP_WARN(logger_, "[%s] source disconnected", name_.c_str());
      }
    }

    closeOwnedSocket(listen_fd_, server_fd);
  }

  void configureClient(int fd) const {
    // Keepalive 用于及时发现 105 关机或断线；接收超时也使 readExact
    // 能够响应关闭请求。
    const int enabled = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
    ::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled));

#ifdef TCP_KEEPIDLE
    const int keep_idle_seconds = 2;
    const int keep_interval_seconds = 1;
    const int keep_probe_count = 3;
    ::setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &keep_idle_seconds,
                 sizeof(keep_idle_seconds));
    ::setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keep_interval_seconds,
                 sizeof(keep_interval_seconds));
    ::setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &keep_probe_count,
                 sizeof(keep_probe_count));
#endif

    struct timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 500000;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  }

  void handleClient(int fd) {
    std::array<std::uint8_t, x30_sensor_relay::kFrameHeaderSize> header_bytes{};

    while (running_.load()) {
      // TCP 是字节流，因此帧头和 payload 都必须通过 readExact 拼接，
      // 不能假设一次 recv 就对应一个完整转发帧。
      if (!readExact(fd, header_bytes.data(), header_bytes.size())) {
        break;
      }

      FrameHeader header;
      // 错误帧头会导致流边界失步，因此关闭连接，并要求 105 写入端从下一帧
      // 边界重新连接。
      if (!x30_sensor_relay::parseFrameHeader(
              header_bytes.data(), header_bytes.size(), header) ||
          header.stream != expected_stream_ ||
          header.payload_size > max_payload_bytes_) {
        invalid_frames_.fetch_add(1);
        RCLCPP_ERROR(logger_,
                     "[%s] invalid frame header; closing source connection",
                     name_.c_str());
        break;
      }

      std::vector<std::uint8_t> payload(header.payload_size);
      if (!payload.empty() &&
          !readExact(fd, payload.data(), payload.size())) {
        break;
      }

      updateSequenceMetrics(header.sequence);
      // payload 按数据流类型解码。格式错误的 payload 会被跳过，
      // 后续合法帧仍可继续处理。
      if (!handler_(header, payload)) {
        invalid_frames_.fetch_add(1);
        RCLCPP_WARN(logger_, "[%s] rejected malformed payload sequence=%lu",
                    name_.c_str(), header.sequence);
        continue;
      }
      frames_.fetch_add(1);
    }
  }

  bool readExact(int fd, std::uint8_t* destination, std::size_t size) const {
    std::size_t offset = 0;
    // 只要仍有字节到达，就容忍重复 socket 超时；连续两秒无进展则判定为
    // 失联或不完整帧。
    auto last_progress = std::chrono::steady_clock::now();
    while (running_.load() && offset < size) {
      const ssize_t received =
          ::recv(fd, destination + offset, size - offset, 0);
      if (received > 0) {
        offset += static_cast<std::size_t>(received);
        last_progress = std::chrono::steady_clock::now();
        continue;
      }
      if (received == 0) {
        return false;
      }
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        if (std::chrono::steady_clock::now() - last_progress >=
            std::chrono::seconds(2)) {
          return false;
        }
        continue;
      }
      return false;
    }
    return offset == size;
  }

  void updateSequenceMetrics(std::uint64_t sequence) {
    // sequence 间隙用于反映 105 有界队列丢帧或断线清队列造成的缺失，
    // 不会据此合成 ROS 数据。
    if (have_sequence_ && sequence > last_sequence_ + 1) {
      sequence_gaps_.fetch_add(sequence - last_sequence_ - 1);
    }
    have_sequence_ = true;
    last_sequence_ = sequence;
  }

  static void closeOwnedSocket(std::atomic<int>& holder, int owned_fd) {
    int expected = owned_fd;
    if (holder.compare_exchange_strong(expected, -1)) {
      ::shutdown(owned_fd, SHUT_RDWR);
      ::close(owned_fd);
    }
  }

  rclcpp::Logger logger_;
  std::string name_;
  std::string bind_address_;
  std::string allowed_source_ip_;
  int port_{0};
  StreamId expected_stream_{StreamId::kPointCloud};
  std::uint32_t max_payload_bytes_{0};
  PayloadHandler handler_;

  std::atomic<bool> running_{false};
  std::atomic<int> startup_state_{0};
  std::atomic<int> listen_fd_{-1};
  std::atomic<int> client_fd_{-1};
  std::thread worker_;

  bool have_sequence_{false};
  std::uint64_t last_sequence_{0};
  std::atomic<std::uint64_t> frames_{0};
  std::atomic<std::uint64_t> invalid_frames_{0};
  std::atomic<std::uint64_t> sequence_gaps_{0};
  std::atomic<std::uint64_t> connections_{0};
};

// ROS1 与 ROS2 的 header 时间表示不同。本链路传输统一纳秒计数，
// 使重建消息保留源时间，而非 TCP 到达时间。
void setStamp(std_msgs::msg::Header& header, std::uint64_t stamp_ns) {
  header.stamp.sec =
      static_cast<std::int32_t>(stamp_ns / 1000000000ULL);
  header.stamp.nanosec =
      static_cast<std::uint32_t>(stamp_ns % 1000000000ULL);
}

bool readVector3(ByteReader& reader, geometry_msgs::msg::Vector3& value) {
  return reader.readDouble(value.x) && reader.readDouble(value.y) &&
         reader.readDouble(value.z);
}

template <typename ArrayType>
bool readDoubleArray(ByteReader& reader, ArrayType& values) {
  for (auto& value : values) {
    if (!reader.readDouble(value)) {
      return false;
    }
  }
  return true;
}

class X30SensorReceiver : public rclcpp::Node {
 public:
  X30SensorReceiver() : Node("x30_sensor_receiver") {
    // 参数定义跨主机合同；覆盖地址和 Topic 名后，同一镜像可部署到其他 X30。
    bind_address_ =
        declare_parameter<std::string>("bind_address", "0.0.0.0");
    allowed_source_ip_ =
        declare_parameter<std::string>("allowed_source_ip", "192.168.1.105");
    point_cloud_port_ = declare_parameter<int>("point_cloud_port", 56110);
    imu_port_ = declare_parameter<int>("imu_port", 56111);
    odometry_port_ = declare_parameter<int>("odometry_port", 56112);
    point_cloud_topic_ = declare_parameter<std::string>(
        "point_cloud_topic", "/x30/lidar_points");
    imu_topic_ =
        declare_parameter<std::string>("imu_topic", "/x30/body_imu");
    odometry_topic_ =
        declare_parameter<std::string>("odometry_topic", "/x30/leg_odom");
    const int configured_max =
        declare_parameter<int>("max_payload_bytes", 67108864);
    max_payload_bytes_ = static_cast<std::uint32_t>(
        std::max(1024, configured_max));

    // 传感器 Topic 是实时数据而非持久状态。best-effort 有界 QoS 可防止慢速
    // 消费者造成无限延迟；由于单帧点云较大，点云历史深度刻意设置得更小。
    point_cloud_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic_,
            rclcpp::QoS(rclcpp::KeepLast(2))
                .best_effort()
                .durability_volatile());
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::QoS(rclcpp::KeepLast(100))
                        .best_effort()
                        .durability_volatile());
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
        odometry_topic_, rclcpp::QoS(rclcpp::KeepLast(100))
                              .best_effort()
                              .durability_volatile());

    // 三个独立监听器保持端到端数据流隔离：
    // 56110 点云、56111 IMU、56112 腿部里程计。
    point_cloud_server_.reset(new TcpStreamServer(
        get_logger(), "point_cloud", bind_address_, allowed_source_ip_,
        point_cloud_port_, StreamId::kPointCloud, max_payload_bytes_,
        [this](const FrameHeader& header,
               const std::vector<std::uint8_t>& payload) {
          return publishPointCloud(header, payload);
        }));
    imu_server_.reset(new TcpStreamServer(
        get_logger(), "imu", bind_address_, allowed_source_ip_, imu_port_,
        StreamId::kImu, max_payload_bytes_,
        [this](const FrameHeader& header,
               const std::vector<std::uint8_t>& payload) {
          return publishImu(header, payload);
        }));
    odometry_server_.reset(new TcpStreamServer(
        get_logger(), "odometry", bind_address_, allowed_source_ip_,
        odometry_port_, StreamId::kOdometry, max_payload_bytes_,
        [this](const FrameHeader& header,
               const std::vector<std::uint8_t>& payload) {
          return publishOdometry(header, payload);
        }));

    // 下游算法需要三路输入，不允许部分启动。抛出异常会销毁已创建的 server
    // 并停止其线程。
    if (!point_cloud_server_->start() || !imu_server_->start() ||
        !odometry_server_->start()) {
      throw std::runtime_error(
          "failed to start one or more passive TCP listeners");
    }

    status_timer_ = create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&X30SensorReceiver::reportStatus, this));

    RCLCPP_INFO(
        get_logger(),
        "passive receiver started; source=%s, topics=(%s, %s, %s)",
        allowed_source_ip_.c_str(), point_cloud_topic_.c_str(),
        imu_topic_.c_str(), odometry_topic_.c_str());
  }

  ~X30SensorReceiver() override {
    if (point_cloud_server_) {
      point_cloud_server_->stop();
    }
    if (imu_server_) {
      imu_server_->stop();
    }
    if (odometry_server_) {
      odometry_server_->stop();
    }
  }

 private:
  bool publishPointCloud(const FrameHeader& header,
                         const std::vector<std::uint8_t>& payload) {
    // 按 105 的序列化内容原样重建 ROS2 PointCloud2 结构和原始数据。
    // 本接收端不执行点云融合、坐标变换、滤波或运动补偿。
    ByteReader reader(payload.data(), payload.size());
    sensor_msgs::msg::PointCloud2 message;
    std::uint32_t ros1_sequence = 0;
    std::uint32_t field_count = 0;
    std::uint8_t bool_value = 0;

    if (!reader.readU32(ros1_sequence) ||
        !reader.readString(message.header.frame_id) ||
        !reader.readU32(message.height) || !reader.readU32(message.width) ||
        !reader.readU32(field_count) || field_count > 128U) {
      return false;
    }
    // ROS2 Header 没有 sequence 字段。源 sequence 仅为格式兼容而解析，
    // 转发帧的 sequence 指标仍会保留。
    (void)ros1_sequence;

    message.fields.reserve(field_count);
    for (std::uint32_t i = 0; i < field_count; ++i) {
      sensor_msgs::msg::PointField field;
      if (!reader.readString(field.name) || !reader.readU32(field.offset) ||
          !reader.readU8(field.datatype) || !reader.readU32(field.count)) {
        return false;
      }
      message.fields.push_back(std::move(field));
    }

    if (!reader.readU8(bool_value)) {
      return false;
    }
    message.is_bigendian = bool_value != 0;
    if (!reader.readU32(message.point_step) ||
        !reader.readU32(message.row_step) || !reader.readU8(bool_value)) {
      return false;
    }
    message.is_dense = bool_value != 0;

    // 点云内部 data 长度单独受限，并要求 payload 字节全部消费完毕，
    // 从而拒绝截断数据和尾部多余字节。
    std::uint32_t data_size = 0;
    if (!reader.readU32(data_size) ||
        data_size > max_payload_bytes_ ||
        !reader.readBytes(message.data, data_size) ||
        reader.remaining() != 0) {
      return false;
    }

    setStamp(message.header, header.source_stamp_ns);
    point_cloud_publisher_->publish(std::move(message));
    return true;
  }

  bool publishImu(const FrameHeader& header,
                  const std::vector<std::uint8_t>& payload) {
    // 字段顺序与 105 序列化端一致，并包含全部协方差数据。
    ByteReader reader(payload.data(), payload.size());
    sensor_msgs::msg::Imu message;
    std::uint32_t ros1_sequence = 0;

    if (!reader.readU32(ros1_sequence) ||
        !reader.readString(message.header.frame_id) ||
        !reader.readDouble(message.orientation.x) ||
        !reader.readDouble(message.orientation.y) ||
        !reader.readDouble(message.orientation.z) ||
        !reader.readDouble(message.orientation.w) ||
        !readDoubleArray(reader, message.orientation_covariance) ||
        !readVector3(reader, message.angular_velocity) ||
        !readDoubleArray(reader, message.angular_velocity_covariance) ||
        !readVector3(reader, message.linear_acceleration) ||
        !readDoubleArray(reader, message.linear_acceleration_covariance) ||
        reader.remaining() != 0) {
      return false;
    }
    (void)ros1_sequence;

    setStamp(message.header, header.source_stamp_ns);
    imu_publisher_->publish(std::move(message));
    return true;
  }

  bool publishOdometry(const FrameHeader& header,
                       const std::vector<std::uint8_t>& payload) {
    // 保留父子 frame、pose、twist 和协方差，不执行定位或坐标系转换。
    ByteReader reader(payload.data(), payload.size());
    nav_msgs::msg::Odometry message;
    std::uint32_t ros1_sequence = 0;

    if (!reader.readU32(ros1_sequence) ||
        !reader.readString(message.header.frame_id) ||
        !reader.readString(message.child_frame_id) ||
        !reader.readDouble(message.pose.pose.position.x) ||
        !reader.readDouble(message.pose.pose.position.y) ||
        !reader.readDouble(message.pose.pose.position.z) ||
        !reader.readDouble(message.pose.pose.orientation.x) ||
        !reader.readDouble(message.pose.pose.orientation.y) ||
        !reader.readDouble(message.pose.pose.orientation.z) ||
        !reader.readDouble(message.pose.pose.orientation.w) ||
        !readDoubleArray(reader, message.pose.covariance) ||
        !readVector3(reader, message.twist.twist.linear) ||
        !readVector3(reader, message.twist.twist.angular) ||
        !readDoubleArray(reader, message.twist.covariance) ||
        reader.remaining() != 0) {
      return false;
    }
    (void)ros1_sequence;

    setStamp(message.header, header.source_stamp_ns);
    odometry_publisher_->publish(std::move(message));
    return true;
  }

  void reportServer(const std::string& name,
                    const TcpStreamServer& server) const {
    RCLCPP_INFO(get_logger(),
                "%s connected=%s frames=%lu invalid=%lu sequence_gaps=%lu "
                "connections=%lu",
                name.c_str(), server.connected() ? "yes" : "no",
                server.frames(), server.invalidFrames(), server.sequenceGaps(),
                server.connections());
  }

  void reportStatus() const {
    // 这些计数用于区分源断线、输入格式错误和上游队列丢帧，
    // 不会修改或重放传感器消息。
    reportServer("point_cloud", *point_cloud_server_);
    reportServer("imu", *imu_server_);
    reportServer("odometry", *odometry_server_);
  }

  std::string bind_address_;
  std::string allowed_source_ip_;
  std::string point_cloud_topic_;
  std::string imu_topic_;
  std::string odometry_topic_;
  int point_cloud_port_{56110};
  int imu_port_{56111};
  int odometry_port_{56112};
  std::uint32_t max_payload_bytes_{67108864U};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      odometry_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::unique_ptr<TcpStreamServer> point_cloud_server_;
  std::unique_ptr<TcpStreamServer> imu_server_;
  std::unique_ptr<TcpStreamServer> odometry_server_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    // TcpStreamServer 管理网络线程；ROS executor 发布解码消息并运行
    // 低频状态定时器。
    rclcpp::spin(std::make_shared<X30SensorReceiver>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("x30_sensor_receiver"),
                 "receiver startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
