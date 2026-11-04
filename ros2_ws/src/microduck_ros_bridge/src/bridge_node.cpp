#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "microduck_ros_bridge/helpers.hpp"

using namespace std::chrono_literals;
using nlohmann::json;

namespace {
using Clock = std::chrono::steady_clock;
constexpr std::size_t kMaxLine = 1024 * 1024;
constexpr auto kReconnectDelay = 500ms;

// Return an owned nonblocking descriptor, or -1 within a bounded connect wait.
int connect_unix(const std::string &path) {
  if (path.size() >= sizeof(sockaddr_un::sun_path))
    return -1;
  const int fd =
      ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0)
    return -1;
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1);
  if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) ==
      0)
    return fd;
  if (errno != EINPROGRESS) {
    ::close(fd);
    return -1;
  }
  pollfd pfd{fd, POLLOUT, 0};
  if (::poll(&pfd, 1, 500) <= 0) {
    ::close(fd);
    return -1;
  }
  int error = 0;
  socklen_t length = sizeof(error);
  if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &length) < 0 ||
      error != 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

bool write_all(int fd, const std::string &data, int timeout_ms = 200) {
  std::size_t sent = 0;
  while (sent < data.size()) {
    const auto n = ::send(fd, data.data() + sent, data.size() - sent,
                          MSG_NOSIGNAL | MSG_DONTWAIT);
    if (n > 0) {
      sent += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR)
      continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd pfd{fd, POLLOUT, 0};
      if (::poll(&pfd, 1, timeout_ms) > 0)
        continue;
    }
    return false;
  }
  return true;
}

enum class ReadResult { Line, Timeout, Closed, Failed };
// Retain partial NDJSON across reads; distinguish idle streams from EOF/errors.
ReadResult read_line(int fd, std::string &buffer, std::string &line,
                     int timeout_ms) {
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  for (;;) {
    const auto newline = buffer.find('\n');
    if (newline != std::string::npos) {
      line = buffer.substr(0, newline);
      buffer.erase(0, newline + 1);
      return ReadResult::Line;
    }
    if (buffer.size() >= kMaxLine)
      return ReadResult::Failed;
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline -
                                                              Clock::now())
            .count();
    if (remaining <= 0)
      return ReadResult::Timeout;
    pollfd pfd{fd, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, static_cast<int>(remaining));
    if (ready == 0)
      return ReadResult::Timeout;
    if (ready < 0) {
      if (errno == EINTR)
        continue;
      return ReadResult::Failed;
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
      return ReadResult::Closed;
    char chunk[8192];
    const auto n =
        ::recv(fd, chunk, std::min(sizeof(chunk), kMaxLine - buffer.size()),
               MSG_DONTWAIT);
    if (n > 0)
      buffer.append(chunk, static_cast<std::size_t>(n));
    else if (n < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)
      return ReadResult::Failed;
    else if (n == 0)
      return ReadResult::Closed;
  }
}

bool finite3(const json &v) {
  return v.is_array() && v.size() == 3 && v[0].is_number() &&
         v[1].is_number() && v[2].is_number() &&
         std::isfinite(v[0].get<double>()) &&
         std::isfinite(v[1].get<double>()) && std::isfinite(v[2].get<double>());
}
geometry_msgs::msg::Quaternion quaternion(const json &q) {
  geometry_msgs::msg::Quaternion out;
  if (!q.is_array() || q.size() != 4)
    return out;
  out.w = q[0].get<double>();
  out.x = q[1].get<double>();
  out.y = q[2].get<double>();
  out.z = q[3].get<double>();
  return out;
}
bool valid_quaternion(const json &q) {
  if (!q.is_array() || q.size() != 4)
    return false;
  double norm = 0.0;
  for (const auto &value : q) {
    if (!value.is_number() || !std::isfinite(value.get<double>()))
      return false;
    norm += value.get<double>() * value.get<double>();
  }
  return std::isfinite(norm) && norm > 1e-12 && std::abs(norm - 1.0) < 0.01;
}

std::int64_t monotonic_ns() {
  timespec ts{};
  if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    return 0;
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
} // namespace

/// Convert shared-host daemon streams to ROS wall-time topics and robot
/// commands. Each socket has one worker; data_mutex_ protects shared sensor
/// metadata/health, and command_mutex_ protects ROS callbacks' intent against
/// the robot worker.
class MicroduckBridge final : public rclcpp::Node {
public:
  MicroduckBridge()
      : Node("microduck_bridge"),
        tf_broadcaster_(tf2_ros::TransformBroadcaster::RequiredInterfaces(
            get_node_parameters_interface(), get_node_topics_interface())) {
    robot_socket_ =
        declare_parameter("robot_socket", "/var/lib/duck-sim/duck-a.sock");
    tof_socket_ =
        declare_parameter("tof_socket", "/var/lib/duck-sim/duck-a-tof.sock");
    command_timeout_ = declare_parameter("command_timeout_sec", 0.25);
    max_forward_ = declare_parameter("max_forward_speed_mps", 0.20);
    max_yaw_ = declare_parameter("max_yaw_rate_radps", 1.0);
    state_timeout_ = declare_parameter("state_timeout_sec", 0.25);
    tof_timeout_ = declare_parameter("tof_timeout_sec", 0.5);
    if (!std::isfinite(command_timeout_) || !std::isfinite(max_forward_) ||
        !std::isfinite(max_yaw_) || !std::isfinite(state_timeout_) ||
        !std::isfinite(tof_timeout_) || command_timeout_ <= 0 ||
        max_forward_ <= 0 || max_yaw_ <= 0 || state_timeout_ <= 0 ||
        tof_timeout_ <= 0)
      throw std::runtime_error("timeouts and velocity limits must be positive");
    if (get_parameter("use_sim_time").as_bool())
      throw std::runtime_error(
          "use_sim_time is unsupported; bridge timestamps use ROS wall time");
    monotonic_ros_offset_ns_ =
        get_clock()->now().nanoseconds() - monotonic_ns();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    joint_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    points_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/tof/points", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/robot_markers", 10);
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10,
        std::bind(&MicroduckBridge::on_command, this, std::placeholders::_1));
    diagnostic_timer_ = create_wall_timer(
        500ms, std::bind(&MicroduckBridge::publish_diagnostics, this));
    robot_thread_ = std::thread(&MicroduckBridge::robot_loop, this);
    tof_thread_ = std::thread(&MicroduckBridge::tof_loop, this);
  }

  ~MicroduckBridge() override {
    stopping_ = true;
    if (robot_thread_.joinable())
      robot_thread_.join();
    if (tof_thread_.joinable())
      tof_thread_.join();
  }

private:
  rclcpp::Time stamp_for(std::uint64_t t_ns) {
    const auto converted = microduck_ros_bridge::monotonic_to_ros_ns(
        t_ns, monotonic_ros_offset_ns_);
    if (!converted)
      throw std::runtime_error("invalid daemon monotonic timestamp");
    return rclcpp::Time(*converted, get_clock()->get_clock_type());
  }

  // Command age includes transit time; steady time bounds its remaining
  // lifetime.
  void on_command(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000U) {
      rejected_commands_++;
      cancel_command();
      return;
    }
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "base_link" &&
        msg->header.frame_id != "base_footprint") {
      rejected_commands_++;
      cancel_command();
      return;
    }
    const auto now = get_clock()->now();
    const auto stamp =
        rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    const auto age = (now - stamp).seconds();
    if (!microduck_ros_bridge::command_fresh(age, command_timeout_)) {
      rejected_commands_++;
      cancel_command();
      return;
    }
    const auto pubs = cmd_sub_->get_publisher_count();
    multiple_publishers_ = pubs > 1;
    if (multiple_publishers_) {
      cancel_command();
      return;
    }
    const auto command = microduck_ros_bridge::validated_command(
        msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z,
        max_forward_, max_yaw_);
    if (!command) {
      rejected_commands_++;
      cancel_command();
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_command_ = *command;
    latest_command_at_ =
        Clock::now() - std::chrono::duration_cast<Clock::duration>(
                           std::chrono::duration<double>(age));
    have_command_ = true;
    expiry_stop_sent_ = false;
  }

  void robot_loop() {
    while (!stopping_) {
      connected_robot_ = false;
      const int fd = connect_unix(robot_socket_);
      if (fd < 0) {
        std::this_thread::sleep_for(kReconnectDelay);
        continue;
      }
      const auto model_response =
          request_on_fd(fd, "robot.model", json::object(), 1);
      if (!model_response || !model_response->contains("result")) {
        ::close(fd);
        std::this_thread::sleep_for(kReconnectDelay);
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        try {
          model_ = model_response->at("result");
        } catch (...) {
          model_ = json::object();
        }
      }
      const json subscribe{{"jsonrpc", "2.0"},
                           {"id", 2},
                           {"method", "robot.subscribe"},
                           {"params", {{"hz", 20}}}};
      if (!write_all(fd, subscribe.dump() + "\n")) {
        ::close(fd);
        std::this_thread::sleep_for(kReconnectDelay);
        continue;
      }
      std::string buffer, line;
      if (read_line(fd, buffer, line, 1000) != ReadResult::Line) {
        ::close(fd);
        std::this_thread::sleep_for(kReconnectDelay);
        continue;
      }
      try {
        if (!json::parse(line)
                 .value("result", json::object())
                 .value("accepted", false))
          throw std::runtime_error("subscription refused");
      } catch (...) {
        ::close(fd);
        std::this_thread::sleep_for(kReconnectDelay);
        continue;
      }
      connected_robot_ = true;
      reset_command();
      previous_odom_.reset();
      filtered_velocity_ = {};
      trajectory_.clear();
      while (!stopping_) {
        flush_command(fd);
        const auto read = read_line(fd, buffer, line, 50);
        if (read == ReadResult::Timeout)
          continue;
        if (read != ReadResult::Line)
          break;
        try {
          const auto message = json::parse(line);
          if (message.value("method", std::string()) == "robot.state" &&
              message.contains("params"))
            publish_state(message.at("params"));
        } catch (const std::exception &e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Invalid robot state: %s", e.what());
        }
      }
      reset_command();
      connected_robot_ = false;
      ::close(fd);
      std::this_thread::sleep_for(kReconnectDelay);
    }
  }

  std::optional<json> request_on_fd(int fd, const std::string &method,
                                    const json &params, int id) {
    const json request{
        {"jsonrpc", "2.0"}, {"id", id}, {"method", method}, {"params", params}};
    std::string buffer, line;
    if (!write_all(fd, request.dump() + "\n") ||
        read_line(fd, buffer, line, 800) != ReadResult::Line)
      return std::nullopt;
    try {
      return json::parse(line);
    } catch (...) {
      return std::nullopt;
    }
  }

  void reset_command() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    have_command_ = false;
    expiry_stop_sent_ = false;
    latest_command_ = {};
  }

  void cancel_command() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    have_command_ = true;
    expiry_stop_sent_ = false;
    latest_command_ = {};
    latest_command_at_ = Clock::now();
  }

  // Forward fresh intent only. Send one zero on cancellation, then stop
  // resending so the daemon's independent deadman remains effective if this
  // process fails.
  void flush_command(int fd) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    multiple_publishers_ = cmd_sub_->get_publisher_count() > 1;
    const auto now = Clock::now();
    bool state_fresh, unsafe;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      state_fresh =
          last_state_mono_ns_ > 0 &&
          monotonic_ns() - last_state_mono_ns_ <= state_timeout_ * 1e9;
      unsafe = fallen_ || limp_ || policy_ == "held";
    }
    const bool blocked = multiple_publishers_ || unsafe || !state_fresh;
    const bool expired =
        have_command_ && now - latest_command_at_ >
                             std::chrono::duration<double>(command_timeout_);
    if (!have_command_ || blocked || expired) {
      if (have_command_ && !expiry_stop_sent_) {
        const json stop{{"jsonrpc", "2.0"},
                        {"method", "robot.move"},
                        {"params", {{"vx", 0.0}, {"vy", 0.0}, {"vyaw", 0.0}}}};
        write_all(fd, stop.dump() + "\n", 10);
        expiry_stop_sent_ = true;
      }
      have_command_ = false;
      return;
    }
    const json message{{"jsonrpc", "2.0"},
                       {"method", "robot.move"},
                       {"params",
                        {{"vx", latest_command_.vx},
                         {"vy", 0.0},
                         {"vyaw", latest_command_.vyaw}}}};
    if (!write_all(fd, message.dump() + "\n", 10)) {
      have_command_ = false;
    }
  }

  // Trunk poses are measured in the IMU's gravity-aligned odometry world.
  // Split planar navigation pose from height/attitude without copying head FK.
  void publish_state(const json &s) {
    const auto t = s.value("t_ns", std::uint64_t{0});
    if (t == 0) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      missing_state_timestamp_ = true;
      return;
    }
    if (t > static_cast<std::uint64_t>(INT64_MAX))
      return;
    const auto sample_age_ns = monotonic_ns() - static_cast<std::int64_t>(t);
    if (sample_age_ns < 0 || sample_age_ns > state_timeout_ * 1e9) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      missing_state_timestamp_ = false;
      return;
    }
    if (!s.contains("safety") || !s["safety"].is_object() ||
        !s["safety"].contains("fallen") || !s["safety"].contains("limp") ||
        !s.contains("policy") || !s.contains("odom"))
      return;
    const auto stamp = stamp_for(t);
    if (stamp.nanoseconds() > get_clock()->now().nanoseconds() + 100000000LL) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      missing_state_timestamp_ = true;
      return;
    }
    const auto odom = s.value("odom", json::object());
    const auto pos = odom.value("position", json::array());
    if (!finite3(pos) || !odom.contains("yaw") ||
        !std::isfinite(odom.at("yaw").get<double>()))
      return;
    if (previous_odom_ && t <= previous_odom_->t_ns)
      return;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      missing_state_timestamp_ = false;
      last_state_mono_ns_ = static_cast<std::int64_t>(t);
      fallen_ = s.value("safety", json::object()).value("fallen", false);
      limp_ = s.value("safety", json::object()).value("limp", false);
      policy_ = s.value("policy", std::string("unknown"));
    }
    const double yaw = odom.at("yaw").get<double>();
    const double z = pos[2].get<double>();
    if (previous_odom_) {
      const auto dt = static_cast<double>(t - previous_odom_->t_ns) / 1e9;
      if (dt > 0.001 && dt < 1.0) {
        const double dx = pos[0].get<double>() - previous_odom_->x;
        const double dy = pos[1].get<double>() - previous_odom_->y;
        const double delta_yaw =
            std::remainder(yaw - previous_odom_->yaw, 2.0 * M_PI);
        const double c = std::cos(yaw), sn = std::sin(yaw);
        const std::array<double, 3> measured{
            (c * dx + sn * dy) / dt, (-sn * dx + c * dy) / dt, delta_yaw / dt};
        for (std::size_t i = 0; i < 3; ++i)
          filtered_velocity_[i] =
              0.7 * filtered_velocity_[i] + 0.3 * measured[i];
      }
    }
    previous_odom_ =
        OdomSample{pos[0].get<double>(), pos[1].get<double>(), yaw, t};
    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = "odom";
    out.child_frame_id = "base_footprint";
    out.pose.pose.position.x = pos[0].get<double>();
    out.pose.pose.position.y = pos[1].get<double>();
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    out.pose.pose.orientation = tf2::toMsg(q);
    out.pose.covariance[0] = out.pose.covariance[7] = 0.01;
    out.pose.covariance[14] = 1e6;
    out.pose.covariance[21] = 1e6;
    out.pose.covariance[28] = 1e6;
    out.pose.covariance[35] = 0.04;
    out.twist.twist.linear.x = filtered_velocity_[0];
    out.twist.twist.linear.y = filtered_velocity_[1];
    out.twist.twist.angular.z = filtered_velocity_[2];
    out.twist.covariance[0] = out.twist.covariance[7] = 0.04;
    out.twist.covariance[14] = out.twist.covariance[21] =
        out.twist.covariance[28] = 1e6;
    out.twist.covariance[35] = 0.09;
    odom_pub_->publish(out);
    geometry_msgs::msg::TransformStamped planar;
    planar.header = out.header;
    planar.child_frame_id = "base_footprint";
    planar.transform.translation.x = out.pose.pose.position.x;
    planar.transform.translation.y = out.pose.pose.position.y;
    planar.transform.rotation = out.pose.pose.orientation;
    std::vector<geometry_msgs::msg::TransformStamped> transforms{planar};

    double roll = 0, pitch = 0, relative_yaw = 0;
    const bool have_orientation = s.contains("imu") &&
                                  s.at("imu").contains("quat") &&
                                  valid_quaternion(s.at("imu").at("quat"));
    if (have_orientation) {
      auto trunk_q = quaternion(s.at("imu").at("quat"));
      tf2::Quaternion world_trunk(trunk_q.x, trunk_q.y, trunk_q.z, trunk_q.w);
      tf2::Quaternion world_footprint;
      world_footprint.setRPY(0, 0, yaw);
      const auto relative = world_footprint.inverse() * world_trunk;
      tf2::Matrix3x3(relative).getRPY(roll, pitch, relative_yaw);
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = "base_link";
      imu.orientation = trunk_q;
      const auto gyro = s.at("imu").value("gyro", json::array());
      if (finite3(gyro)) {
        imu.angular_velocity.x = gyro[0].get<double>();
        imu.angular_velocity.y = gyro[1].get<double>();
        imu.angular_velocity.z = gyro[2].get<double>();
      }
      imu.linear_acceleration_covariance[0] = -1.0;
      imu.orientation_covariance[0] = imu.orientation_covariance[4] =
          imu.orientation_covariance[8] = 0.02;
      imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] =
          imu.angular_velocity_covariance[8] = 0.03;
      imu_pub_->publish(imu);
    }
    if (have_orientation) {
      geometry_msgs::msg::TransformStamped trunk;
      trunk.header.stamp = stamp;
      trunk.header.frame_id = "base_footprint";
      trunk.child_frame_id = "base_link";
      trunk.transform.translation.z = z;
      tf2::Quaternion trunk_rotation;
      trunk_rotation.setRPY(roll, pitch, relative_yaw);
      trunk.transform.rotation = tf2::toMsg(trunk_rotation);
      transforms.push_back(trunk);
    }

    if (have_orientation && s.contains("frames")) {
      for (const auto &entry :
           {std::pair<std::string, std::string>("tof", "tof"),
            {"camera", "camera"}}) {
        if (!s.at("frames").contains(entry.first))
          continue;
        const auto pose = s.at("frames").at(entry.first);
        if (!finite3(pose.value("pos", json::array())) ||
            !pose.contains("quat") || !valid_quaternion(pose.at("quat")))
          continue;
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "base_link";
        tf.child_frame_id = entry.second;
        const auto p = pose.at("pos");
        tf.transform.translation.x = p[0];
        tf.transform.translation.y = p[1];
        tf.transform.translation.z = p[2];
        tf.transform.rotation = quaternion(pose.at("quat"));
        transforms.push_back(tf);
      }
    }
    tf_broadcaster_.sendTransform(transforms);
    publish_joints(s, stamp);
    if (have_orientation)
      publish_markers(s, stamp);
  }

  void publish_joints(const json &s, const rclcpp::Time &stamp) {
    if (!s.contains("joints") || !s.at("joints").is_array())
      return;
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.header.frame_id = "base_link";
    json model;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      model = model_;
    }
    if (model.contains("joint_names"))
      for (const auto &name : model["joint_names"])
        js.name.push_back(name.get<std::string>());
    for (const auto &v : s["joints"]) {
      if (!v.is_number() || !std::isfinite(v.get<double>()))
        return;
      js.position.push_back(v.get<double>());
    }
    if (js.name.size() != js.position.size())
      return;
    if (s.contains("velocities") && s["velocities"].is_array() &&
        s["velocities"].size() == js.position.size()) {
      bool valid = true;
      for (const auto &v : s["velocities"]) {
        if (!v.is_number() || !std::isfinite(v.get<double>())) {
          valid = false;
          break;
        }
      }
      if (valid)
        for (const auto &v : s["velocities"])
          js.velocity.push_back(v.get<double>());
    }
    joint_pub_->publish(js);
  }

  void publish_markers(const json &s, const rclcpp::Time &stamp) {
    json model;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      model = model_;
    }
    if (!model.contains("skeleton") || !model["skeleton"].is_array() ||
        !s.contains("skeleton") || !s["skeleton"].is_array())
      return;
    visualization_msgs::msg::MarkerArray array;
    const auto &links = model["skeleton"];
    const auto &poses = s["skeleton"];
    if (links.size() != poses.size())
      return;
    for (std::size_t i = 0; i < links.size(); ++i) {
      const auto &pose = poses[i];
      if (!finite3(pose.value("pos", json::array())) ||
          !pose.contains("quat") || !valid_quaternion(pose.at("quat")))
        continue;
      const auto p = pose.at("pos");
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = "base_link";
      marker.ns = "skeleton";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = p[0];
      marker.pose.position.y = p[1];
      marker.pose.position.z = p[2];
      marker.pose.orientation = quaternion(pose.at("quat"));
      marker.scale.x = marker.scale.y = marker.scale.z = 0.035;
      marker.color.r = 0.95F;
      marker.color.g = 0.65F;
      marker.color.b = 0.12F;
      marker.color.a = 1.0F;
      marker.text = links[i].value("name", std::string());
      array.markers.push_back(marker);
    }
    if (s.contains("odom") &&
        finite3(s["odom"].value("position", json::array()))) {
      const auto position = s["odom"]["position"];
      geometry_msgs::msg::Point point;
      point.x = position[0];
      point.y = position[1];
      point.z = 0.0;
      trajectory_.push_back(point);
      if (trajectory_.size() > 3000)
        trajectory_.erase(trajectory_.begin(), trajectory_.begin() + 1000);
      visualization_msgs::msg::Marker path;
      path.header.stamp = stamp;
      path.header.frame_id = "odom";
      path.ns = "trajectory";
      path.id = 0;
      path.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path.action = visualization_msgs::msg::Marker::ADD;
      path.pose.orientation.w = 1.0;
      path.scale.x = 0.012;
      path.color.r = 0.1F;
      path.color.g = 0.8F;
      path.color.b = 0.95F;
      path.color.a = 0.9F;
      path.points = trajectory_;
      array.markers.push_back(path);
    }
    marker_pub_->publish(array);
  }

  void tof_loop() {
    while (!stopping_) {
      const int fd = connect_unix(tof_socket_);
      if (fd < 0) {
        std::this_thread::sleep_for(kReconnectDelay);
        continue;
      }
      const json subscribe{{"jsonrpc", "2.0"},
                           {"id", 1},
                           {"method", "tof.stream"},
                           {"params", json::object()}};
      std::string buffer, line;
      if (!write_all(fd, subscribe.dump() + "\n") ||
          read_line(fd, buffer, line, 800) != ReadResult::Line) {
        ::close(fd);
        std::this_thread::sleep_for(kReconnectDelay);
        continue;
      }
      try {
        if (!json::parse(line)
                 .value("result", json::object())
                 .value("accepted", false))
          throw std::runtime_error("ToF refused");
      } catch (...) {
        ::close(fd);
        std::this_thread::sleep_for(kReconnectDelay);
        continue;
      }
      connected_tof_ = true;
      while (!stopping_) {
        const auto read = read_line(fd, buffer, line, 250);
        if (read == ReadResult::Timeout)
          continue;
        if (read != ReadResult::Line)
          break;
        try {
          const auto m = json::parse(line);
          if (m.value("method", std::string()) == "tof.frame")
            publish_tof(m.at("params"));
        } catch (const std::exception &e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Invalid ToF frame: %s", e.what());
        }
      }
      connected_tof_ = false;
      ::close(fd);
      std::this_thread::sleep_for(kReconnectDelay);
    }
  }

  void publish_tof(const json &frame) {
    if (frame.value("t_ns", std::uint64_t{0}) == 0) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      missing_tof_timestamp_ = true;
      return;
    }
    const auto t = frame.value("t_ns", std::uint64_t{0});
    if (t > static_cast<std::uint64_t>(INT64_MAX))
      return;
    const auto age_ns = monotonic_ns() - static_cast<std::int64_t>(t);
    std::int64_t previous_tof_ns;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      previous_tof_ns = last_tof_mono_ns_;
    }
    if (age_ns < 0 || age_ns > tof_timeout_ * 1e9 ||
        static_cast<std::int64_t>(t) <= previous_tof_ns)
      return;
    json model;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      model = model_;
      missing_tof_timestamp_ = false;
    }
    const auto rows = frame.value("rows", 0);
    const auto cols = frame.value("cols", 0);
    const auto distances = frame.value("distance_mm", json::array());
    const auto statuses = frame.value("status", json::array());
    const auto beams = model.value("tof_beams", json::array());
    if (rows <= 0 || cols <= 0 ||
        distances.size() != static_cast<std::size_t>(rows * cols) ||
        statuses.size() != distances.size() || beams.size() != distances.size())
      return;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp_for(frame.value("t_ns", std::uint64_t{0}));
    cloud.header.frame_id = "tof";
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    std::vector<std::array<float, 3>> points;
    std::uint32_t no_target = 0, failed = 0;
    for (std::size_t i = 0; i < distances.size(); ++i) {
      const auto distance = distances[i].get<std::int16_t>();
      const auto status = statuses[i].get<std::uint8_t>();
      const auto &beam = beams[i];
      if (status == 255) {
        ++no_target;
        continue;
      }
      if (status != 5 && status != 9) {
        ++failed;
        continue;
      }
      if (!microduck_ros_bridge::tof_valid_return(status, distance) ||
          !finite3(beam)) {
        ++failed;
        continue;
      }
      const double range = static_cast<double>(distance) / 1000.0;
      points.push_back({static_cast<float>(beam[0].get<double>() * range),
                        static_cast<float>(beam[1].get<double>() * range),
                        static_cast<float>(beam[2].get<double>() * range)});
    }
    modifier.resize(points.size());
    if (!points.empty()) {
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"),
          z(cloud, "z");
      for (const auto &point : points) {
        *x = point[0];
        *y = point[1];
        *z = point[2];
        ++x;
        ++y;
        ++z;
      }
    }
    points_pub_->publish(cloud);
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_tof_mono_ns_ = static_cast<std::int64_t>(t);
      missing_tof_timestamp_ = false;
      tof_no_target_zones_ = no_target;
      tof_failed_zones_ = failed;
    }
  }

  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "microduck_bridge";
    status.hardware_id = robot_socket_;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "connected";
    bool state_fresh, tof_fresh, fallen, limp, missing_timestamp;
    std::string policy;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      state_fresh =
          last_state_mono_ns_ > 0 &&
          monotonic_ns() - last_state_mono_ns_ <= state_timeout_ * 1e9;
      tof_fresh = last_tof_mono_ns_ > 0 &&
                  monotonic_ns() - last_tof_mono_ns_ <= tof_timeout_ * 1e9;
      fallen = fallen_;
      limp = limp_;
      policy = policy_;
      missing_timestamp = missing_state_timestamp_ || missing_tof_timestamp_;
    }
    if (!connected_robot_ || !state_fresh || !connected_tof_ || !tof_fresh ||
        multiple_publishers_ || fallen || limp || policy == "held" ||
        missing_timestamp) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = !connected_robot_ ? "robot socket disconnected"
                       : !state_fresh    ? "robot state stale"
                       : !connected_tof_ ? "ToF socket disconnected"
                       : !tof_fresh      ? "ToF data stale"
                       : multiple_publishers_
                           ? "multiple cmd_vel publishers; commands inhibited"
                       : fallen           ? "robot fallen"
                       : limp             ? "robot limp"
                       : policy == "held" ? "robot policy held"
                                          : "daemon timestamp unavailable";
    }
    auto add = [&status](const std::string &key, const std::string &value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };
    add("robot_connected", connected_robot_ ? "true" : "false");
    add("state_fresh", state_fresh ? "true" : "false");
    add("tof_connected", connected_tof_ ? "true" : "false");
    add("tof_fresh", tof_fresh ? "true" : "false");
    add("policy", policy);
    add("fallen", fallen ? "true" : "false");
    add("limp", limp ? "true" : "false");
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      add("tof_no_target_zones", std::to_string(tof_no_target_zones_));
      add("tof_failed_zones", std::to_string(tof_failed_zones_));
    }
    add("cmd_vel_publishers", std::to_string(cmd_sub_->get_publisher_count()));
    add("rejected_commands", std::to_string(rejected_commands_.load()));
    array.status.push_back(status);
    diagnostic_pub_->publish(array);
  }

  std::string robot_socket_, tof_socket_;
  double command_timeout_{}, max_forward_{}, max_yaw_{}, state_timeout_{},
      tof_timeout_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::thread robot_thread_, tof_thread_;
  std::atomic<bool> stopping_{false}, connected_robot_{false},
      connected_tof_{false}, fallen_{false}, multiple_publishers_{false};
  std::atomic<std::uint64_t> rejected_commands_{0};
  std::mutex command_mutex_;
  microduck_ros_bridge::Command latest_command_{};
  Clock::time_point latest_command_at_{};
  bool have_command_{false}, expiry_stop_sent_{false};
  json model_ = json::object();
  std::string policy_ = "unknown";
  std::mutex data_mutex_;
  std::int64_t monotonic_ros_offset_ns_{0};
  bool limp_{false}, missing_state_timestamp_{false},
      missing_tof_timestamp_{false};
  std::int64_t last_state_mono_ns_{0}, last_tof_mono_ns_{0};
  std::uint32_t tof_no_target_zones_{0}, tof_failed_zones_{0};
  struct OdomSample {
    double x, y, yaw;
    std::uint64_t t_ns;
  };
  std::optional<OdomSample> previous_odom_;
  std::array<double, 3> filtered_velocity_{};
  std::vector<geometry_msgs::msg::Point> trajectory_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MicroduckBridge>());
  rclcpp::shutdown();
  return 0;
}
