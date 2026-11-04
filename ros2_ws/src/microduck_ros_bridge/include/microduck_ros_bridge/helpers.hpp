#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace microduck_ros_bridge {

/// Robot-frame forward speed (m/s) and left-positive yaw rate (rad/s).
struct Command {
  double vx{};
  double vyaw{};
};

/// Future, non-finite and expired sample ages cannot authorise movement.
inline bool command_fresh(double age_seconds, double timeout_seconds) {
  return std::isfinite(age_seconds) && timeout_seconds > 0.0 &&
         age_seconds >= 0.0 && age_seconds <= timeout_seconds;
}

/// Reject non-finite requests and clamp forward/yaw; lateral motion is
/// discarded.
inline std::optional<Command> validated_command(double vx, double vy, double wz,
                                                double max_forward,
                                                double max_yaw) {
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) ||
      max_forward <= 0.0 || max_yaw <= 0.0)
    return std::nullopt;
  return Command{std::clamp(vx, -max_forward, max_forward),
                 std::clamp(wz, -max_yaw, max_yaw)};
}

/// Match tof::Frame::zone: ST status 5/9 and strictly positive millimetres.
inline bool tof_valid_return(std::uint8_t status, std::int16_t distance_mm) {
  return (status == 5 || status == 9) && distance_mm > 0;
}

/// Apply the startup wall/monotonic offset; absent or overflowing stamps fail.
inline std::optional<std::int64_t> monotonic_to_ros_ns(std::uint64_t sample_ns,
                                                       std::int64_t offset_ns) {
  if (sample_ns == 0 || sample_ns > static_cast<std::uint64_t>(INT64_MAX))
    return std::nullopt;
  const auto sample = static_cast<std::int64_t>(sample_ns);
  if ((offset_ns > 0 && sample > INT64_MAX - offset_ns) ||
      (offset_ns < 0 && sample < INT64_MIN - offset_ns))
    return std::nullopt;
  const auto result = sample + offset_ns;
  return result > 0 ? std::optional<std::int64_t>(result) : std::nullopt;
}

} // namespace microduck_ros_bridge
