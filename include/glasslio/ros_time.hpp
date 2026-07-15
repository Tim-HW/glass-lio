#ifndef GLASSLIO_ROS_TIME_HPP
#define GLASSLIO_ROS_TIME_HPP

#include <rclcpp/rclcpp.hpp>

namespace glasslio
{

/// Full-precision timestamp in seconds from any message carrying a std_msgs
/// header. The original port used `float sec + nano/1e9`, which loses ~3 digits
/// of precision on Unix epoch time; rclcpp::Time keeps int64 nanoseconds.
template<typename MsgPtr>
inline double stamp_sec(const MsgPtr & msg)
{
  return rclcpp::Time(msg->header.stamp).seconds();
}

}  // namespace glasslio

#endif  // GLASSLIO_ROS_TIME_HPP
