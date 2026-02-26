#ifndef NAV2_COVERAGE__UTILS_HPP_
#define NAV2_COVERAGE__UTILS_HPP_

#include <string>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_coverage
{

inline std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

inline double distXY(const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b)
{
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
}

builtin_interfaces::msg::Time toTimeMsg(const rclcpp::Time & t)
{
    builtin_interfaces::msg::Time msg;
    const int64_t ns = t.nanoseconds();
    msg.sec = static_cast<int32_t>(ns / 1000000000LL);
    msg.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return msg;
}

}

#endif // NAV2_COVERAGE__UTILS_HPP_