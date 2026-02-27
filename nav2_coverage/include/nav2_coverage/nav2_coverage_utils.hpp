// MIT License

// Copyright (c) 2026 Nguyen Thanh Tai

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef NAV2_COVERAGE__UTILS_HPP_
#define NAV2_COVERAGE__UTILS_HPP_

#include <string>
#include <algorithm>
#include <cctype>
#include <cmath>
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_coverage
{

inline std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), 
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

inline double distXY(const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b)
{
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
}

inline builtin_interfaces::msg::Time toTimeMsg(const rclcpp::Time & t)
{
    builtin_interfaces::msg::Time msg;
    const int64_t ns = t.nanoseconds();
    constexpr int64_t kBillion = 1000000000LL;
    int64_t sec = ns / kBillion;
    int64_t nanosec = ns % kBillion;
    if (nanosec < 0) {
        --sec;
        nanosec += kBillion;
    }
    msg.sec = static_cast<int32_t>(sec);
    msg.nanosec = static_cast<uint32_t>(nanosec);
    return msg;
}

}

#endif // NAV2_COVERAGE__UTILS_HPP_