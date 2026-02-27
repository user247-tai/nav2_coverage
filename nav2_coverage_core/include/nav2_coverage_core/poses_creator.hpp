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

#ifndef NAV2_COVERAGE_CORE__POSES_CREATOR_HPP_
#define NAV2_COVERAGE_CORE__POSES_CREATOR_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/logger.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_coverage_msgs/action/cover_map.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <cmath>

using CoverAllMap = nav2_coverage_msgs::action::CoverMap;
using GoalHandleCoverAllMap = rclcpp_action::ServerGoalHandle<CoverAllMap>;

namespace nav2_coverage_core
{

struct GridMeta
{
    std::string frame_id;
    double resolution{0.05};
    int width{0};
    int height{0};
    double origin_x{0.0};
    double origin_y{0.0};
};

class PosesCreator
{
public:
    typedef std::shared_ptr<nav2_coverage_core::PosesCreator> Ptr;

    struct Options
    {
        // Raw scan stride (meters)
        double grid_step_x{0.05};
        double grid_step_y{0.05};

        // Filters
        bool allow_unknown_map{false};
        bool allow_unknown_costmap{false};
        bool skip_outside_costmap{true};
    };

    virtual ~PosesCreator() {}

    virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name) = 0;

    virtual void cleanup() = 0;

    virtual void activate() = 0;

    virtual void deactivate() = 0;

    virtual geometry_msgs::msg::PoseArray createPoses(
        const std::string & order_mode /* "rows" or "columns" */,
        const bool enable_serpentine,
        const bool columns_left_to_right,
        const bool rows_bottom_to_top,
        const float downsample_step_x,
        const float downsample_step_y,
        const int map_occ_threshold,
        const int costmap_occ_threshold) = 0;

    virtual std::vector<geometry_msgs::msg::Pose> orderSerpentine(
        const std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>> & nodes,
        int dsx_cells,
        int dsy_cells,
        const std::string & order_mode,
        bool columns_left_to_right,
        bool rows_bottom_to_top) = 0;

    inline GridMeta metaFromGrid(const nav_msgs::msg::OccupancyGrid & grid) {
        GridMeta m;
        m.frame_id = grid.header.frame_id.empty() ? "map" : grid.header.frame_id;
        m.resolution = static_cast<double>(grid.info.resolution);
        m.width = static_cast<int>(grid.info.width);
        m.height = static_cast<int>(grid.info.height);
        m.origin_x = static_cast<double>(grid.info.origin.position.x);
        m.origin_y = static_cast<double>(grid.info.origin.position.y);
        return m;
    }

    virtual bool isDataReady() = 0;

    inline bool worldToGrid(double wx, double wy, const GridMeta & meta, int & mx, int & my) {
        mx = static_cast<int>(std::floor((wx - meta.origin_x) / meta.resolution));
        my = static_cast<int>(std::floor((wy - meta.origin_y) / meta.resolution));
        if (mx < 0 || my < 0 || mx >= meta.width || my >= meta.height) {
            return false;
        }
        return true;
    }

    inline std::string getName() {return name_;}

protected:
    std::string name_;
    double wait_data_timeout_sec_{5.0};
};

}

#endif // NAV2_COVERAGE_CORE__POSES_CREATOR_HPP_
