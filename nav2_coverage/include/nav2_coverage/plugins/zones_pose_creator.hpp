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
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef NAV2_COVERAGE__ZONES_POSE_CREATOR_HPP_
#define NAV2_COVERAGE__ZONES_POSE_CREATOR_HPP_

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <climits>
#include <numeric>
#include "nav2_coverage_core/poses_creator.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/callback_group.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

namespace nav2_coverage
{

struct ZoneRegion
{
  int zone_id{0};
  int region_index{0};
  std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>> nodes;
  double centroid_x{0.0};
  double centroid_y{0.0};
};

class ZonesPoseCreator : public nav2_coverage_core::PosesCreator
{
public:
  ZonesPoseCreator() : PosesCreator() {}

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::PoseArray createPoses(
    const std::string & order_mode,
    const bool enable_serpentine,
    const bool columns_left_to_right,
    const bool rows_bottom_to_top,
    const float downsample_step_x,
    const float downsample_step_y,
    const int map_occ_threshold,
    const int costmap_occ_threshold) override;

  std::vector<geometry_msgs::msg::Pose> orderSerpentine(
    const std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>> & nodes,
    int dsx_cells,
    int dsy_cells,
    const std::string & order_mode,
    bool columns_left_to_right,
    bool rows_bottom_to_top) override;

  bool isDataReady() override;

protected:
  void onZonesMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  std::vector<ZoneRegion> extractByZoneId(
    const nav_msgs::msg::OccupancyGrid & zones_map);

  std::vector<ZoneRegion> runConnectedComponentLabeling(
    const nav_msgs::msg::OccupancyGrid & zones_map);

  std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>> processRegion(
    const ZoneRegion & region,
    const nav2_coverage_core::GridMeta & zones_meta,
    const nav_msgs::msg::OccupancyGrid & costmap,
    const nav2_coverage_core::GridMeta & cost_meta,
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map,
    int step_x_cells,
    int step_y_cells,
    int dsx_cells,
    int dsy_cells,
    int map_occ_threshold,
    int costmap_occ_threshold);

  std::vector<int> determineZoneOrder(
    const std::vector<ZoneRegion> & regions,
    const std::string & frame_id);

  bool getRobotPose(
    const std::string & frame_id,
    geometry_msgs::msg::PoseStamped & pose);

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr zones_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  // Data
  nav_msgs::msg::OccupancyGrid::SharedPtr zones_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::mutex data_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Node & callback group
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // Parameters
  std::string zones_map_topic_;
  std::string costmap_topic_;
  std::string map_topic_;
  double grid_step_x_{0.05};
  double grid_step_y_{0.05};
  std::string zone_map_encoding_{"zone_id"};
  std::string zone_grouping_mode_{"by_id"};
  std::string zone_order_mode_{"ascending"};
  std::vector<int64_t> custom_zone_order_;
  int min_zone_cells_{10};
  bool allow_unknown_map_{false};
  bool allow_unknown_costmap_{false};
  bool skip_outside_costmap_{true};
  bool use_map_topic_{true};
  std::string robot_base_frame_{"base_link"};
};

}  // namespace nav2_coverage

#endif  // NAV2_COVERAGE__ZONES_POSE_CREATOR_HPP_
