#ifndef NAV2_COVERAGE__FULLMAP_POSE_CREATOR_HPP_
#define NAV2_COVERAGE__FULLMAP_POSE_CREATOR_HPP_

#include <string>
#include <memory>
#include <vector>
#include "nav2_coverage_core/poses_creator.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/callback_group.hpp"

namespace nav2_coverage
{

class FullmapPoseCreator : public nav2_coverage_core::PosesCreator
{
public:

    FullmapPoseCreator(): PosesCreator() {}

    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name) override;

    void cleanup() override {}

    void activate() override {}

    void deactivate() override {}

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
    void onMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void onCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    std::mutex grid_mutex_;

    std::string map_topic_;
    std::string costmap_topic_;
    double grid_step_x_;
    double grid_step_y_;
    bool allow_unknown_map_;
    bool allow_unknown_costmap_;
    bool skip_outside_costmap_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}

#endif  // NAV2_COVERAGE__FULLMAP_POSE_CREATOR_HPP_