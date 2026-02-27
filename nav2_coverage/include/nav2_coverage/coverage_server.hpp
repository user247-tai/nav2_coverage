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

#ifndef NAV2_COVERAGE__COVERAGE_SERVER_HPP_
#define NAV2_COVERAGE__COVERAGE_SERVER_HPP_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_coverage_msgs/action/cover_map.hpp"
#include "nav2_coverage_core/poses_creator.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "rclcpp/callback_group.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_coverage
{

using Compute = nav2_msgs::action::ComputePathThroughPoses;
using Follow = nav2_msgs::action::FollowPath;
using PosesCreatorMap = std::unordered_map<std::string, nav2_coverage_core::PosesCreator::Ptr>;

class CoverageServer : public nav2_util::LifecycleNode
{
public:
    explicit CoverageServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~CoverageServer();

protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
    // Action goal handling
    template<typename ActionT>
    void getPreemptedGoalIfRequested(typename std::shared_ptr<const typename ActionT::Goal> & goal, const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server);
    
    template<typename ActionT>
    bool checkAndWarnIfCancelled(std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server);

    template<typename ActionT>
    bool checkAndWarnIfPreempted(std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server);

    // Coverage logic
    void runPipeline();

    bool coverMap(geometry_msgs::msg::PoseArray poses);

    // Helpers
    size_t findNearestIndex(const std::vector<geometry_msgs::msg::PoseStamped> & goals, const geometry_msgs::msg::PoseStamped & robot_pose);
    
    bool waitForData(double timeout_sec);
    
    nav_msgs::msg::Path downsamplePath(const nav_msgs::msg::Path & in) const;
    
    bool findPosesCreatorId(const std::string & c_name, std::string & name);
    
    void stopTimer();

    void updateRobotPose();

    // Action clients
    rclcpp_action::Client<Compute>::SharedPtr compute_client_;
    rclcpp_action::Client<Follow>::SharedPtr follow_client_;

    // Publishers (debug)
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr graph_nodes_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr update_pose_cb_group_;

    // Action server
    using Action = nav2_coverage_msgs::action::CoverMap;
    using ActionServer = nav2_util::SimpleActionServer<Action>;
    std::unique_ptr<ActionServer> action_server_;

    // Pose creator plugins
    pluginlib::ClassLoader<nav2_coverage_core::PosesCreator> poses_creator_loader_;
    PosesCreatorMap poses_creators_;
    std::vector<std::string> default_poses_creator_ids_;
    std::vector<std::string> default_poses_creator_types_;
    std::vector<std::string> poses_creator_ids_;
    std::vector<std::string> poses_creator_types_;
    std::string poses_creator_ids_concat_, current_poses_creator_;

    // Parameters components
    std::string graph_nodes_topic_;
    std::string debug_path_topic_;
    std::string compute_action_name_;
    std::string follow_action_name_;
    bool set_start_from_first_pose_{false};
    std::string planner_id_;
    std::string controller_id_;
    std::string goal_checker_id_;
    int retries_on_failure_{0};
    int recovery_obstacle_max_cost_{100};
    double poses_timeout_sec_{5.0};
    double compute_timeout_sec_{30.0};
    double follow_timeout_sec_{0.0}; 
    int downsample_keep_every_n_{0};
    double downsample_min_dist_{0.0};

    // Costmap components
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
    nav2_costmap_2d::Costmap2D * costmap_;
    std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> collision_checker_;

    // Other components
    std::mutex goal_list_mutex_;
    std::vector<geometry_msgs::msg::PoseStamped> goal_list_;
    size_t current_index_{0};
};

}

#endif  // NAV2_COVERAGE__COVERAGE_SERVER_HPP_