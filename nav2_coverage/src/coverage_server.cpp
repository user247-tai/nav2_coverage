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

#include "nav2_coverage/coverage_server.hpp"
#include "nav2_coverage/nav2_coverage_utils.hpp"
#include "nav2_coverage_core/coverage_exceptions.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_coverage
{

template<typename GoalT>
auto setPlannerId(GoalT & goal, const std::string & planner_id, int) -> decltype(goal.planner_id = planner_id, void())
{
    goal.planner_id = planner_id;
}
template<typename GoalT>
void setPlannerId(GoalT &, const std::string &, long) {}

template<typename GoalT>
auto setComputeGoals(
    GoalT & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & list,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id,
    int) -> decltype(goal.goals.goals = list, void())
{
    goal.goals.header.stamp = stamp;
    goal.goals.header.frame_id = frame_id;
    goal.goals.goals = list;
}
template<typename GoalT>
auto setComputeGoals(
    GoalT & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & list,
    const builtin_interfaces::msg::Time &,
    const std::string &,
    long) -> decltype(goal.goals = list, void())
{
    goal.goals = list;
}

template<typename GoalT>
auto setControllerId(GoalT & goal, const std::string & id, int)
    -> decltype(goal.controller_id = id, void())
{
  goal.controller_id = id;
}
template<typename GoalT>
void setControllerId(GoalT &, const std::string &, long) {}

template<typename GoalT>
auto setGoalCheckerId(GoalT & goal, const std::string & id, int) -> decltype(goal.goal_checker_id = id, void())
{
    goal.goal_checker_id = id;
}
template<typename GoalT>
void setGoalCheckerId(GoalT &, const std::string &, long) {}

CoverageServer::CoverageServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("coverage_server", "", options),
  poses_creator_loader_("nav2_coverage_core", "nav2_coverage_core::PosesCreator"),
  default_poses_creator_ids_{"poses_creator"},
  default_poses_creator_types_{"nav2_coverage::FullmapPoseCreator"}
{
    RCLCPP_INFO(get_logger(), "Creating coverage server");

    declare_parameter("poses_creator_plugins", default_poses_creator_ids_);

    // topics
    declare_parameter("graph_nodes_topic", "/graph_nodes");
    declare_parameter("debug_path_topic", "/debug/computed_path");

    // action clients
    declare_parameter("compute_action_name", "/compute_path_through_poses");
    declare_parameter("follow_action_name", "/follow_path");

    // behavior
    declare_parameter("set_start_from_first_pose", true);
    declare_parameter("planner_id", "");
    declare_parameter("controller_id", "");
    declare_parameter("goal_checker_id", "");
    declare_parameter("retries_on_failure", 0);
    declare_parameter("recovery_obstacle_max_cost", 100);

    // timeouts
    declare_parameter("poses_timeout_sec", 5.0);
    declare_parameter("compute_timeout_sec", 30.0);
    declare_parameter("follow_timeout_sec", 0.0);  // 0 => no timeout

    // downsample computed path
    declare_parameter("downsample_keep_every_n", 0);
    declare_parameter("downsample_min_dist", 0.0);

    // Setup the global costmap
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "global_costmap", std::string{get_namespace()},
        get_parameter("use_sim_time").as_bool());
}

CoverageServer::~CoverageServer()
{
    stopTimer();
    costmap_thread_.reset();
    poses_creators_.clear();
}

nav2_util::CallbackReturn CoverageServer::on_configure(const rclcpp_lifecycle::State & state)
{
    auto node = shared_from_this();

    RCLCPP_INFO(get_logger(), "Configuring coverage interface");

    RCLCPP_INFO(get_logger(), "Getting poses creator plugins..");

    get_parameter("poses_creator_plugins", poses_creator_ids_);
    if (poses_creator_ids_ == default_poses_creator_ids_) {
        for (size_t i = 0; i < default_poses_creator_ids_.size(); ++i) {
        nav2_util::declare_parameter_if_not_declared(
            node, default_poses_creator_ids_[i] + ".plugin",
            rclcpp::ParameterValue(default_poses_creator_types_[i]));
        }
    }

    poses_creator_types_.resize(poses_creator_ids_.size());

    get_parameter("graph_nodes_topic", graph_nodes_topic_);
    get_parameter("debug_path_topic", debug_path_topic_);
    get_parameter("compute_action_name", compute_action_name_);
    get_parameter("follow_action_name", follow_action_name_);
    get_parameter("set_start_from_first_pose", set_start_from_first_pose_);
    get_parameter("planner_id", planner_id_);
    get_parameter("controller_id", controller_id_);
    get_parameter("goal_checker_id", goal_checker_id_);
    get_parameter("retries_on_failure", retries_on_failure_);
    get_parameter("recovery_obstacle_max_cost", recovery_obstacle_max_cost_);
    get_parameter("poses_timeout_sec", poses_timeout_sec_);
    get_parameter("compute_timeout_sec", compute_timeout_sec_);
    get_parameter("follow_timeout_sec", follow_timeout_sec_);
    get_parameter("downsample_keep_every_n", downsample_keep_every_n_);
    get_parameter("downsample_min_dist", downsample_min_dist_);

    // Configure costmap
    costmap_ros_->configure();
    costmap_ = costmap_ros_->getCostmap();
    if (!costmap_ros_->getUseRadius()) {
        collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
    }
  
    // Launch a thread to run the costmap node
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

    for (size_t i = 0; i != poses_creator_ids_.size(); i++) {
        try {
            poses_creator_types_[i] = nav2_util::get_plugin_type_param(
                node, poses_creator_ids_[i]);
            nav2_coverage_core::PosesCreator::Ptr poses_creator =
                poses_creator_loader_.createUniqueInstance(poses_creator_types_[i]);
            RCLCPP_INFO(
                get_logger(), "Created poses_creator : %s of type %s",
                poses_creator_ids_[i].c_str(), poses_creator_types_[i].c_str());
            poses_creator->configure(node, poses_creator_ids_[i]);
            poses_creators_.insert({poses_creator_ids_[i], poses_creator});
        } catch (const std::exception & ex) {
            RCLCPP_FATAL(
                get_logger(),
                "Failed to create poses_creator. Exception: %s", ex.what());
            on_cleanup(state);
            return nav2_util::CallbackReturn::FAILURE;
        }
    }
    
    poses_creator_ids_concat_.clear();
    for (size_t i = 0; i != poses_creator_ids_.size(); i++) {
        poses_creator_ids_concat_ += poses_creator_ids_[i] + std::string(" ");
    }

    RCLCPP_INFO(get_logger(), "Coverage Server has %s poses generator available.", poses_creator_ids_concat_.c_str());

    // Debug publishers (latched)
    graph_nodes_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(graph_nodes_topic_, rclcpp::QoS(1).reliable().transient_local());
    debug_path_pub_ = create_publisher<nav_msgs::msg::Path>(debug_path_topic_, rclcpp::QoS(1).reliable().transient_local());

    // Action clients
    compute_client_ = rclcpp_action::create_client<Compute>(shared_from_this(), compute_action_name_);
    follow_client_ = rclcpp_action::create_client<Follow>(shared_from_this(), follow_action_name_);

    // Action server
    try {
        action_server_ = std::make_unique<ActionServer>(
        shared_from_this(),
        "cover_map",
        std::bind(&CoverageServer::runPipeline, this),
        nullptr,
        std::chrono::milliseconds(500),
        true /*spin thread*/, rcl_action_server_get_default_options(),
        false /*use_realtime_priority*//*soft realtime*/);
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(get_logger(), "Error creating action server! %s", e.what());
        on_cleanup(state);
        return nav2_util::CallbackReturn::FAILURE;
    }

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Activating");

    const auto costmap_ros_state = costmap_ros_->activate();
    if (costmap_ros_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        return nav2_util::CallbackReturn::FAILURE;
    }
    PosesCreatorMap::iterator it;
    for (it = poses_creators_.begin(); it != poses_creators_.end(); ++it) {
        it->second->activate();
    }
    graph_nodes_pub_->on_activate();
    debug_path_pub_->on_activate();
    action_server_->activate();
    
    // ***TODO***: Add callback for dynamic parameters 
    //  auto node = shared_from_this();
    //  dyn_params_handler_ = node->add_on_set_parameters_callback(
    //     std::bind(&ControllerServer::dynamicParametersCallback, this, _1));

    // create bond connection
    createBond();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");

    stopTimer();

    action_server_->deactivate();

    PosesCreatorMap::iterator it;
    for (it = poses_creators_.begin(); it != poses_creators_.end(); ++it) {
        it->second->deactivate();
    }

    graph_nodes_pub_->on_deactivate();
    debug_path_pub_->on_deactivate();

    costmap_ros_->deactivate();

    current_index_ = 0;
    // destroy bond connection
    destroyBond();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up");

    // Cleanup the helper classes
    PosesCreatorMap::iterator it;
    for (it = poses_creators_.begin(); it != poses_creators_.end(); ++it) {
        it->second->cleanup();
    }
    poses_creators_.clear();
    costmap_ros_->cleanup();

    // Release any allocated resources
    action_server_.reset();
    costmap_thread_.reset();

    graph_nodes_pub_.reset();
    debug_path_pub_.reset();
    compute_client_.reset();
    follow_client_.reset();
    
    costmap_ = nullptr;
    current_index_ = 0;

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
}

void CoverageServer::stopTimer() 
{
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
}

template<typename ActionT>
void CoverageServer::getPreemptedGoalIfRequested(typename std::shared_ptr<const typename ActionT::Goal> & goal, const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename ActionT>
bool CoverageServer::checkAndWarnIfCancelled(std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was cancelled. Cancelling cover_map action");
    return true;
  }
  return false;
}

template<typename ActionT>
bool CoverageServer::checkAndWarnIfPreempted(std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (action_server->is_preempt_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was preempted. Cancelling cover_map action");
    return true;
  }
  return false;
}

bool CoverageServer::findPosesCreatorId(const std::string & c_name, std::string & current_poses_creator)
{
    if (poses_creators_.find(c_name) == poses_creators_.end()) {
        if (poses_creators_.size() == 1 && c_name.empty()) {
        RCLCPP_WARN_ONCE(
            get_logger(), "No poses creator was specified in parameter 'current_poses_creator'."
            " Server will use only plugin loaded %s. "
            "This warning will appear once.", poses_creator_ids_concat_.c_str());
        current_poses_creator = poses_creators_.begin()->first;
        } else {
        RCLCPP_ERROR(
            get_logger(), "CoverageServer called with poses creator name %s in parameter"
            " 'current_poses_creator', which does not exist. Available poses creators are: %s.",
            c_name.c_str(), poses_creator_ids_concat_.c_str());
        return false;
        }
    } else {
        RCLCPP_DEBUG(get_logger(), "Selected poses creator: %s.", c_name.c_str());
        current_poses_creator = c_name;
    }

    return true;
}

nav_msgs::msg::Path CoverageServer::downsamplePath(const nav_msgs::msg::Path & in) const
{
    if (in.poses.empty()) return in;

    if (downsample_keep_every_n_ > 1) {
        nav_msgs::msg::Path out;
        out.header = in.header;
        out.poses.push_back(in.poses.front());
        for (size_t i = static_cast<size_t>(downsample_keep_every_n_); i < in.poses.size(); i += static_cast<size_t>(downsample_keep_every_n_)) {
            out.poses.push_back(in.poses[i]);
        }
        if (out.poses.back().pose.position.x != in.poses.back().pose.position.x || out.poses.back().pose.position.y != in.poses.back().pose.position.y) {
            out.poses.push_back(in.poses.back());
        }
        return out;
    }

    if (downsample_min_dist_ <= 0.0) return in;

    nav_msgs::msg::Path out;
    out.header = in.header;
    out.poses.push_back(in.poses.front());

    auto last = in.poses.front();
    for (size_t i = 1; i + 1 < in.poses.size(); ++i) {
        if (distXY(in.poses[i], last) >= downsample_min_dist_) {
            out.poses.push_back(in.poses[i]);
            last = in.poses[i];
        }
    }

    out.poses.push_back(in.poses.back());
    return out;
}

size_t CoverageServer::findNearestIndex(const std::vector<geometry_msgs::msg::PoseStamped> & goals, const geometry_msgs::msg::PoseStamped & robot_pose)
{
    size_t best_i = 0;
    double best_d = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < goals.size(); ++i) {
        const double dx = goals[i].pose.position.x - robot_pose.pose.position.x;
        const double dy = goals[i].pose.position.y - robot_pose.pose.position.y;
        const double d = std::hypot(dx, dy);
        if (d < best_d) {
            best_d = d;
            best_i = i;
        }
    }

    size_t index_offset = 0;

    try {
        const auto current_goal = (action_server_ != nullptr) ? action_server_->get_current_goal() : nullptr;
        if (current_goal) {
            const float step = (current_goal->order_mode == "columns") ? current_goal->downsample_step_y : current_goal->downsample_step_x;
            const float safe_step = (step > 0.0f) ? step : 1.0f;
            index_offset = static_cast<size_t>(std::ceil(1.0f / safe_step));
        }
    } catch(const std::exception & e) {
        stopTimer();
        RCLCPP_ERROR(this->get_logger(), "Unexpected error in findNearestIndex: %s", e.what());
    } 


    if ((best_i >= current_index_ + 1) && (best_i <= current_index_ + index_offset)) {
        return best_i;
    }
    return current_index_;
}

void CoverageServer::updateRobotPose()
{
    std::lock_guard<std::mutex> lk(goal_list_mutex_);
    if (!costmap_ros_ || (goal_list_.empty())) {
        return;
    }
    geometry_msgs::msg::PoseStamped pose;
    if (costmap_ros_->getRobotPose(pose)) {
        size_t temp_index = findNearestIndex(goal_list_, pose);
        size_t index_offset = 0;
        
        if (action_server_ != nullptr) {
            const auto current_goal = action_server_->get_current_goal();
            if (current_goal) {
                const float step = (current_goal->order_mode == "columns") ? current_goal->downsample_step_y : current_goal->downsample_step_x;
                const float safe_step = (step > 0.0f) ? step : 1.0f;
                index_offset = static_cast<size_t>(std::ceil(1.0f / safe_step));
            }
        }

        if ((temp_index >= current_index_ + 1) && (temp_index <= current_index_ + index_offset)) {
            current_index_ = temp_index;
        }
    }
}

bool CoverageServer::waitForData(double timeout_sec)
{
    const auto start = now();
    rclcpp::Rate r(20.0);

    while (rclcpp::ok()) {
        if (checkAndWarnIfCancelled(action_server_) || checkAndWarnIfPreempted(action_server_)) {
            RCLCPP_INFO(get_logger(), "Action server preempt/cancel requested. Stopping.");
            return false;
        }

        if (poses_creators_[current_poses_creator_]->isDataReady()) {
            return true;
        }

        if (timeout_sec > 0.0 && (now() - start).seconds() > timeout_sec) {
            throw nav2_coverage_core::TimeoutCreatePoses("Timed out waiting for data to create poses.");
        }

        r.sleep();
    }
    return false;
}


void CoverageServer::runPipeline()
{
    RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

    auto goal = action_server_->get_current_goal();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();

    if (!goal) {return;}

    if (!action_server_ || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
    }

    if (checkAndWarnIfCancelled(action_server_)) {
        action_server_->terminate_all();
        return;
    }
    getPreemptedGoalIfRequested(goal, action_server_);

    try {
        std::string pc_name = goal->poses_creator_id;
        std::string current_poses_creator;
        if (findPosesCreatorId(pc_name, current_poses_creator)) {
            current_poses_creator_ = current_poses_creator;
        } else {
            throw nav2_coverage_core::CoverageException("Failed to find poses creator name: " + pc_name);
        }

        if (!waitForData(poses_timeout_sec_)) {
            result->error_code = Action::Result::UNKNOWN;
            result->error_msg = "Goal have been canceled or preempted.";
            action_server_->terminate_all(result);
            return;
        }

        std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
        feedback->status_code = Action::Feedback::CREATING_POSES;
        action_server_->publish_feedback(feedback);

        geometry_msgs::msg::PoseArray poses = poses_creators_[current_poses_creator_]->createPoses(goal->order_mode,
                                                                                                    goal->enable_serpentine,
                                                                                                    goal->columns_left_to_right,
                                                                                                    goal->rows_bottom_to_top,
                                                                                                    goal->downsample_step_x,
                                                                                                    goal->downsample_step_y,
                                                                                                    goal->map_occ_threshold,
                                                                                                    goal->costmap_occ_threshold);

        poses.header.stamp = nav2_coverage::toTimeMsg(now());
        if (poses.header.frame_id.empty()) poses.header.frame_id = "map";

        if (poses.poses.empty()) {
            throw nav2_coverage_core::FailedToPlanCoverage("Poses creator returned an empty list of poses to cover.");
        }

        if (graph_nodes_pub_ && graph_nodes_pub_->is_activated()) {
            graph_nodes_pub_->publish(poses);
        }

        if (coverMap(std::move(poses))) {
            RCLCPP_INFO(get_logger(), "Coverage succeeded, setting result");
            stopTimer();
            result->error_code = Action::Result::SUCCESS;
            result->error_msg = "Coverage completed successfully.";
            action_server_->succeeded_current(result);
            return;
        }

        stopTimer();
        result->error_code = Action::Result::UNKNOWN;
        result->error_msg = "Goal have been canceled or preempted.";
        action_server_->terminate_all(result);
        return;

    } catch(nav2_coverage_core::TimeoutCreatePoses & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        stopTimer();
        result->error_code = Action::Result::TIMEOUT;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    } catch (nav2_coverage_core::FailedToPlanCoverage & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        stopTimer();
        result->error_code = Action::Result::FAIL_TO_PLAN;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    } catch (nav2_coverage_core::FailedToFollowPlanCoverage & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        stopTimer();
        result->error_code = Action::Result::FAIL_TO_FOLLOW_PATH;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    } catch (nav2_coverage_core::FailedToRecoverCoverage & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        stopTimer();
        result->error_code = Action::Result::FAIL_TO_RECOVER;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    } catch (nav2_coverage_core::CoverageException & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        stopTimer();
        result->error_code = Action::Result::UNKNOWN;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Unexpected error: %s", e.what());
        stopTimer();
        result->error_code = Action::Result::UNKNOWN;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    }
}

bool CoverageServer::coverMap(geometry_msgs::msg::PoseArray poses) 
{
    // --- Compute path to cover map ---
    std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
    feedback->status_code = Action::Feedback::PLANNING;
    action_server_->publish_feedback(feedback);

    if (!compute_client_->wait_for_action_server(std::chrono::seconds(2))) {
        throw nav2_coverage_core::FailedToPlanCoverage("ComputePathThroughPoses action server not available");
    }

    Compute::Goal compute_goal;
    const auto stamp_msg = toTimeMsg(now());
    const std::string frame_id = poses.header.frame_id;

    if (set_start_from_first_pose_) {
        geometry_msgs::msg::PoseStamped start;
        start.header.stamp = stamp_msg;
        start.header.frame_id = frame_id;
        start.pose = poses.poses.front();
        compute_goal.start = start;
    }

    setPlannerId(compute_goal, planner_id_, 0);

    {
        std::lock_guard<std::mutex> lk(goal_list_mutex_);
        goal_list_.resize(0);
        for (const auto & p : poses.poses) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.stamp = stamp_msg;
            ps.header.frame_id = frame_id;
            ps.pose = p;
            goal_list_.push_back(ps);
        }
        current_index_ = 0;
    }

    setComputeGoals(compute_goal, goal_list_, stamp_msg, frame_id, 0);
    auto compute_goal_future = compute_client_->async_send_goal(compute_goal);

    // wait goal-handle in small steps to allow preempt/cancel
    {
        const auto start_t = now();
        while (rclcpp::ok()) {
            if (checkAndWarnIfCancelled(action_server_) || checkAndWarnIfPreempted(action_server_)) {
                return false;
            }
            if (compute_goal_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                break;
            }
            if (compute_timeout_sec_ > 0.0 && (now() - start_t).seconds() > compute_timeout_sec_) {
                throw nav2_coverage_core::FailedToPlanCoverage("Timed out waiting for plan to be computed.");
            }
        }
    }

    auto compute_goal_handle = compute_goal_future.get();
    if (!compute_goal_handle) {
        throw nav2_coverage_core::FailedToPlanCoverage("ComputePathThroughPoses goal was rejected by the action server");
    }

    auto compute_result_future = compute_client_->async_get_result(compute_goal_handle);
    {
        const auto start_t = now();
        while (rclcpp::ok()) {
            if (checkAndWarnIfCancelled(action_server_) || checkAndWarnIfPreempted(action_server_)) {
                compute_client_->async_cancel_goal(compute_goal_handle);
                return false;
            }
            if (compute_result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
                break;
            }
            if (compute_timeout_sec_ > 0.0 && (now() - start_t).seconds() > compute_timeout_sec_) {
                compute_client_->async_cancel_goal(compute_goal_handle);
                throw nav2_coverage_core::FailedToPlanCoverage("Timed out waiting for plan to be computed.");
            }
        }
    }

    auto compute_wrapped = compute_result_future.get();
    if (compute_wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !compute_wrapped.result || compute_wrapped.result->path.poses.empty())
    {
        throw nav2_coverage_core::FailedToPlanCoverage("Failed to compute a path through the poses. Action result code: ");
    }

    // Create timer to update current index based on robot pose
    if (!timer_) {
        timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&CoverageServer::updateRobotPose, this), update_pose_cb_group_);
    }

    nav_msgs::msg::Path path = compute_wrapped.result->path;
    path.header.stamp = stamp_msg;
    path.header.frame_id = frame_id;

    nav_msgs::msg::Path ds_path = downsamplePath(path);
    ds_path.header.stamp = toTimeMsg(now());
    ds_path.header.frame_id = frame_id;

    if (debug_path_pub_ && debug_path_pub_->is_activated()) {
        debug_path_pub_->publish(ds_path);
    }

    // --- Follow path to cover map ---
    feedback->status_code = Action::Feedback::COVERING;
    action_server_->publish_feedback(feedback);

    if (!follow_client_->wait_for_action_server(std::chrono::seconds(2))) {
        throw nav2_coverage_core::FailedToFollowPlanCoverage("FollowPath action server not available");
    }

    Follow::Goal follow_goal;
    follow_goal.path = ds_path;
    setControllerId(follow_goal, controller_id_, 0);
    setGoalCheckerId(follow_goal, goal_checker_id_, 0);

    auto follow_goal_future = follow_client_->async_send_goal(follow_goal);

    {
        const auto start_t = now();
        while (rclcpp::ok()) {
            if (checkAndWarnIfCancelled(action_server_) || checkAndWarnIfPreempted(action_server_)) {
                return false;
            }
            if (follow_goal_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                break;
            }
            if (follow_timeout_sec_ > 0.0 && (now() - start_t).seconds() > follow_timeout_sec_) {
                throw nav2_coverage_core::FailedToFollowPlanCoverage("Timed out waiting for path following to complete.");
            }
        }
    }

    auto follow_goal_handle = follow_goal_future.get();
    if (!follow_goal_handle) {
        throw nav2_coverage_core::FailedToFollowPlanCoverage("FollowPath goal was rejected by the action server");
    }

    auto follow_result_future = follow_client_->async_get_result(follow_goal_handle);
    {
        const auto start_t = now();
        while (rclcpp::ok()) {
            if (checkAndWarnIfCancelled(action_server_) || checkAndWarnIfPreempted(action_server_)) {
                follow_client_->async_cancel_goal(follow_goal_handle);
                return false;
            }
            if (follow_result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
                break;
            }
            if (follow_timeout_sec_ > 0.0 && (now() - start_t).seconds() > follow_timeout_sec_) {
                follow_client_->async_cancel_goal(follow_goal_handle);
                throw nav2_coverage_core::FailedToFollowPlanCoverage("Timed out waiting for path following to complete.");
            }
        }
    }

    auto follow_wrapped = follow_result_future.get();

    // --- Recovery on failure to follow path ---
    if (follow_wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
        bool success_recovered = false;

        int retries_remaining = retries_on_failure_;
        while (retries_remaining == -1 || retries_remaining > 0) {
            bool continue_flag = false;
            if (retries_remaining > 0) {
                --retries_remaining; 
            }

            feedback->status_code = Action::Feedback::RECOVERING;
            action_server_->publish_feedback(feedback);

            geometry_msgs::msg::PoseStamped current_robot_pose;
            if (costmap_ros_->getRobotPose(current_robot_pose)) {
                // Create new path from current position to remaining goals
                std::vector<geometry_msgs::msg::PoseStamped> goals_snapshot;
                size_t current_index_snapshot = 0;
                {
                    std::lock_guard<std::mutex> lk(goal_list_mutex_);
                    goals_snapshot = goal_list_;
                    current_index_snapshot = current_index_;
                }

                std::vector<geometry_msgs::msg::PoseStamped> remaining_goal_list;
                remaining_goal_list.reserve(goals_snapshot.size() > current_index_snapshot ? (goals_snapshot.size() - current_index_snapshot + 1) : 1);
                remaining_goal_list.push_back(current_robot_pose);
                
                std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
                unsigned int mx = 0;
                unsigned int my = 0;
                bool use_radius = costmap_ros_->getUseRadius();
                
                for (size_t i = current_index_snapshot + 1; i < goals_snapshot.size(); ++i) {
                    unsigned int cost = nav2_costmap_2d::FREE_SPACE;
                    if (use_radius) {
                        if (costmap_->worldToMap(goals_snapshot[i].pose.position.x, goals_snapshot[i].pose.position.y, mx, my)) {
                            cost = costmap_->getCost(mx, my);
                        } else {
                            cost = nav2_costmap_2d::LETHAL_OBSTACLE;
                        }
                    } else {
                        nav2_costmap_2d::Footprint footprint = costmap_ros_->getRobotFootprint();
                        auto theta = tf2::getYaw(goals_snapshot[i].pose.orientation);
                        cost = static_cast<unsigned int>(collision_checker_->footprintCostAtPose(
                        goals_snapshot[i].pose.position.x, goals_snapshot[i].pose.position.y, theta, footprint));
                    }
                    if (cost >= static_cast<unsigned int>(recovery_obstacle_max_cost_)) {
                        continue;
                    } else {
                        remaining_goal_list.push_back(goals_snapshot[i]);
                    }
                }

                {
                    std::lock_guard<std::mutex> lk(goal_list_mutex_);
                    goal_list_.resize(remaining_goal_list.size());
                    goal_list_ = remaining_goal_list;
                    current_index_ = 0;
                }

                geometry_msgs::msg::PoseArray msg;
                msg.header.stamp = nav2_coverage::toTimeMsg(now());
                msg.header.frame_id = frame_id;
                for (const auto & p: goal_list_) {
                    msg.poses.push_back(p.pose);
                }

                if (graph_nodes_pub_ && graph_nodes_pub_->is_activated()) {
                    graph_nodes_pub_->publish(msg);
                }

                feedback->status_code = Action::Feedback::PLANNING;
                action_server_->publish_feedback(feedback);

                // Re-plan path
                if (!compute_client_->wait_for_action_server(std::chrono::seconds(2))) {
                    RCLCPP_WARN(get_logger(), "ComputePathThroughPoses action server not available during recovery");
                    continue;
                }

                Compute::Goal recovery_compute_goal;
                const auto recovery_stamp_msg = toTimeMsg(now());

                setPlannerId(recovery_compute_goal, planner_id_, 0);

                setComputeGoals(recovery_compute_goal, remaining_goal_list, recovery_stamp_msg, frame_id, 0);

                auto recovery_compute_goal_future = compute_client_->async_send_goal(recovery_compute_goal);

                // wait goal-handle in small steps to allow preempt/cancel
                continue_flag = false;
                {
                    const auto start_t = now();
                    while (rclcpp::ok()) {
                        if (checkAndWarnIfCancelled(action_server_) || checkAndWarnIfPreempted(action_server_)) {
                            return false;
                        }
                        if (recovery_compute_goal_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                            break;
                        }
                        if (compute_timeout_sec_ > 0.0 && (now() - start_t).seconds() > compute_timeout_sec_) {
                            RCLCPP_WARN(get_logger(), "ComputePathThroughPoses action server not available during recovery");
                            continue_flag = true;
                            break;
                        }
                    }
                }if (continue_flag) {continue;}

                auto recovery_compute_goal_handle = recovery_compute_goal_future.get();
                if (!recovery_compute_goal_handle) {
                    RCLCPP_WARN(get_logger(), "ComputePathThroughPoses goal rejected during recovery");
                    continue;
                }

                auto recovery_compute_result_future = compute_client_->async_get_result(recovery_compute_goal_handle);

                // wait for result with preempt/cancel checks
                continue_flag = false;
                {
                const auto start_t = now();
                while (rclcpp::ok()) {
                    if (checkAndWarnIfCancelled(action_server_) || checkAndWarnIfPreempted(action_server_)) {
                        compute_client_->async_cancel_goal(recovery_compute_goal_handle);
                        return false;
                    }
                    if (recovery_compute_result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
                        break;
                    }
                    if (compute_timeout_sec_ > 0.0 && (now() - start_t).seconds() > compute_timeout_sec_) {
                        compute_client_->async_cancel_goal(recovery_compute_goal_handle);
                        RCLCPP_WARN(get_logger(), "Timeout waiting ComputePathThroughPoses result during recovery");
                        continue_flag = true;
                        break;
                    }
                }
                }if (continue_flag) {continue;}

                auto recovery_compute_wrapped = recovery_compute_result_future.get();
                if (recovery_compute_wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
                    !recovery_compute_wrapped.result || recovery_compute_wrapped.result->path.poses.empty())
                {
                    RCLCPP_WARN(get_logger(), "ComputePathThroughPoses failed or returned empty path during recovery");
                    continue;
                }

                nav_msgs::msg::Path recovery_path = recovery_compute_wrapped.result->path;
                recovery_path.header.stamp = stamp_msg;
                recovery_path.header.frame_id = frame_id;

                ds_path = downsamplePath(recovery_path);
                ds_path.header.stamp = toTimeMsg(now());
                ds_path.header.frame_id = frame_id;

                if (debug_path_pub_ && debug_path_pub_->is_activated()) {
                    debug_path_pub_->publish(ds_path);
                }

                // ---- Covering ----
                feedback->status_code = Action::Feedback::COVERING;
                action_server_->publish_feedback(feedback);

                if (!follow_client_->wait_for_action_server(std::chrono::seconds(2))) {
                    RCLCPP_WARN(get_logger(), "FollowPath action server not available during recovery");
                    continue;
                }

                Follow::Goal recovery_follow_goal;
                recovery_follow_goal.path = ds_path;
                setControllerId(recovery_follow_goal, controller_id_, 0);
                setGoalCheckerId(recovery_follow_goal, goal_checker_id_, 0);

                auto recovery_follow_goal_future = follow_client_->async_send_goal(recovery_follow_goal);

                // wait goal-handle in small steps
                continue_flag = false;
                {
                    const auto start_t = now();
                    while (rclcpp::ok()) {
                        if (checkAndWarnIfCancelled(action_server_) || checkAndWarnIfPreempted(action_server_)) {
                            return false;
                        }
                        if (recovery_follow_goal_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                            break;
                        }
                        if (follow_timeout_sec_ > 0.0 && (now() - start_t).seconds() > follow_timeout_sec_) {
                            RCLCPP_WARN(get_logger(), "FollowPath action server not available during recovery");
                            continue_flag = true;
                            break;
                        }
                    }
                }if (continue_flag) {continue;}

                auto recovery_follow_goal_handle = recovery_follow_goal_future.get();
                if (!recovery_follow_goal_handle) {
                RCLCPP_WARN(get_logger(), "FollowPath goal rejected during recovery");
                continue;
                }

                auto recovery_follow_result_future = follow_client_->async_get_result(recovery_follow_goal_handle);

                // wait result with preempt/cancel checks
                continue_flag = false;
                {
                    const auto start_t = now();
                    while (rclcpp::ok()) {
                        if (checkAndWarnIfCancelled(action_server_) || checkAndWarnIfPreempted(action_server_)) {
                            follow_client_->async_cancel_goal(recovery_follow_goal_handle);
                            return false;
                        }
                        if (recovery_follow_result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
                            break;
                        }
                        if (follow_timeout_sec_ > 0.0 && (now() - start_t).seconds() > follow_timeout_sec_) {
                            follow_client_->async_cancel_goal(recovery_follow_goal_handle);
                            RCLCPP_WARN(get_logger(), "Timeout waiting FollowPath result during recovery");
                            continue_flag = true;
                            break;
                        }
                    }
                } if (continue_flag) {continue;}

                auto recovery_follow_wrapped = recovery_follow_result_future.get();
                if (recovery_follow_wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_WARN(get_logger(), "FollowPath failed during recovery");
                    continue;
                } 
                else {
                    success_recovered = true;
                    break;
                }
            }
        }

        if (!success_recovered) {
            throw nav2_coverage_core::FailedToFollowPlanCoverage("Failed to follow path and all recovery attempts exhausted.");
        }
    }

    // Success
    return true;
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_coverage::CoverageServer)

