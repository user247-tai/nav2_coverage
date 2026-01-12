# nav2_coverage

![coverage_full](https://github.com/user-attachments/assets/bb9305a6-9ab7-40eb-8daa-4fc9d34e8b61)


## Overview
`nav2_coverage` (Nav2 Coverage Server) is a Nav2 server that aims to replace
[opennav_coverage](https://github.com/open-navigation/opennav_coverage) by providing a
simple **“cover the whole map”** workflow using existing Nav2 servers:

- **Planner Server**: plan using `nav2_msgs/action/ComputePathThroughPoses`
- **Controller Server**: execute using `nav2_msgs/action/FollowPath`

`nav2_coverage` exposes a single action server:
- `nav2_msgs/action/CoverAllMap`

When you send a `CoverAllMap` goal, the server will:
1) Create coverage poses from `/map` + `/global_costmap/costmap`  
2) Call **ComputePathThroughPoses** to generate a coverage path  
3) Call **FollowPath** to execute the computed path  
4) Return success when `FollowPath` succeeds, otherwise return the corresponding error

Action definition reference:
- [CoverAllMap.action](https://github.com/user247-tai/navigation2/blob/tai/kilted/nav2_msgs/action/CoverAllMap.action)

---

## How it works

### 1) Coverage poses creation
Coverage poses are generated from `OccupancyGrid` sources:
- `map_topic` (usually `/map`)
- `costmap_topic` (usually `/global_costmap/costmap`)

Cells are filtered using occupancy thresholds and unknown-space rules, then downsampled
by binning to produce a dense but manageable set of candidate poses.

### 2) Ordering (rows / columns)
When `enable_serpentine=true`, poses are ordered in a “lawnmower / boustrophedon” style:
- Odd group keeps direction
- Even group reverses direction

This reduces unnecessary turning and deadhead travel.

### 3) Planning and execution
- `ComputePathThroughPoses` produces a global path visiting the coverage poses
- The path can be downsampled (`downsample_keep_every_n` or `downsample_min_dist`)
- `FollowPath` executes the final path

---

## Debug topics
`nav2_coverage` publishes debug outputs (latched) for visualization:

- `graph_nodes_topic` (`geometry_msgs/PoseArray`): generated coverage poses
- `debug_path_topic` (`nav_msgs/Path`): computed (optionally downsampled) path

---

## Configuration

### Coverage server parameters (example)
```yaml
coverage_server:
  ros__parameters:
    # ---- topics ----
    map_topic: "/map"
    costmap_topic: "/global_costmap/costmap"

    graph_nodes_topic: "/graph_nodes"
    debug_path_topic: "/debug/computed_path"

    # ---- actions ----
    compute_action_name: "/compute_path_through_poses"
    follow_action_name: "/follow_path"

    # ---- behavior ----
    set_start_from_first_pose: true

    # ---- timeouts ----
    wait_grid_timeout_sec: 5.0
    compute_timeout_sec: 30.0
    follow_timeout_sec: 0.0   # 0 => no timeout

    # ---- downsample computed path ----
    downsample_keep_every_n: 0
    downsample_min_dist: 0.0

    # ---- pose generation ----
    grid_step_x: 0.05
    grid_step_y: 0.05
    allow_unknown_map: false
    allow_unknown_costmap: false
    skip_outside_costmap: true

    # ---- recovery ----
    retries_on_failure: -1   # -1 => infinite retries
```

---

## Recommended Nav2 configuration
This server works best with:
- **Planner** that produces clean pose-to-pose paths (NavFn, Smac 2D)
- **Controller** that tracks paths strictly (Regulated Pure Pursuit, Graceful Controller)

Example:
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"
      tolerance: 0.125
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 2.0
      cost_travel_multiplier: 2.0
      use_final_approach_orientation: false
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001

    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 60.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true

    FollowPath:
      plugin: "nav2_graceful_controller::GracefulController"
      transform_tolerance: 0.1
      min_lookahead: 0.15
      max_lookahead: 0.65
      max_robot_pose_search_dist: 0.65
      initial_rotation: true
      initial_rotation_threshold: 0.75
      prefer_final_rotation: true
      allow_backward: false
      k_phi: 2.0
      k_delta: 1.0
      beta: 0.4
      lambda: 2.0
      v_linear_min: 0.1
      v_linear_max: 0.75
      v_angular_max: 5.0
      v_angular_min_in_place: 0.25
      slowdown_radius: 0.25
```
---

## Compatibility
This package has been built and tested with:
- [Nav2 Kilted branch](https://github.com/ros-navigation/navigation2/tree/kilted) (main branch)

---

## License
- **Original Author**: user247-tai
- License: MIT License
