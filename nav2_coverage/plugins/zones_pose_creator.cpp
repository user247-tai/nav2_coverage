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

#include "nav2_coverage/plugins/zones_pose_creator.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>

namespace nav2_coverage
{

void ZonesPoseCreator::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name)
{
  name_ = name;
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".zones_map_topic", rclcpp::ParameterValue("/zones_map"));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".costmap_topic", rclcpp::ParameterValue("/global_costmap/costmap"));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".map_topic", rclcpp::ParameterValue("/map"));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_map_topic", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".grid_step_x", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".grid_step_y", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".zone_map_encoding", rclcpp::ParameterValue("zone_id"));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".zone_grouping_mode", rclcpp::ParameterValue("by_id"));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".zone_order_mode", rclcpp::ParameterValue("ascending"));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".custom_zone_order", rclcpp::ParameterValue(std::vector<int64_t>{}));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".min_zone_cells", rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".allow_unknown_map", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".allow_unknown_costmap", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".skip_outside_costmap", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".robot_base_frame", rclcpp::ParameterValue("base_link"));

  node_->get_parameter(name + ".zones_map_topic", zones_map_topic_);
  node_->get_parameter(name + ".costmap_topic", costmap_topic_);
  node_->get_parameter(name + ".map_topic", map_topic_);
  node_->get_parameter(name + ".use_map_topic", use_map_topic_);
  node_->get_parameter(name + ".grid_step_x", grid_step_x_);
  node_->get_parameter(name + ".grid_step_y", grid_step_y_);
  node_->get_parameter(name + ".zone_map_encoding", zone_map_encoding_);
  node_->get_parameter(name + ".zone_grouping_mode", zone_grouping_mode_);
  node_->get_parameter(name + ".zone_order_mode", zone_order_mode_);
  node_->get_parameter(name + ".custom_zone_order", custom_zone_order_);
  node_->get_parameter(name + ".min_zone_cells", min_zone_cells_);
  node_->get_parameter(name + ".allow_unknown_map", allow_unknown_map_);
  node_->get_parameter(name + ".allow_unknown_costmap", allow_unknown_costmap_);
  node_->get_parameter(name + ".skip_outside_costmap", skip_outside_costmap_);
  node_->get_parameter(name + ".robot_base_frame", robot_base_frame_);

  cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  zones_map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    zones_map_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&ZonesPoseCreator::onZonesMapCallback, this, std::placeholders::_1),
    sub_opts);

  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&ZonesPoseCreator::onCostmapCallback, this, std::placeholders::_1),
    sub_opts);

  if (use_map_topic_) {
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ZonesPoseCreator::onMapCallback, this, std::placeholders::_1),
      sub_opts);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ZonesPoseCreator::cleanup()
{
  zones_map_sub_.reset();
  costmap_sub_.reset();
  map_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  node_.reset();
}

void ZonesPoseCreator::activate()
{
}

void ZonesPoseCreator::deactivate()
{
}

void ZonesPoseCreator::onZonesMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  zones_map_ = msg;
}

void ZonesPoseCreator::onCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  costmap_ = msg;
}

void ZonesPoseCreator::onMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  map_ = msg;
}

bool ZonesPoseCreator::isDataReady()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return static_cast<bool>(zones_map_) && static_cast<bool>(costmap_);
}

bool ZonesPoseCreator::getRobotPose(
  const std::string & frame_id,
  geometry_msgs::msg::PoseStamped & pose)
{
  if (!tf_buffer_) {
    return false;
  }
  try {
    auto transform = tf_buffer_->lookupTransform(
      frame_id, robot_base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Failed to get robot pose: %s", ex.what());
    return false;
  }
}

std::vector<ZoneRegion> ZonesPoseCreator::extractByZoneId(
  const nav_msgs::msg::OccupancyGrid & zones_map)
{
  const auto meta = metaFromGrid(zones_map);
  const int w = meta.width;
  const int h = meta.height;
  const bool binary_mode = (zone_map_encoding_ == "binary");

  std::unordered_map<int, ZoneRegion> zone_map;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      int idx = y * w + x;
      int8_t raw = zones_map.data[idx];

      bool is_zone = false;
      int zone_id = 0;
      if (binary_mode) {
        is_zone = (raw != 0);
        zone_id = 1;  // all non-zero cells share the same zone in by_id mode
      } else {
        zone_id = static_cast<int>(raw);
        is_zone = (zone_id > 0);
      }

      if (!is_zone) {
        continue;
      }

      if (zone_map.find(zone_id) == zone_map.end()) {
        ZoneRegion region;
        region.zone_id = zone_id;
        region.region_index = 0;
        zone_map[zone_id] = std::move(region);
      }

      double wx = meta.origin_x + (static_cast<double>(x) + 0.5) * meta.resolution;
      double wy = meta.origin_y + (static_cast<double>(y) + 0.5) * meta.resolution;

      geometry_msgs::msg::Pose p;
      p.position.x = wx;
      p.position.y = wy;
      p.position.z = 0.0;
      p.orientation.w = 1.0;

      zone_map[zone_id].nodes.emplace_back(x, y, p);
    }
  }

  std::vector<ZoneRegion> regions;
  for (auto & kv : zone_map) {
    auto & region = kv.second;
    long long sum_x = 0;
    long long sum_y = 0;
    for (const auto & n : region.nodes) {
      sum_x += std::get<0>(n);
      sum_y += std::get<1>(n);
    }
    if (!region.nodes.empty()) {
      region.centroid_x = meta.origin_x +
        (static_cast<double>(sum_x) / region.nodes.size() + 0.5) * meta.resolution;
      region.centroid_y = meta.origin_y +
        (static_cast<double>(sum_y) / region.nodes.size() + 0.5) * meta.resolution;
    }
    regions.push_back(std::move(region));
  }

  return regions;
}

std::vector<ZoneRegion> ZonesPoseCreator::runConnectedComponentLabeling(
  const nav_msgs::msg::OccupancyGrid & zones_map)
{
  const auto meta = metaFromGrid(zones_map);
  const int w = meta.width;
  const int h = meta.height;
  const bool binary_mode = (zone_map_encoding_ == "binary");

  std::vector<ZoneRegion> regions;
  std::vector<bool> visited(static_cast<size_t>(w) * h, false);

  const int dx[4] = {-1, 1, 0, 0};
  const int dy[4] = {0, 0, -1, 1};
  int next_binary_id = 1;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      int idx = y * w + x;
      if (visited[idx]) {
        continue;
      }

      int8_t raw = zones_map.data[idx];
      bool is_zone = false;
      int expected_value = 0;

      if (binary_mode) {
        is_zone = (raw != 0);
        expected_value = 0;  // dummy; in binary mode we only check != 0
      } else {
        expected_value = static_cast<int>(raw);
        is_zone = (expected_value > 0);
      }

      if (!is_zone) {
        visited[idx] = true;
        continue;
      }

      ZoneRegion region;
      if (binary_mode) {
        region.zone_id = next_binary_id++;  // auto-assign unique sequential ID
      } else {
        region.zone_id = expected_value;
      }
      region.region_index = 0;

      std::queue<std::pair<int, int>> q;
      q.push({x, y});
      visited[idx] = true;

      long long sum_x = 0;
      long long sum_y = 0;
      int count = 0;

      while (!q.empty()) {
        auto [cx, cy] = q.front();
        q.pop();

        double wx = meta.origin_x + (static_cast<double>(cx) + 0.5) * meta.resolution;
        double wy = meta.origin_y + (static_cast<double>(cy) + 0.5) * meta.resolution;

        geometry_msgs::msg::Pose p;
        p.position.x = wx;
        p.position.y = wy;
        p.position.z = 0.0;
        p.orientation.w = 1.0;

        region.nodes.emplace_back(cx, cy, p);
        sum_x += cx;
        sum_y += cy;
        ++count;

        for (int i = 0; i < 4; ++i) {
          int nx = cx + dx[i];
          int ny = cy + dy[i];
          if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
            continue;
          }
          int nidx = ny * w + nx;
          if (visited[nidx]) {
            continue;
          }

          int8_t nraw = zones_map.data[nidx];
          bool neighbor_is_zone = false;
          if (binary_mode) {
            neighbor_is_zone = (nraw != 0);
          } else {
            neighbor_is_zone = (nraw == expected_value);
          }

          if (!neighbor_is_zone) {
            continue;
          }
          visited[nidx] = true;
          q.push({nx, ny});
        }
      }

      if (count > 0) {
        region.centroid_x = meta.origin_x +
          (static_cast<double>(sum_x) / count + 0.5) * meta.resolution;
        region.centroid_y = meta.origin_y +
          (static_cast<double>(sum_y) / count + 0.5) * meta.resolution;
      }

      regions.push_back(std::move(region));
    }
  }

  return regions;
}

std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>>
ZonesPoseCreator::processRegion(
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
  int costmap_occ_threshold)
{
  // Build cell set and bounding box
  std::unordered_set<uint64_t> cell_set;
  int min_mx = INT_MAX;
  int max_mx = INT_MIN;
  int min_my = INT_MAX;
  int max_my = INT_MIN;

  for (const auto & n : region.nodes) {
    int mx = std::get<0>(n);
    int my = std::get<1>(n);
    uint64_t key = (static_cast<uint64_t>(mx) << 32) | static_cast<uint32_t>(my);
    cell_set.insert(key);
    min_mx = std::min(min_mx, mx);
    max_mx = std::max(max_mx, mx);
    min_my = std::min(min_my, my);
    max_my = std::max(max_my, my);
  }

  // Grid sampling + filtering
  std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>> filtered;

  for (int my = min_my; my <= max_my; my += step_y_cells) {
    for (int mx = min_mx; mx <= max_mx; mx += step_x_cells) {
      uint64_t key = (static_cast<uint64_t>(mx) << 32) | static_cast<uint32_t>(my);
      if (cell_set.find(key) == cell_set.end()) {
        continue;
      }

      double wx = zones_meta.origin_x + (static_cast<double>(mx) + 0.5) * zones_meta.resolution;
      double wy = zones_meta.origin_y + (static_cast<double>(my) + 0.5) * zones_meta.resolution;

      // Costmap filter
      int cx = 0, cy = 0;
      if (!worldToGrid(wx, wy, cost_meta, cx, cy)) {
        if (skip_outside_costmap_) {
          continue;
        }
      } else {
        int cidx = cy * cost_meta.width + cx;
        int8_t c = costmap.data[static_cast<size_t>(cidx)];
        if (c < 0) {
          if (!allow_unknown_costmap_) {
            continue;
          }
        } else {
          if (c >= costmap_occ_threshold) {
            continue;
          }
        }
      }

      // Optional map filter
      if (use_map_topic_ && map) {
        auto map_meta = metaFromGrid(*map);
        int mx_map = 0, my_map = 0;
        if (worldToGrid(wx, wy, map_meta, mx_map, my_map)) {
          int midx = my_map * map_meta.width + mx_map;
          int8_t mval = map->data[static_cast<size_t>(midx)];
          if (mval < 0) {
            if (!allow_unknown_map_) {
              continue;
            }
          } else {
            if (mval >= map_occ_threshold) {
              continue;
            }
          }
        } else if (skip_outside_costmap_) {
          continue;
        }
      }

      geometry_msgs::msg::Pose p;
      p.position.x = wx;
      p.position.y = wy;
      p.position.z = 0.0;
      p.orientation.w = 1.0;
      filtered.emplace_back(mx, my, p);
    }
  }

  // Fallback for small regions that missed the global grid
  if (filtered.empty() && !region.nodes.empty()) {
    long long sum_x = 0;
    long long sum_y = 0;
    for (const auto & n : region.nodes) {
      sum_x += std::get<0>(n);
      sum_y += std::get<1>(n);
    }
    int cx = static_cast<int>(sum_x / static_cast<long long>(region.nodes.size()));
    int cy = static_cast<int>(sum_y / static_cast<long long>(region.nodes.size()));

    size_t best_i = 0;
    int best_d2 = INT_MAX;
    for (size_t i = 0; i < region.nodes.size(); ++i) {
      int dx = std::get<0>(region.nodes[i]) - cx;
      int dy = std::get<1>(region.nodes[i]) - cy;
      int d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_i = i;
      }
    }

    const auto & best = region.nodes[best_i];
    int mx = std::get<0>(best);
    int my = std::get<1>(best);
    double wx = zones_meta.origin_x + (static_cast<double>(mx) + 0.5) * zones_meta.resolution;
    double wy = zones_meta.origin_y + (static_cast<double>(my) + 0.5) * zones_meta.resolution;

    bool valid = true;
    int cxx = 0, cyy = 0;
    if (!worldToGrid(wx, wy, cost_meta, cxx, cyy)) {
      if (skip_outside_costmap_) {
        valid = false;
      }
    } else {
      int cidx = cyy * cost_meta.width + cxx;
      int8_t c = costmap.data[static_cast<size_t>(cidx)];
      if (c < 0) {
        if (!allow_unknown_costmap_) {
          valid = false;
        }
      } else if (c >= costmap_occ_threshold) {
        valid = false;
      }
    }

    if (valid && use_map_topic_ && map) {
      auto map_meta = metaFromGrid(*map);
      int mx_map = 0, my_map = 0;
      if (worldToGrid(wx, wy, map_meta, mx_map, my_map)) {
        int midx = my_map * map_meta.width + mx_map;
        int8_t mval = map->data[static_cast<size_t>(midx)];
        if (mval < 0) {
          if (!allow_unknown_map_) {
            valid = false;
          }
        } else if (mval >= map_occ_threshold) {
          valid = false;
        }
      } else if (skip_outside_costmap_) {
        valid = false;
      }
    }

    if (valid) {
      geometry_msgs::msg::Pose p;
      p.position.x = wx;
      p.position.y = wy;
      p.position.z = 0.0;
      p.orientation.w = 1.0;
      filtered.emplace_back(mx, my, p);
    }
  }

  // Binning (downsampling) using downsample_step
  if (dsx_cells > 1 || dsy_cells > 1) {
    struct BinVal {
      int mx;
      int my;
      geometry_msgs::msg::Pose pose;
      int d2;
    };
    std::unordered_map<uint64_t, BinVal> bins;
    auto pack_key = [](int bx, int by) -> uint64_t {
      return (static_cast<uint64_t>(static_cast<uint32_t>(bx)) << 32) |
        static_cast<uint32_t>(by);
    };

    int half_dsx = dsx_cells / 2;
    int half_dsy = dsy_cells / 2;

    for (const auto & n : filtered) {
      int mx = std::get<0>(n);
      int my = std::get<1>(n);
      const auto & pose = std::get<2>(n);
      int bx = mx / dsx_cells;
      int by = my / dsy_cells;
      int center_mx = bx * dsx_cells + half_dsx;
      int center_my = by * dsy_cells + half_dsy;
      int dx = mx - center_mx;
      int dy = my - center_my;
      int d2 = dx * dx + dy * dy;

      uint64_t key = pack_key(bx, by);
      auto it = bins.find(key);
      if (it == bins.end() || d2 < it->second.d2) {
        bins[key] = BinVal{mx, my, pose, d2};
      }
    }

    filtered.clear();
    for (const auto & kv : bins) {
      const auto & v = kv.second;
      filtered.emplace_back(v.mx, v.my, v.pose);
    }
  }

  return filtered;
}

std::vector<int> ZonesPoseCreator::determineZoneOrder(
  const std::vector<ZoneRegion> & regions,
  const std::string & frame_id)
{
  std::vector<int> indices(regions.size());
  std::iota(indices.begin(), indices.end(), 0);

  if (zone_order_mode_ == "nearest") {
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(frame_id, robot_pose)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Failed to get robot pose for nearest zone ordering. Falling back to ascending.");
      // Fall back to ascending for this call only, without mutating the parameter
      std::sort(
        indices.begin(), indices.end(),
        [&](int a, int b) {
          if (regions[a].zone_id != regions[b].zone_id) {
            return regions[a].zone_id < regions[b].zone_id;
          }
          return regions[a].region_index < regions[b].region_index;
        });
      return indices;
    } else {
      std::sort(
        indices.begin(), indices.end(),
        [&](int a, int b) {
          double da = std::hypot(
            regions[a].centroid_x - robot_pose.pose.position.x,
            regions[a].centroid_y - robot_pose.pose.position.y);
          double db = std::hypot(
            regions[b].centroid_x - robot_pose.pose.position.x,
            regions[b].centroid_y - robot_pose.pose.position.y);
          return da < db;
        });
      return indices;
    }
  }

  if (zone_order_mode_ == "descending") {
    std::sort(
      indices.begin(), indices.end(),
      [&](int a, int b) {
        if (regions[a].zone_id != regions[b].zone_id) {
          return regions[a].zone_id > regions[b].zone_id;
        }
        return regions[a].region_index > regions[b].region_index;
      });
  } else if (zone_order_mode_ == "custom") {
    std::unordered_map<int, int> priority;
    for (size_t i = 0; i < custom_zone_order_.size(); ++i) {
      priority[static_cast<int>(custom_zone_order_[i])] = static_cast<int>(i);
    }
    std::sort(
      indices.begin(), indices.end(),
      [&](int a, int b) {
        int pa = priority.count(regions[a].zone_id) ? priority[regions[a].zone_id] : INT_MAX;
        int pb = priority.count(regions[b].zone_id) ? priority[regions[b].zone_id] : INT_MAX;
        if (pa != pb) {
          return pa < pb;
        }
        return regions[a].region_index < regions[b].region_index;
      });
  } else {
    // ascending (default)
    std::sort(
      indices.begin(), indices.end(),
      [&](int a, int b) {
        if (regions[a].zone_id != regions[b].zone_id) {
          return regions[a].zone_id < regions[b].zone_id;
        }
        return regions[a].region_index < regions[b].region_index;
      });
  }

  return indices;
}

geometry_msgs::msg::PoseArray ZonesPoseCreator::createPoses(
  const std::string & order_mode,
  const bool enable_serpentine,
  const bool columns_left_to_right,
  const bool rows_bottom_to_top,
  const float downsample_step_x,
  const float downsample_step_y,
  const int map_occ_threshold,
  const int costmap_occ_threshold)
{
  nav_msgs::msg::OccupancyGrid::SharedPtr zones_map;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    zones_map = zones_map_;
    costmap = costmap_;
    map = map_;
  }

  if (!zones_map || !costmap) {
    throw std::runtime_error("Zones map or costmap data not ready");
  }

  const auto zones_meta = metaFromGrid(*zones_map);
  const auto cost_meta = metaFromGrid(*costmap);

  if (zones_meta.resolution <= 0.0 || cost_meta.resolution <= 0.0) {
    throw std::runtime_error("Invalid occupancy grid: non-positive resolution");
  }

  // Extract raw zone regions
  std::vector<ZoneRegion> regions;
  if (zone_grouping_mode_ == "by_connected_component") {
    regions = runConnectedComponentLabeling(*zones_map);
  } else {
    regions = extractByZoneId(*zones_map);
  }

  if (regions.empty()) {
    throw std::runtime_error("No valid zones found in zones map");
  }

  // Grid and bin parameters
  const int step_x_cells = std::max(
    1, static_cast<int>(std::lround(grid_step_x_ / zones_meta.resolution)));
  const int step_y_cells = std::max(
    1, static_cast<int>(std::lround(grid_step_y_ / zones_meta.resolution)));

  const int dsx_cells = (downsample_step_x > 0.0f) ?
    std::max(1, static_cast<int>(std::lround(downsample_step_x / zones_meta.resolution))) :
    1;
  const int dsy_cells = (downsample_step_y > 0.0f) ?
    std::max(1, static_cast<int>(std::lround(downsample_step_y / zones_meta.resolution))) :
    1;

  // Process each region: sample, filter, bin
  for (auto & region : regions) {
    region.nodes = processRegion(
      region, zones_meta, *costmap, cost_meta, map,
      step_x_cells, step_y_cells, dsx_cells, dsy_cells,
      map_occ_threshold, costmap_occ_threshold);
  }

  // Determine zone order
  std::vector<int> order_indices = determineZoneOrder(regions, zones_meta.frame_id);

  geometry_msgs::msg::PoseArray out;
  out.header.frame_id = zones_meta.frame_id;

  // Order poses within each zone and concatenate
  for (int region_idx : order_indices) {
    auto & region = regions[region_idx];

    if (static_cast<int>(region.nodes.size()) < min_zone_cells_) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "Skipping zone %d region %d with only %zu valid cells (min=%d)",
        region.zone_id, region.region_index, region.nodes.size(), min_zone_cells_);
      continue;
    }

    std::vector<geometry_msgs::msg::Pose> ordered;
    auto mode = order_mode;
    std::transform(
      mode.begin(), mode.end(), mode.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mode != "columns") {
      mode = "rows";
    }

    if (!enable_serpentine) {
      std::sort(
        region.nodes.begin(), region.nodes.end(),
        [&](const auto & a, const auto & b) {
          int ax = std::get<0>(a), ay = std::get<1>(a);
          int bx = std::get<0>(b), by = std::get<1>(b);
          if (mode == "rows") {
            int primary_a = rows_bottom_to_top ? -ay : ay;
            int primary_b = rows_bottom_to_top ? -by : by;
            if (primary_a != primary_b) {
              return primary_a < primary_b;
            }
            return columns_left_to_right ? (ax < bx) : (ax > bx);
          } else {
            int primary_a = columns_left_to_right ? ax : -ax;
            int primary_b = columns_left_to_right ? bx : -bx;
            if (primary_a != primary_b) {
              return primary_a < primary_b;
            }
            return rows_bottom_to_top ? (ay < by) : (ay > by);
          }
        });
      ordered.reserve(region.nodes.size());
      for (const auto & n : region.nodes) {
        ordered.push_back(std::get<2>(n));
      }
    } else {
      ordered = orderSerpentine(
        region.nodes, dsx_cells, dsy_cells, order_mode,
        columns_left_to_right, rows_bottom_to_top);
    }

    out.poses.insert(out.poses.end(), ordered.begin(), ordered.end());
  }

  return out;
}

std::vector<geometry_msgs::msg::Pose> ZonesPoseCreator::orderSerpentine(
  const std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>> & nodes,
  int dsx_cells,
  int dsy_cells,
  const std::string & order_mode,
  bool columns_left_to_right,
  bool rows_bottom_to_top)
{
  std::vector<geometry_msgs::msg::Pose> ordered;

  auto mode = order_mode;
  std::transform(
    mode.begin(), mode.end(), mode.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (mode != "columns") {
    mode = "rows";
  }

  if (mode == "columns") {
    std::unordered_map<int, std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>>> cols;
    cols.reserve(nodes.size());
    for (const auto & t : nodes) {
      const int mx = std::get<0>(t);
      const int bx = mx / std::max(1, dsx_cells);
      cols[bx].push_back(t);
    }

    std::vector<int> keys;
    keys.reserve(cols.size());
    for (const auto & kv : cols) {
      keys.push_back(kv.first);
    }
    std::sort(
      keys.begin(), keys.end(),
      [&](int a, int b) {
        return columns_left_to_right ? (a < b) : (a > b);
      });

    ordered.reserve(nodes.size());
    for (size_t i = 0; i < keys.size(); ++i) {
      auto & col = cols[keys[i]];
      std::sort(
        col.begin(), col.end(),
        [&](const auto & a, const auto & b) {
          const int my_a = std::get<1>(a);
          const int my_b = std::get<1>(b);
          const int by_a = my_a / std::max(1, dsy_cells);
          const int by_b = my_b / std::max(1, dsy_cells);
          if (by_a != by_b) {
            return rows_bottom_to_top ? (by_a < by_b) : (by_a > by_b);
          }
          if (my_a != my_b) {
            return rows_bottom_to_top ? (my_a < my_b) : (my_a > my_b);
          }
          return std::get<0>(a) < std::get<0>(b);
        });

      if (((i + 1) % 2) == 0) {
        std::reverse(col.begin(), col.end());
      }

      for (const auto & t : col) {
        ordered.push_back(std::get<2>(t));
      }
    }

    return ordered;
  }

  // mode == "rows"
  std::unordered_map<int, std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>>> rows;
  rows.reserve(nodes.size());
  for (const auto & t : nodes) {
    const int my = std::get<1>(t);
    const int by = my / std::max(1, dsy_cells);
    rows[by].push_back(t);
  }

  std::vector<int> keys;
  keys.reserve(rows.size());
  for (const auto & kv : rows) {
    keys.push_back(kv.first);
  }
  std::sort(
    keys.begin(), keys.end(),
    [&](int a, int b) {
      return rows_bottom_to_top ? (a < b) : (a > b);
    });

  ordered.reserve(nodes.size());
  for (size_t i = 0; i < keys.size(); ++i) {
    auto & row = rows[keys[i]];
    std::sort(
      row.begin(), row.end(),
      [&](const auto & a, const auto & b) {
        const int mx_a = std::get<0>(a);
        const int mx_b = std::get<0>(b);
        const int bx_a = mx_a / std::max(1, dsx_cells);
        const int bx_b = mx_b / std::max(1, dsx_cells);
        if (bx_a != bx_b) {
          return columns_left_to_right ? (bx_a < bx_b) : (bx_a > bx_b);
        }
        if (mx_a != mx_b) {
          return columns_left_to_right ? (mx_a < mx_b) : (mx_a > mx_b);
        }
        return std::get<1>(a) < std::get<1>(b);
      });

    if (((i + 1) % 2) == 0) {
      std::reverse(row.begin(), row.end());
    }

    for (const auto & t : row) {
      ordered.push_back(std::get<2>(t));
    }
  }

  return ordered;
}

}  // namespace nav2_coverage

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_coverage::ZonesPoseCreator, nav2_coverage_core::PosesCreator)
