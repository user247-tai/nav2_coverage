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

#include "nav2_coverage/plugins/fullmap_pose_creator.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>

namespace nav2_coverage
{

void FullmapPoseCreator::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name) 
{
    name_ = name;
    node_ = parent.lock();

    if (!node_) {
        throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".map_topic", rclcpp::ParameterValue("/map"));
    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".costmap_topic", rclcpp::ParameterValue("/global_costmap/costmap"));
    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".grid_step_x", rclcpp::ParameterValue(0.05));
    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".grid_step_y", rclcpp::ParameterValue(0.05));
    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".allow_unknown_map", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".allow_unknown_costmap", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".skip_outside_costmap", rclcpp::ParameterValue(true));

    node_->get_parameter(name + ".map_topic", map_topic_);
    node_->get_parameter(name + ".costmap_topic", costmap_topic_);
    node_->get_parameter(name + ".grid_step_x", grid_step_x_);
    node_->get_parameter(name + ".grid_step_y", grid_step_y_);
    node_->get_parameter(name + ".allow_unknown_map", allow_unknown_map_);
    node_->get_parameter(name + ".allow_unknown_costmap", allow_unknown_costmap_);
    node_->get_parameter(name + ".skip_outside_costmap", skip_outside_costmap_);

    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_;

    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&FullmapPoseCreator::onMapCallback, this, std::placeholders::_1));

    costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&FullmapPoseCreator::onCostmapCallback, this, std::placeholders::_1));
}

void FullmapPoseCreator::onMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    map_ = msg;
}

void FullmapPoseCreator::onCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    costmap_ = msg;
}

bool FullmapPoseCreator::isDataReady() {
    return map_ != nullptr && costmap_ != nullptr;
}

geometry_msgs::msg::PoseArray FullmapPoseCreator::createPoses(
    const std::string & order_mode,
    const bool enable_serpentine,
    const bool columns_left_to_right,
    const bool rows_bottom_to_top,
    const float downsample_step_x,
    const float downsample_step_y,
    const int map_occ_threshold,
    const int costmap_occ_threshold)
{
    const auto map_meta = metaFromGrid(*map_);
    const auto cost_meta = metaFromGrid(*costmap_);

    const auto & map_data = map_->data;
    const auto & cost_data = costmap_->data;

    const int step_x_cells = std::max(1, static_cast<int>(std::lround(grid_step_x_ / map_meta.resolution)));
    const int step_y_cells = std::max(1, static_cast<int>(std::lround(grid_step_y_ / map_meta.resolution)));

    const int dsx_cells = (downsample_step_x > 0.0) ?
        std::max(1, static_cast<int>(std::lround(downsample_step_x / map_meta.resolution))) : 1;
    const int dsy_cells = (downsample_step_y > 0.0) ?
        std::max(1, static_cast<int>(std::lround(downsample_step_y / map_meta.resolution))) : 1;

    const int half_dsx = dsx_cells / 2;
    const int half_dsy = dsy_cells / 2;

    // bins[(bx,by)] = (mx,my,pose,d2)
    struct BinVal { int mx; int my; geometry_msgs::msg::Pose pose; int d2; };
    std::unordered_map<std::uint64_t, BinVal> bins;
    bins.reserve(static_cast<size_t>(map_meta.width * map_meta.height / (dsx_cells * dsy_cells + 1)));

    auto pack_key = [](int bx, int by) -> std::uint64_t {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(bx)) << 32) |
            static_cast<std::uint32_t>(by);
    };

    for (int my = 0; my < map_meta.height; my += step_y_cells) {
        const int row_base = my * map_meta.width;
        for (int mx = 0; mx < map_meta.width; mx += step_x_cells) {
            const int idx = row_base + mx;
            const int8_t occ = map_data[static_cast<size_t>(idx)];

            // map filter
            if (occ < 0) {
                if (!allow_unknown_map_) continue;
            } else {
                if (occ >= map_occ_threshold) continue;
            }

            const double wx = map_meta.origin_x + (static_cast<double>(mx) + 0.5) * map_meta.resolution;
            const double wy = map_meta.origin_y + (static_cast<double>(my) + 0.5) * map_meta.resolution;

            // costmap filter
            int cx = 0, cy = 0;
            if (!worldToGrid(wx, wy, cost_meta, cx, cy)) {
                if (skip_outside_costmap_) continue;
            } else {
                const int cidx = cy * cost_meta.width + cx;
                const int8_t c = cost_data[static_cast<size_t>(cidx)];
                if (c < 0) {
                    if (!allow_unknown_costmap_) continue;
                } else {
                    if (c >= costmap_occ_threshold) continue;
                }
            }

            const int bx = mx / dsx_cells;
            const int by = my / dsy_cells;

            const int center_mx = bx * dsx_cells + half_dsx;
            const int center_my = by * dsy_cells + half_dsy;
            const int dx = mx - center_mx;
            const int dy = my - center_my;
            const int d2 = dx * dx + dy * dy;

            geometry_msgs::msg::Pose p;
            p.position.x = wx;
            p.position.y = wy;
            p.position.z = 0.0;
            p.orientation.w = 1.0;

            const std::uint64_t key = pack_key(bx, by);
            auto it = bins.find(key);
            if (it == bins.end() || d2 < it->second.d2) {
                bins[key] = BinVal{mx, my, p, d2};
            }
        }
    }

    std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>> nodes;
    nodes.reserve(bins.size());
    for (const auto & kv : bins) {
        const auto & v = kv.second;
        nodes.emplace_back(v.mx, v.my, v.pose);
    }

    geometry_msgs::msg::PoseArray out;
    out.header.frame_id = map_meta.frame_id;

    if (!enable_serpentine) {
        std::sort(nodes.begin(), nodes.end(), [](const auto & a, const auto & b) {
            const int ax = std::get<0>(a), ay = std::get<1>(a);
            const int bx = std::get<0>(b), by = std::get<1>(b);
            if (ax != bx) return ax < bx;
            return ay < by;
        });
        out.poses.reserve(nodes.size());
        for (const auto & t : nodes) {
            out.poses.push_back(std::get<2>(t));
        }
        return out;
    }

    out.poses = orderSerpentine(nodes, dsx_cells, dsy_cells, order_mode,
                                columns_left_to_right, rows_bottom_to_top);
    return out;
}

std::vector<geometry_msgs::msg::Pose> FullmapPoseCreator::orderSerpentine(
    const std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>> & nodes,
    int dsx_cells,
    int dsy_cells,
    const std::string & order_mode,
    bool columns_left_to_right,
    bool rows_bottom_to_top)
{
    // nodes tuples are (mx,my,pose)
    std::vector<geometry_msgs::msg::Pose> ordered;

    auto mode = order_mode;
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
    if (mode != "columns") {
        mode = "rows";
    }

    if (mode == "columns") {
        // group by bx
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
        std::sort(keys.begin(), keys.end(), [&](int a, int b) {
            return columns_left_to_right ? (a < b) : (a > b);
        });

        ordered.reserve(nodes.size());
        for (size_t i = 0; i < keys.size(); ++i) {
            auto & col = cols[keys[i]];
            std::sort(col.begin(), col.end(), [&](const auto & a, const auto & b) {
                const int my_a = std::get<1>(a);
                const int my_b = std::get<1>(b);
                const int by_a = my_a / std::max(1, dsy_cells);
                const int by_b = my_b / std::max(1, dsy_cells);
                if (by_a != by_b) return by_a < by_b;
                if (my_a != my_b) return my_a < my_b;
                return std::get<0>(a) < std::get<0>(b);
            });

            // even columns reversed (1-based)
            if (((i + 1) % 2) == 0) {
                std::reverse(col.begin(), col.end());
            }

            for (const auto & t : col) {
                ordered.push_back(std::get<2>(t));
            }
        }

        return ordered;
    }

    // mode == "rows": group by by
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
    std::sort(keys.begin(), keys.end(), [&](int a, int b) {
        return rows_bottom_to_top ? (a < b) : (a > b);
    });

    ordered.reserve(nodes.size());
    for (size_t i = 0; i < keys.size(); ++i) {
        auto & row = rows[keys[i]];
        std::sort(row.begin(), row.end(), [&](const auto & a, const auto & b) {
            const int mx_a = std::get<0>(a);
            const int mx_b = std::get<0>(b);
            const int bx_a = mx_a / std::max(1, dsx_cells);
            const int bx_b = mx_b / std::max(1, dsx_cells);
            if (bx_a != bx_b) return bx_a < bx_b;
            if (mx_a != mx_b) return mx_a < mx_b;
            return std::get<1>(a) < std::get<1>(b);
        });

        // even rows reversed (1-based)
        if (((i + 1) % 2) == 0) {
            std::reverse(row.begin(), row.end());
        }

        for (const auto & t : row) {
            ordered.push_back(std::get<2>(t));
        }
    }

    return ordered;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_coverage::FullmapPoseCreator, nav2_coverage_core::PosesCreator)
