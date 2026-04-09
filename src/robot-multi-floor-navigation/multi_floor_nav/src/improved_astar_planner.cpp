#include <multi_floor_nav/improved_astar_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <tf/tf.h>
#include <algorithm>
#include <limits>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(improved_astar::ImprovedAStarPlanner, nav_core::BaseGlobalPlanner)

namespace improved_astar {

namespace {
constexpr double kSqrt2 = 1.41421356;
constexpr double kOctileDiag = 0.41421356;  // sqrt(2) - 1
constexpr double kEps = 1e-9;
constexpr double kBetaGoal = 0.45;
constexpr double kBetaTurn = 0.95;
constexpr double kPi = 3.14159265358979323846;
}  // namespace

ImprovedAStarPlanner::ImprovedAStarPlanner() {}

ImprovedAStarPlanner::ImprovedAStarPlanner(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros);
}

void ImprovedAStarPlanner::initialize(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (initialized_) {
        ROS_WARN("[ImprovedAStar] Already initialized, skipping.");
        return;
    }

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("density_radius", density_radius_, 2);
    private_nh.param("inflation_lambda", inflation_lambda_, 0.5);
    private_nh.param("threshold_open", threshold_open_, 0.20);
    private_nh.param("threshold_narrow", threshold_narrow_, 0.45);
    private_nh.param("w_obs_open", w_obs_open_, 0.15);
    private_nh.param("w_obs_normal", w_obs_normal_, 0.45);
    private_nh.param("w_obs_narrow", w_obs_narrow_, 0.90);
    private_nh.param("w_dir_open", w_dir_open_, 0.35);
    private_nh.param("w_dir_normal", w_dir_normal_, 0.25);
    private_nh.param("w_dir_narrow", w_dir_narrow_, 0.15);
    private_nh.param("alpha_open", alpha_open_, 1.35);
    private_nh.param("alpha_normal", alpha_normal_, 1.15);
    private_nh.param("alpha_narrow", alpha_narrow_, 1.00);
    private_nh.param("neutral_cost", neutral_cost_, 50.0);
    private_nh.param("lethal_cost_threshold", lethal_cost_threshold_, 253);
    private_nh.param("smoothing_enabled", smoothing_enabled_, true);
    private_nh.param("smoothing_max_jump_m", smoothing_max_jump_m_, 2.0);
    private_nh.param("smoothing_bspline_upsample", smoothing_bspline_upsample_, 3);
    private_nh.param("smoothing_min_point_spacing_m", smoothing_min_point_spacing_m_, 0.03);
    private_nh.param("smoothing_safety_clearance_m", smoothing_safety_clearance_m_, 0.12);
    private_nh.param("smoothing_danger_clearance_m", smoothing_danger_clearance_m_, 0.20);
    private_nh.param("smoothing_turn_angle_threshold_rad", smoothing_turn_angle_threshold_rad_, 0.35);

    density_radius_ = std::max(1, density_radius_);
    neutral_cost_ = std::max(1e-6, neutral_cost_);
    smoothing_bspline_upsample_ = std::max(1, smoothing_bspline_upsample_);
    smoothing_max_jump_m_ = std::max(0.0, smoothing_max_jump_m_);
    smoothing_min_point_spacing_m_ = std::max(0.0, smoothing_min_point_spacing_m_);
    smoothing_safety_clearance_m_ = std::max(0.0, smoothing_safety_clearance_m_);
    smoothing_danger_clearance_m_ = std::max(smoothing_safety_clearance_m_, smoothing_danger_clearance_m_);
    smoothing_turn_angle_threshold_rad_ = std::max(0.0, std::min(kPi, smoothing_turn_angle_threshold_rad_));
    threshold_open_ = std::max(0.0, std::min(1.0, threshold_open_));
    threshold_narrow_ = std::max(threshold_open_, std::min(1.0, threshold_narrow_));

    ROS_INFO("[ImprovedAStar] Initialized: density_radius=%d, neutral_cost=%.1f, "
             "threshold_open=%.2f, threshold_narrow=%.2f",
             density_radius_, neutral_cost_, threshold_open_, threshold_narrow_);
    ROS_INFO("[ImprovedAStar] w_obs=(%.2f,%.2f,%.2f), w_dir=(%.2f,%.2f,%.2f), "
             "alpha=(%.2f,%.2f,%.2f)",
             w_obs_open_, w_obs_normal_, w_obs_narrow_,
             w_dir_open_, w_dir_normal_, w_dir_narrow_,
             alpha_open_, alpha_normal_, alpha_narrow_);
    ROS_INFO("[ImprovedAStar] smoothing enabled=%s, max_jump=%.2f m, upsample=%d, "
             "safe_clearance=%.2f m, danger_clearance=%.2f m",
             smoothing_enabled_ ? "true" : "false",
             smoothing_max_jump_m_, smoothing_bspline_upsample_,
             smoothing_safety_clearance_m_, smoothing_danger_clearance_m_);

    initialized_ = true;
}

bool ImprovedAStarPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {

    if (!initialized_) {
        ROS_ERROR("[ImprovedAStar] Not initialized.");
        return false;
    }

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();

    uint32_t mx_start = 0;
    uint32_t my_start = 0;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y,
                              mx_start, my_start)) {
        ROS_WARN("[ImprovedAStar] Start position (%.2f, %.2f) is outside the costmap.",
                 start.pose.position.x, start.pose.position.y);
        return false;
    }

    uint32_t mx_goal = 0;
    uint32_t my_goal = 0;
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y,
                              mx_goal, my_goal)) {
        ROS_WARN("[ImprovedAStar] Goal position (%.2f, %.2f) is outside the costmap.",
                 goal.pose.position.x, goal.pose.position.y);
        return false;
    }

    int32_t sx = static_cast<int32_t>(mx_start);
    int32_t sy = static_cast<int32_t>(my_start);
    int32_t gx = static_cast<int32_t>(mx_goal);
    int32_t gy = static_cast<int32_t>(my_goal);

    int32_t size_x = static_cast<int32_t>(costmap_->getSizeInCellsX());
    int32_t size_y = static_cast<int32_t>(costmap_->getSizeInCellsY());

    // Search radius for finding free cells near occupied start/goal points.
    // map resolution = 0.02m, inflation_radius = 0.3m = 15 cells.
    // Use 30 cells (0.6m) to comfortably exceed inflation zone.
    const int32_t kSearchRadius = 30;

    if (!isFree(gx, gy)) {
        ROS_WARN("[ImprovedAStar] Goal cell (%d, %d) cost=%d is occupied, "
                 "searching within %d cells.",
                 gx, gy,
                 static_cast<int32_t>(costmap_->getCost(
                     static_cast<uint32_t>(gx), static_cast<uint32_t>(gy))),
                 kSearchRadius);
        bool found = false;
        for (int32_t r = 1; r <= kSearchRadius && !found; ++r) {
            for (int32_t dx = -r; dx <= r && !found; ++dx) {
                for (int32_t dy = -r; dy <= r && !found; ++dy) {
                    if (std::abs(dx) != r && std::abs(dy) != r) {
                        continue;
                    }
                    int32_t cx = gx + dx;
                    int32_t cy = gy + dy;
                    if (cx < 0 || cx >= size_x || cy < 0 || cy >= size_y) {
                        continue;
                    }
                    if (isFree(cx, cy)) {
                        ROS_INFO("[ImprovedAStar] Adjusted goal from (%d,%d) to (%d,%d) (r=%d).",
                                 gx, gy, cx, cy, r);
                        gx = cx;
                        gy = cy;
                        found = true;
                    }
                }
            }
        }
        if (!found) {
            ROS_WARN("[ImprovedAStar] Cannot find free goal cell within %d cells.", kSearchRadius);
            return false;
        }
    }

    if (!isFree(sx, sy)) {
        ROS_WARN("[ImprovedAStar] Start cell (%d, %d) cost=%d is occupied, "
                 "searching within %d cells.",
                 sx, sy,
                 static_cast<int32_t>(costmap_->getCost(
                     static_cast<uint32_t>(sx), static_cast<uint32_t>(sy))),
                 kSearchRadius);
        bool found = false;
        for (int32_t r = 1; r <= kSearchRadius && !found; ++r) {
            for (int32_t dx = -r; dx <= r && !found; ++dx) {
                for (int32_t dy = -r; dy <= r && !found; ++dy) {
                    if (std::abs(dx) != r && std::abs(dy) != r) {
                        continue;
                    }
                    int32_t cx = sx + dx;
                    int32_t cy = sy + dy;
                    if (cx < 0 || cx >= size_x || cy < 0 || cy >= size_y) {
                        continue;
                    }
                    if (isFree(cx, cy)) {
                        ROS_INFO("[ImprovedAStar] Adjusted start from (%d,%d) to (%d,%d) (r=%d).",
                                 sx, sy, cx, cy, r);
                        sx = cx;
                        sy = cy;
                        found = true;
                    }
                }
            }
        }
        if (!found) {
            ROS_WARN("[ImprovedAStar] Cannot find free start cell within %d cells.", kSearchRadius);
            return false;
        }
    }

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::unordered_map<int64_t, AStarNode> closed;
    std::unordered_map<int64_t, double> best_g;

    AStarNode start_node;
    start_node.x = sx;
    start_node.y = sy;
    start_node.parent_x = -1;
    start_node.parent_y = -1;
    start_node.g = 0.0;
    start_node.f = weightedHeuristic(sx, sy, gx, gy, alpha_normal_);

    int64_t start_idx = coordToIndex(sx, sy);
    best_g[start_idx] = 0.0;
    open_list.push(start_node);

    int32_t expanded_count = 0;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        open_list.pop();

        int64_t cur_idx = coordToIndex(current.x, current.y);

        if (closed.count(cur_idx) > 0) {
            continue;
        }

        auto bg_it = best_g.find(cur_idx);
        if (bg_it != best_g.end() && current.g > bg_it->second + kEps) {
            continue;
        }

        if (current.x == gx && current.y == gy) {
            closed[cur_idx] = current;

            std::vector<std::pair<double, double>> waypoints;
            AStarNode trace = current;
            while (true) {
                double wx = 0.0;
                double wy = 0.0;
                costmap_->mapToWorld(static_cast<uint32_t>(trace.x),
                                    static_cast<uint32_t>(trace.y), wx, wy);
                waypoints.emplace_back(wx, wy);
                if (trace.parent_x < 0) {
                    break;
                }
                int64_t parent_idx = coordToIndex(trace.parent_x, trace.parent_y);
                auto parent_it = closed.find(parent_idx);
                if (parent_it == closed.end()) {
                    break;
                }
                trace = parent_it->second;
            }

            std::reverse(waypoints.begin(), waypoints.end());

            std::vector<std::pair<double, double>> raw_waypoints = deduplicatePath(waypoints);
            std::vector<std::pair<double, double>> final_waypoints = raw_waypoints;
            if (smoothing_enabled_) {
                final_waypoints = smoothPathWithFallback(raw_waypoints);
                if (final_waypoints.size() < 2) {
                    final_waypoints = raw_waypoints;
                }
            }

            PathMetrics raw_metrics = computePathMetrics(raw_waypoints);
            PathMetrics smooth_metrics = computePathMetrics(final_waypoints);

            plan.reserve(final_waypoints.size());
            for (const auto& wp : final_waypoints) {
                geometry_msgs::PoseStamped pose;
                pose.header = goal.header;
                pose.pose.position.x = wp.first;
                pose.pose.position.y = wp.second;
                pose.pose.position.z = 0.0;
                pose.pose.orientation = goal.pose.orientation;
                plan.emplace_back(pose);
            }

            logPathComparison(raw_metrics, smooth_metrics,
                              raw_waypoints.size(), final_waypoints.size(), expanded_count);
            ROS_INFO("[ImprovedAStar] Plan found: %zu waypoints, %d nodes expanded.",
                     plan.size(), expanded_count);
            return true;
        }

        auto neighbors = goalBiasedNeighbors(current.x, current.y, gx, gy);
        for (const auto& nb : neighbors) {
            int32_t nx = nb.first;
            int32_t ny = nb.second;
            if (nx < 0 || nx >= size_x || ny < 0 || ny >= size_y) {
                continue;
            }
            int64_t nxt_idx = coordToIndex(nx, ny);
            if (closed.count(nxt_idx) > 0) {
                continue;
            }
            if (!isFree(nx, ny)) {
                continue;
            }

            double sc = stepCost(current.x, current.y, nx, ny);
            double base_g = current.g + sc;

            double rho = localDensity(nx, ny);
            AdaptiveWeights aw = adaptiveWeights(rho);

            double phi = proximityNorm(nx, ny);
            double c_obs = aw.w_obs * neutral_cost_ * phi * phi;

            double c_goal_dir = directionPenalty(current.x, current.y, nx, ny, gx, gy);
            double c_turn = turnPenalty(current.parent_x, current.parent_y,
                                        current.x, current.y, nx, ny);
            double c_dir = aw.w_dir * neutral_cost_ *
                           (kBetaGoal * c_goal_dir + kBetaTurn * c_turn);

            double g_new = base_g + c_obs + c_dir;

            auto bg_nxt = best_g.find(nxt_idx);
            if (bg_nxt != best_g.end() && g_new >= bg_nxt->second) {
                continue;
            }

            best_g[nxt_idx] = g_new;

            AStarNode next_node;
            next_node.x = nx;
            next_node.y = ny;
            next_node.parent_x = current.x;
            next_node.parent_y = current.y;
            next_node.g = g_new;
            next_node.f = g_new + weightedHeuristic(nx, ny, gx, gy, aw.alpha);
            open_list.push(next_node);
        }

        closed[cur_idx] = current;
        expanded_count++;
    }

    ROS_WARN("[ImprovedAStar] Failed to find a plan (expanded %d nodes).", expanded_count);
    return false;
}

int64_t ImprovedAStarPlanner::coordToIndex(int32_t x, int32_t y) const {
    return static_cast<int64_t>(y) * static_cast<int64_t>(costmap_->getSizeInCellsX()) +
           static_cast<int64_t>(x);
}

std::vector<std::pair<int32_t, int32_t>> ImprovedAStarPlanner::goalBiasedNeighbors(
    int32_t x, int32_t y, int32_t gx, int32_t gy) const {

    std::vector<std::pair<int32_t, int32_t>> result;
    result.reserve(6);

    result.emplace_back(x + 1, y);
    result.emplace_back(x - 1, y);
    result.emplace_back(x, y + 1);
    result.emplace_back(x, y - 1);

    int32_t dx_goal = gx - x;
    int32_t dy_goal = gy - y;

    if (std::abs(dx_goal) >= std::abs(dy_goal) && dx_goal != 0) {
        int32_t sx = (dx_goal > 0) ? 1 : -1;
        result.emplace_back(x + sx, y + 1);
        result.emplace_back(x + sx, y - 1);
    } else if (dy_goal != 0) {
        int32_t sy = (dy_goal > 0) ? 1 : -1;
        result.emplace_back(x + 1, y + sy);
        result.emplace_back(x - 1, y + sy);
    }

    return result;
}

double ImprovedAStarPlanner::localDensity(int32_t x, int32_t y) const {
    int32_t size_x = static_cast<int32_t>(costmap_->getSizeInCellsX());
    int32_t size_y = static_cast<int32_t>(costmap_->getSizeInCellsY());

    int32_t n_total = 0;
    int32_t n_occ = 0;
    int32_t n_inflation = 0;

    for (int32_t yy = y - density_radius_; yy <= y + density_radius_; ++yy) {
        if (yy < 0 || yy >= size_y) {
            continue;
        }
        for (int32_t xx = x - density_radius_; xx <= x + density_radius_; ++xx) {
            if (xx < 0 || xx >= size_x) {
                continue;
            }
            n_total++;
            uint8_t cost = costmap_->getCost(
                static_cast<uint32_t>(xx), static_cast<uint32_t>(yy));
            if (cost >= costmap_2d::LETHAL_OBSTACLE) {
                n_occ++;
            } else if (cost > costmap_2d::FREE_SPACE &&
                       cost < costmap_2d::LETHAL_OBSTACLE) {
                n_inflation++;
            }
        }
    }

    if (n_total == 0) {
        return 0.0;
    }
    double rho = (static_cast<double>(n_occ) +
                  inflation_lambda_ * static_cast<double>(n_inflation)) /
                 static_cast<double>(n_total);
    return std::max(0.0, std::min(1.0, rho));
}

AdaptiveWeights ImprovedAStarPlanner::adaptiveWeights(double rho) const {
    AdaptiveWeights aw;
    if (rho < threshold_open_) {
        aw.w_obs = w_obs_open_;
        aw.w_dir = w_dir_open_;
        aw.alpha = alpha_open_;
    } else if (rho >= threshold_narrow_) {
        aw.w_obs = w_obs_narrow_;
        aw.w_dir = w_dir_narrow_;
        aw.alpha = alpha_narrow_;
    } else {
        aw.w_obs = w_obs_normal_;
        aw.w_dir = w_dir_normal_;
        aw.alpha = alpha_normal_;
    }
    return aw;
}

double ImprovedAStarPlanner::directionPenalty(
    int32_t cx, int32_t cy, int32_t nx, int32_t ny,
    int32_t gx, int32_t gy) const {

    double step_dx = static_cast<double>(nx - cx);
    double step_dy = static_cast<double>(ny - cy);
    double goal_dx = static_cast<double>(gx - cx);
    double goal_dy = static_cast<double>(gy - cy);

    double step_norm = std::hypot(step_dx, step_dy);
    double goal_norm = std::hypot(goal_dx, goal_dy);
    if (step_norm <= kEps || goal_norm <= kEps) {
        return 0.0;
    }

    double cos_theta = (step_dx * goal_dx + step_dy * goal_dy) /
                       (step_norm * goal_norm);
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    return 1.0 - cos_theta;
}

double ImprovedAStarPlanner::turnPenalty(
    int32_t px, int32_t py, int32_t cx, int32_t cy,
    int32_t nx, int32_t ny) const {

    if (px < 0) {
        return 0.0;
    }

    double v1x = static_cast<double>(cx - px);
    double v1y = static_cast<double>(cy - py);
    double v2x = static_cast<double>(nx - cx);
    double v2y = static_cast<double>(ny - cy);

    double n1 = std::hypot(v1x, v1y);
    double n2 = std::hypot(v2x, v2y);
    if (n1 <= kEps || n2 <= kEps) {
        return 0.0;
    }

    double cos_theta = (v1x * v2x + v1y * v2y) / (n1 * n2);
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    return 1.0 - cos_theta;
}

double ImprovedAStarPlanner::weightedHeuristic(
    int32_t x, int32_t y, int32_t gx, int32_t gy, double alpha) const {

    double dx = std::abs(static_cast<double>(gx - x));
    double dy = std::abs(static_cast<double>(gy - y));
    double min_d = std::min(dx, dy);
    double max_d = std::max(dx, dy);
    double octile = max_d + kOctileDiag * min_d;
    return alpha * neutral_cost_ * octile;
}

double ImprovedAStarPlanner::proximityNorm(int32_t x, int32_t y) const {
    uint8_t cost = costmap_->getCost(
        static_cast<uint32_t>(x), static_cast<uint32_t>(y));
    if (cost >= costmap_2d::LETHAL_OBSTACLE) {
        return 1.0;
    }
    double normalized = static_cast<double>(cost) /
                        static_cast<double>(costmap_2d::LETHAL_OBSTACLE);
    return normalized;
}

double ImprovedAStarPlanner::stepCost(
    int32_t cx, int32_t cy, int32_t nx, int32_t ny) const {

    int32_t dx = std::abs(nx - cx);
    int32_t dy = std::abs(ny - cy);
    if (dx + dy == 2) {
        return kSqrt2;
    }
    return 1.0;
}

std::vector<std::pair<double, double>> ImprovedAStarPlanner::deduplicatePath(
    const std::vector<std::pair<double, double>>& path) const {
    std::vector<std::pair<double, double>> deduped;
    deduped.reserve(path.size());
    for (const auto& point : path) {
        if (!deduped.empty()) {
            double dx = point.first - deduped.back().first;
            double dy = point.second - deduped.back().second;
            if (std::hypot(dx, dy) <= kEps) {
                continue;
            }
        }
        deduped.emplace_back(point);
    }
    return deduped;
}

bool ImprovedAStarPlanner::isPointSafeWorld(double world_x, double world_y) const {
    uint32_t map_x = 0;
    uint32_t map_y = 0;
    if (!costmap_->worldToMap(world_x, world_y, map_x, map_y)) {
        return false;
    }
    uint8_t cost = costmap_->getCost(map_x, map_y);
    if (static_cast<int32_t>(cost) >= lethal_cost_threshold_) {
        return false;
    }
    return pointClearanceMeters(world_x, world_y) + kEps >= smoothing_safety_clearance_m_;
}

double ImprovedAStarPlanner::pointClearanceMeters(double world_x, double world_y) const {
    uint32_t map_x = 0;
    uint32_t map_y = 0;
    if (!costmap_->worldToMap(world_x, world_y, map_x, map_y)) {
        return 0.0;
    }

    int32_t size_x = static_cast<int32_t>(costmap_->getSizeInCellsX());
    int32_t size_y = static_cast<int32_t>(costmap_->getSizeInCellsY());
    int32_t cell_x = static_cast<int32_t>(map_x);
    int32_t cell_y = static_cast<int32_t>(map_y);

    uint8_t center_cost = costmap_->getCost(map_x, map_y);
    if (center_cost >= costmap_2d::LETHAL_OBSTACLE) {
        return 0.0;
    }

    int32_t max_radius_cells = static_cast<int32_t>(
        std::ceil(std::max(smoothing_danger_clearance_m_, smoothing_safety_clearance_m_) /
                  std::max(costmap_->getResolution(), kEps))) + 2;
    double min_distance = std::numeric_limits<double>::infinity();

    for (int32_t dy = -max_radius_cells; dy <= max_radius_cells; ++dy) {
        int32_t ny = cell_y + dy;
        if (ny < 0 || ny >= size_y) {
            continue;
        }
        for (int32_t dx = -max_radius_cells; dx <= max_radius_cells; ++dx) {
            int32_t nx = cell_x + dx;
            if (nx < 0 || nx >= size_x) {
                continue;
            }
            uint8_t cost = costmap_->getCost(static_cast<uint32_t>(nx), static_cast<uint32_t>(ny));
            if (cost < costmap_2d::LETHAL_OBSTACLE) {
                continue;
            }
            double obs_x = 0.0;
            double obs_y = 0.0;
            costmap_->mapToWorld(static_cast<uint32_t>(nx), static_cast<uint32_t>(ny), obs_x, obs_y);
            double distance = std::hypot(world_x - obs_x, world_y - obs_y);
            min_distance = std::min(min_distance, distance);
        }
    }

    if (!std::isfinite(min_distance)) {
        return static_cast<double>(max_radius_cells) * costmap_->getResolution();
    }
    return min_distance;
}

bool ImprovedAStarPlanner::isSegmentSafeWorld(double start_x, double start_y,
                                              double end_x, double end_y) const {
    double segment_length = std::hypot(end_x - start_x, end_y - start_y);
    double sample_step = std::max(0.5 * costmap_->getResolution(), 0.01);
    int32_t sample_count = std::max(1, static_cast<int32_t>(std::ceil(segment_length / sample_step)));
    for (int32_t index = 0; index <= sample_count; ++index) {
        double t = static_cast<double>(index) / static_cast<double>(sample_count);
        double sample_x = start_x + t * (end_x - start_x);
        double sample_y = start_y + t * (end_y - start_y);
        if (!isPointSafeWorld(sample_x, sample_y)) {
            return false;
        }
    }
    return true;
}

std::vector<std::pair<double, double>> ImprovedAStarPlanner::shortcutPath(
    const std::vector<std::pair<double, double>>& path) const {
    if (path.size() <= 2 || smoothing_max_jump_m_ <= kEps) {
        return path;
    }

    std::vector<std::pair<double, double>> shortcut;
    shortcut.reserve(path.size());
    size_t anchor_index = 0;
    shortcut.emplace_back(path.front());

    while (anchor_index + 1 < path.size()) {
        size_t best_index = anchor_index + 1;
        for (size_t candidate = path.size() - 1; candidate > anchor_index + 1; --candidate) {
            double dx = path[candidate].first - path[anchor_index].first;
            double dy = path[candidate].second - path[anchor_index].second;
            double jump_distance = std::hypot(dx, dy);
            if (jump_distance > smoothing_max_jump_m_) {
                continue;
            }
            if (isSegmentSafeWorld(path[anchor_index].first, path[anchor_index].second,
                                   path[candidate].first, path[candidate].second)) {
                best_index = candidate;
                break;
            }
        }
        shortcut.emplace_back(path[best_index]);
        anchor_index = best_index;
    }

    return deduplicatePath(shortcut);
}

std::vector<std::pair<double, double>> ImprovedAStarPlanner::smoothPathBSpline(
    const std::vector<std::pair<double, double>>& path) const {
    if (path.size() < 4) {
        return path;
    }

    std::vector<std::pair<double, double>> smoothed;
    smoothed.reserve(path.size() * static_cast<size_t>(smoothing_bspline_upsample_ + 1));
    smoothed.emplace_back(path.front());

    for (size_t index = 0; index + 3 < path.size(); ++index) {
        const auto& p0 = path[index];
        const auto& p1 = path[index + 1];
        const auto& p2 = path[index + 2];
        const auto& p3 = path[index + 3];

        for (int32_t step = 1; step <= smoothing_bspline_upsample_; ++step) {
            double t = static_cast<double>(step) /
                       static_cast<double>(smoothing_bspline_upsample_ + 1);
            double t2 = t * t;
            double t3 = t2 * t;

            double basis0 = (-t3 + 3.0 * t2 - 3.0 * t + 1.0) / 6.0;
            double basis1 = (3.0 * t3 - 6.0 * t2 + 4.0) / 6.0;
            double basis2 = (-3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0) / 6.0;
            double basis3 = t3 / 6.0;

            double smooth_x = basis0 * p0.first + basis1 * p1.first +
                              basis2 * p2.first + basis3 * p3.first;
            double smooth_y = basis0 * p0.second + basis1 * p1.second +
                              basis2 * p2.second + basis3 * p3.second;
            smoothed.emplace_back(smooth_x, smooth_y);
        }
        smoothed.emplace_back(p2);
    }

    smoothed.emplace_back(path.back());
    return deduplicatePath(smoothed);
}

std::vector<std::pair<double, double>> ImprovedAStarPlanner::smoothPathWithFallback(
    const std::vector<std::pair<double, double>>& path) const {
    std::vector<std::pair<double, double>> deduped = deduplicatePath(path);
    if (deduped.size() <= 2) {
        return deduped;
    }

    std::vector<std::pair<double, double>> shortcut = shortcutPath(deduped);
    std::vector<std::pair<double, double>> candidate = smoothPathBSpline(shortcut);
    if (candidate.size() < 2) {
        return shortcut;
    }

    std::vector<std::pair<double, double>> final_path;
    final_path.reserve(candidate.size());
    final_path.emplace_back(candidate.front());

    for (size_t index = 1; index < candidate.size(); ++index) {
        const auto& point = candidate[index];
        const auto& prev = final_path.back();
        bool keep_candidate = isPointSafeWorld(point.first, point.second) &&
                              isSegmentSafeWorld(prev.first, prev.second,
                                                 point.first, point.second);
        if (!keep_candidate) {
            if (index < shortcut.size()) {
                const auto& fallback = shortcut[index];
                if (isPointSafeWorld(fallback.first, fallback.second) &&
                    isSegmentSafeWorld(prev.first, prev.second,
                                       fallback.first, fallback.second)) {
                    final_path.emplace_back(fallback);
                }
            }
            continue;
        }

        double dx = point.first - prev.first;
        double dy = point.second - prev.second;
        if (std::hypot(dx, dy) < smoothing_min_point_spacing_m_) {
            continue;
        }
        final_path.emplace_back(point);
    }

    if (final_path.back() != shortcut.back()) {
        if (isSegmentSafeWorld(final_path.back().first, final_path.back().second,
                               shortcut.back().first, shortcut.back().second)) {
            final_path.emplace_back(shortcut.back());
        }
    }

    if (final_path.size() < 2) {
        return shortcut;
    }
    return deduplicatePath(final_path);
}

ImprovedAStarPlanner::PathMetrics ImprovedAStarPlanner::computePathMetrics(
    const std::vector<std::pair<double, double>>& path) const {
    PathMetrics metrics;
    if (path.empty()) {
        return metrics;
    }

    metrics.min_clearance_m = std::numeric_limits<double>::infinity();
    double clearance_sum = 0.0;
    int32_t valid_clearance_count = 0;

    for (size_t index = 0; index < path.size(); ++index) {
        double clearance = pointClearanceMeters(path[index].first, path[index].second);
        metrics.min_clearance_m = std::min(metrics.min_clearance_m, clearance);
        clearance_sum += clearance;
        valid_clearance_count++;
        if (clearance + kEps < smoothing_danger_clearance_m_) {
            metrics.danger_point_count++;
        }

        if (index > 0) {
            double dx = path[index].first - path[index - 1].first;
            double dy = path[index].second - path[index - 1].second;
            metrics.path_length_m += std::hypot(dx, dy);
        }

        if (index > 0 && index + 1 < path.size()) {
            double prev_dx = path[index].first - path[index - 1].first;
            double prev_dy = path[index].second - path[index - 1].second;
            double next_dx = path[index + 1].first - path[index].first;
            double next_dy = path[index + 1].second - path[index].second;
            double prev_norm = std::hypot(prev_dx, prev_dy);
            double next_norm = std::hypot(next_dx, next_dy);
            if (prev_norm <= kEps || next_norm <= kEps) {
                continue;
            }
            double cos_theta = (prev_dx * next_dx + prev_dy * next_dy) / (prev_norm * next_norm);
            cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
            double angle = std::acos(cos_theta);
            metrics.sum_abs_turn_rad += std::abs(angle);
            if (angle + kEps >= smoothing_turn_angle_threshold_rad_) {
                metrics.turn_point_count++;
            }
        }
    }

    if (valid_clearance_count > 0) {
        metrics.mean_clearance_m = clearance_sum / static_cast<double>(valid_clearance_count);
    }
    if (!std::isfinite(metrics.min_clearance_m)) {
        metrics.min_clearance_m = 0.0;
    }
    return metrics;
}

void ImprovedAStarPlanner::logPathComparison(const PathMetrics& raw_metrics,
                                             const PathMetrics& smooth_metrics,
                                             size_t raw_waypoints,
                                             size_t smooth_waypoints,
                                             int32_t expanded_count) const {
    ROS_INFO("[ImprovedAStar][RawMetrics] expanded=%d waypoints=%zu path_length_m=%.3f "
             "min_clearance_m=%.3f mean_clearance_m=%.3f sum_abs_turn_rad=%.3f "
             "turn_point_count=%d danger_point_count=%d",
             expanded_count, raw_waypoints,
             raw_metrics.path_length_m,
             raw_metrics.min_clearance_m,
             raw_metrics.mean_clearance_m,
             raw_metrics.sum_abs_turn_rad,
             raw_metrics.turn_point_count,
             raw_metrics.danger_point_count);

    ROS_INFO("[ImprovedAStar][SmoothMetrics] expanded=%d waypoints=%zu path_length_m=%.3f "
             "min_clearance_m=%.3f mean_clearance_m=%.3f sum_abs_turn_rad=%.3f "
             "turn_point_count=%d danger_point_count=%d",
             expanded_count, smooth_waypoints,
             smooth_metrics.path_length_m,
             smooth_metrics.min_clearance_m,
             smooth_metrics.mean_clearance_m,
             smooth_metrics.sum_abs_turn_rad,
             smooth_metrics.turn_point_count,
             smooth_metrics.danger_point_count);
}

bool ImprovedAStarPlanner::isFree(int32_t x, int32_t y) const {
    uint8_t cost = costmap_->getCost(
        static_cast<uint32_t>(x), static_cast<uint32_t>(y));
    // lethal_cost_threshold_ controls how close to walls the planner may route.
    // 253 (INSCRIBED): allows full inflation zone, may produce paths too close to walls.
    // 128: blocks the inner half of the inflation zone, keeps ~0.15m extra clearance.
    return static_cast<int32_t>(cost) < lethal_cost_threshold_;
}

}  // namespace improved_astar
