#include <multi_floor_nav/improved_astar_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <tf/tf.h>
#include <algorithm>
#include <limits>

PLUGINLIB_EXPORT_CLASS(improved_astar::ImprovedAStarPlanner, nav_core::BaseGlobalPlanner)

namespace improved_astar {

namespace {
constexpr double kSqrt2 = 1.41421356;
constexpr double kOctileDiag = 0.41421356;  // sqrt(2) - 1
constexpr double kEps = 1e-9;
constexpr double kBetaGoal = 0.45;
constexpr double kBetaTurn = 0.95;
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

    density_radius_ = std::max(1, density_radius_);
    neutral_cost_ = std::max(1e-6, neutral_cost_);
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

            plan.reserve(waypoints.size());
            for (const auto& wp : waypoints) {
                geometry_msgs::PoseStamped pose;
                pose.header = goal.header;
                pose.pose.position.x = wp.first;
                pose.pose.position.y = wp.second;
                pose.pose.position.z = 0.0;
                pose.pose.orientation = goal.pose.orientation;
                plan.emplace_back(pose);
            }

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

bool ImprovedAStarPlanner::isFree(int32_t x, int32_t y) const {
    uint8_t cost = costmap_->getCost(
        static_cast<uint32_t>(x), static_cast<uint32_t>(y));
    // lethal_cost_threshold_ controls how close to walls the planner may route.
    // 253 (INSCRIBED): allows full inflation zone, may produce paths too close to walls.
    // 128: blocks the inner half of the inflation zone, keeps ~0.15m extra clearance.
    return static_cast<int32_t>(cost) < lethal_cost_threshold_;
}

}  // namespace improved_astar
