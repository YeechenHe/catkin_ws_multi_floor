#ifndef IMPROVED_ASTAR_PLANNER_H
#define IMPROVED_ASTAR_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <string>
#include <utility>

namespace improved_astar {

struct AStarNode {
    int32_t x = 0;
    int32_t y = 0;
    int32_t parent_x = -1;
    int32_t parent_y = -1;
    double g = 0.0;
    double f = 0.0;

    bool operator>(const AStarNode& other) const { return f > other.f; }
};

struct AdaptiveWeights {
    double w_obs = 0.0;
    double w_dir = 0.0;
    double alpha = 1.0;
};

class ImprovedAStarPlanner : public nav_core::BaseGlobalPlanner {
public:
    ImprovedAStarPlanner();
    explicit ImprovedAStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    int64_t coordToIndex(int32_t x, int32_t y) const;

    std::vector<std::pair<int32_t, int32_t>> goalBiasedNeighbors(
        int32_t x, int32_t y, int32_t gx, int32_t gy) const;

    double localDensity(int32_t x, int32_t y) const;

    AdaptiveWeights adaptiveWeights(double rho) const;

    double directionPenalty(int32_t cx, int32_t cy,
                           int32_t nx, int32_t ny,
                           int32_t gx, int32_t gy) const;

    double turnPenalty(int32_t px, int32_t py,
                      int32_t cx, int32_t cy,
                      int32_t nx, int32_t ny) const;

    double weightedHeuristic(int32_t x, int32_t y,
                             int32_t gx, int32_t gy,
                             double alpha) const;

    double proximityNorm(int32_t x, int32_t y) const;

    double stepCost(int32_t cx, int32_t cy, int32_t nx, int32_t ny) const;

    bool isFree(int32_t x, int32_t y) const;

    costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
    costmap_2d::Costmap2D* costmap_ = nullptr;
    bool initialized_ = false;

    int32_t density_radius_ = 2;
    double inflation_lambda_ = 0.5;
    double threshold_open_ = 0.20;
    double threshold_narrow_ = 0.45;

    double w_obs_open_ = 0.15;
    double w_obs_normal_ = 0.45;
    double w_obs_narrow_ = 0.90;

    double w_dir_open_ = 0.35;
    double w_dir_normal_ = 0.25;
    double w_dir_narrow_ = 0.15;

    double alpha_open_ = 1.35;
    double alpha_normal_ = 1.15;
    double alpha_narrow_ = 1.00;

    double neutral_cost_ = 50.0;

    // Cells with cost >= lethal_cost_threshold_ are treated as impassable.
    // Default 253 (INSCRIBED_INFLATED_OBSTACLE) allows the full inflation zone.
    // Lower values (e.g. 128) force the planner to keep a larger clearance from walls.
    int32_t lethal_cost_threshold_ = 253;
};

}  // namespace improved_astar

#endif  // IMPROVED_ASTAR_PLANNER_H
