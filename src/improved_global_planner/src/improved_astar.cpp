/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Based on the original global_planner A* implementation
 *  Copyright (c) 2008, 2013, Willow Garage
 *  All rights reserved.
 *
 *  本文件提供 ImprovedAStarExpansion，用于后续对 A* 算法进行改进实验。
 *  当前实现基本等价于 AStarExpansion，只是在关键位置预留了可以修改启发函数、
 *  代价权重等的 hook。
 *
 *********************************************************************/
#include <improved_global_planner/improved_astar.h>
#include <costmap_2d/cost_values.h>

namespace improved_global_planner {

ImprovedAStarExpansion::ImprovedAStarExpansion(PotentialCalculator* p_calc, int xs, int ys)
    : Expander(p_calc, xs, ys) {}

bool ImprovedAStarExpansion::calculatePotentials(unsigned char* costs,
                                                 double start_x, double start_y,
                                                 double end_x,   double end_y,
                                                 int cycles, float* potential) {
  queue_.clear();
  int start_i = toIndex(start_x, start_y);
  queue_.push_back(Index(start_i, 0.0f));

  std::fill(potential, potential + ns_, POT_HIGH);
  potential[start_i] = 0.0f;

  int goal_i = toIndex(end_x, end_y);
  int cycle = 0;

  while (!queue_.empty() && cycle < cycles) {
    Index top = queue_.front();
    std::pop_heap(queue_.begin(), queue_.end(), greater1());
    queue_.pop_back();

    int i = top.i;
    if (i == goal_i)
      return true;

    // 这里的 4 邻域扩展策略也可以作为未来的改进点（例如 8 邻域、方向性权重等）
    add(costs, potential, potential[i], i + 1,    end_x, end_y);
    add(costs, potential, potential[i], i - 1,    end_x, end_y);
    add(costs, potential, potential[i], i + nx_,  end_x, end_y);
    add(costs, potential, potential[i], i - nx_,  end_x, end_y);

    ++cycle;
  }

  return false;
}

void ImprovedAStarExpansion::add(unsigned char* costs,
                                 float* potential,
                                 float prev_potential,
                                 int next_i,
                                 int end_x,
                                 int end_y) {
  if (next_i < 0 || next_i >= ns_)
    return;

  if (potential[next_i] < POT_HIGH)
    return;

  if (costs[next_i] >= lethal_cost_ &&
      !(unknown_ && costs[next_i] == costmap_2d::NO_INFORMATION))
    return;

  // === 预留改进点 1：局部代价模型 ===
  // 目前与原始 A* 相同：在原有 potential 基础上增加 (cost + neutral_cost_)
  // 你可以在这里改变单步代价的计算方式，例如加入方向、曲率、离障碍距离等。
  potential[next_i] =
      p_calc_->calculatePotential(potential,
                                  costs[next_i] + neutral_cost_,
                                  next_i,
                                  prev_potential);

  // 计算当前格点 (x, y) 与目标 (end_x, end_y) 的启发式距离
  int x = next_i % nx_;
  int y = next_i / nx_;

  // === 预留改进点 2：启发函数 h(x) ===
  // 当前是 |dx| + |dy|（Manhattan），再乘以 neutral_cost_。
  // 你可以在这里替换成自己的启发函数（欧氏距离、加方向代价等）。
  float dx = static_cast<float>(end_x - x);
  float dy = static_cast<float>(end_y - y);
  float heuristic = std::fabs(dx) + std::fabs(dy);

  float f_cost = potential[next_i] + heuristic * neutral_cost_;

  queue_.push_back(Index(next_i, f_cost));
  std::push_heap(queue_.begin(), queue_.end(), greater1());
}

}  // namespace improved_global_planner

