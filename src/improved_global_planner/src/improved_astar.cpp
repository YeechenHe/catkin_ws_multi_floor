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
#include <cmath>

namespace improved_global_planner {

ImprovedAStarExpansion::ImprovedAStarExpansion(PotentialCalculator* p_calc, int xs, int ys)
    : Expander(p_calc, xs, ys),
      density_radius_(2),
      inflation_lambda_(0.5f),
      threshold_open_(0.20f),
      threshold_narrow_(0.45f),
      w_obs_open_(0.15f),
      w_obs_normal_(0.45f),
      w_obs_narrow_(0.90f),
      w_dir_open_(0.35f),
      w_dir_normal_(0.25f),
      w_dir_narrow_(0.15f),
      alpha_open_(1.35f),
      alpha_normal_(1.15f),
      alpha_narrow_(1.00f) {}

void ImprovedAStarExpansion::setAdaptiveParams(int density_radius,
                                               float inflation_lambda,
                                               float threshold_open,
                                               float threshold_narrow,
                                               float w_obs_open,
                                               float w_obs_normal,
                                               float w_obs_narrow,
                                               float w_dir_open,
                                               float w_dir_normal,
                                               float w_dir_narrow,
                                               float alpha_open,
                                               float alpha_normal,
                                               float alpha_narrow) {
  density_radius_ = std::max(1, density_radius);
  inflation_lambda_ = std::max(0.0f, inflation_lambda);
  threshold_open_ = std::max(0.0f, std::min(1.0f, threshold_open));
  threshold_narrow_ = std::max(threshold_open_, std::min(1.0f, threshold_narrow));

  w_obs_open_ = std::max(0.0f, w_obs_open);
  w_obs_normal_ = std::max(0.0f, w_obs_normal);
  w_obs_narrow_ = std::max(0.0f, w_obs_narrow);
  w_dir_open_ = std::max(0.0f, w_dir_open);
  w_dir_normal_ = std::max(0.0f, w_dir_normal);
  w_dir_narrow_ = std::max(0.0f, w_dir_narrow);

  // alpha>=1 更符合 Weighted A* 的常用设定
  alpha_open_ = std::max(1.0f, alpha_open);
  alpha_normal_ = std::max(1.0f, alpha_normal);
  alpha_narrow_ = std::max(1.0f, alpha_narrow);
}

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
  const int goal_x = static_cast<int>(end_x);
  const int goal_y = static_cast<int>(end_y);
  int cycle = 0;

  while (!queue_.empty() && cycle < cycles) {
    Index top = queue_.front();
    std::pop_heap(queue_.begin(), queue_.end(), greater1());
    queue_.pop_back();

    int i = top.i;
    if (i == goal_i)
      return true;

    // 基础 4 邻域（上、下、左、右）
    add(costs, potential, potential[i], i, i + 1,    end_x, end_y);
    add(costs, potential, potential[i], i, i - 1,    end_x, end_y);
    add(costs, potential, potential[i], i, i + nx_,  end_x, end_y);
    add(costs, potential, potential[i], i, i - nx_,  end_x, end_y);

    // 方向引导 2 邻域：在目标主方向上补两个对角扩展，形成 6 邻域
    // 规则：
    // 1) 若 |dx| >= |dy|，主方向按 x（左/右）选择两条“前向对角”
    // 2) 否则主方向按 y（上/下）选择两条“前向对角”
    const int x = i % nx_;
    const int y = i / nx_;
    const int dx_goal = goal_x - x;
    const int dy_goal = goal_y - y;
    int diag_offset_a = 0;
    int diag_offset_b = 0;

    if (std::abs(dx_goal) >= std::abs(dy_goal) && dx_goal != 0) {
      const int sx = dx_goal > 0 ? 1 : -1;
      diag_offset_a = sx + nx_;  // 右下 或 左下
      diag_offset_b = sx - nx_;  // 右上 或 左上
    } else if (dy_goal != 0) {
      const int sy = dy_goal > 0 ? 1 : -1;
      diag_offset_a = sy * nx_ + 1;  // 下右 或 上右
      diag_offset_b = sy * nx_ - 1;  // 下左 或 上左
    }

    if (diag_offset_a != 0)
      add(costs, potential, potential[i], i, i + diag_offset_a, end_x, end_y);
    if (diag_offset_b != 0)
      add(costs, potential, potential[i], i, i + diag_offset_b, end_x, end_y);

    ++cycle;
  }

  return false;
}

void ImprovedAStarExpansion::add(unsigned char* costs,
                                 float* potential,
                                 float prev_potential,
                                 int current_i,
                                 int next_i,
                                 int end_x,
                                 int end_y) {
  if (next_i < 0 || next_i >= ns_)
    return;

  const int current_x = current_i % nx_;
  const int current_y = current_i / nx_;
  const int next_x = next_i % nx_;
  const int next_y = next_i / nx_;

  // 防止跨行越界（例如最右侧 +1 跑到下一行开头）
  if (std::abs(next_x - current_x) > 1 || std::abs(next_y - current_y) > 1)
    return;

  if (potential[next_i] < POT_HIGH)
    return;

  if (costs[next_i] >= lethal_cost_ &&
      !(unknown_ && costs[next_i] == costmap_2d::NO_INFORMATION))
    return;

  // 局部代价模型（基础项）：
  // - 基础与原始 A* 一致
  // - 对角步增加一个步长惩罚（约 sqrt(2)-1），避免对角与直行代价完全相同
  const bool is_diagonal = (std::abs(next_x - current_x) == 1) &&
                           (std::abs(next_y - current_y) == 1);
  const float diagonal_penalty = is_diagonal ? 0.41421356f * neutral_cost_ : 0.0f;
  const float base_g =
      p_calc_->calculatePotential(potential,
                                  costs[next_i] + neutral_cost_,
                                  next_i,
                                  prev_potential) + diagonal_penalty;

  // ------------------------------------------------------------------
  // 基于局部障碍物密度 rho 的三模式自适应权重
  //
  // rho = (N_occ + lambda * N_inflation) / N_total,  rho in [0,1]
  //
  // 模式划分（粗略阈值，后续可调）：
  // - 宽阔: rho < T1
  // - 常规: T1 <= rho < T2
  // - 狭窄: rho >= T2
  //
  // 目标：
  // - 宽阔：更偏向最短路与搜索效率
  // - 常规：安全与效率平衡
  // - 狭窄：安全优先（更强近障碍惩罚）
  // ------------------------------------------------------------------
  const int density_radius = density_radius_;        // 局部窗口半径（窗口边长=2r+1）
  const float inflation_lambda = inflation_lambda_;  // 膨胀区计数权重
  const float T1 = threshold_open_;                  // 宽阔/常规阈值
  const float T2 = threshold_narrow_;                // 常规/狭窄阈值

  int n_total = 0;
  int n_occ = 0;
  int n_inflation = 0;
  for (int wy = next_y - density_radius; wy <= next_y + density_radius; ++wy) {
    if (wy < 0 || wy >= ny_) continue;
    for (int wx = next_x - density_radius; wx <= next_x + density_radius; ++wx) {
      if (wx < 0 || wx >= nx_) continue;
      ++n_total;
      const int wi = toIndex(wx, wy);
      const unsigned char c = costs[wi];
      if (c >= lethal_cost_) {
        ++n_occ;
      } else if (c > neutral_cost_) {
        ++n_inflation;
      }
    }
  }

  const float rho = (n_total > 0)
      ? (static_cast<float>(n_occ) + inflation_lambda * static_cast<float>(n_inflation)) /
            static_cast<float>(n_total)
      : 0.0f;

  // 自适应权重（可调初值）
  float w_obs = w_obs_normal_;   // 近障碍惩罚权重
  float w_dir = w_dir_normal_;   // 目标方向对齐权重
  float alpha = alpha_normal_;   // 启发项放大系数（Weighted A*）
  if (rho < T1) {
    // 宽阔：更激进，偏向最短路
    w_obs = w_obs_open_;
    w_dir = w_dir_open_;
    alpha = alpha_open_;
  } else if (rho >= T2) {
    // 狭窄：更保守，优先安全
    w_obs = w_obs_narrow_;
    w_dir = w_dir_narrow_;
    alpha = alpha_narrow_;
  }

  // ------------------------------------------------------------------
  // 核心公式 1：障碍惩罚项（安全）
  // C_obs = w_obs * neutral_cost * (c_norm)^2
  // 其中 c_norm = min(cost, lethal-1) / (lethal-1)
  // 使用平方项让高代价区域惩罚增长更快，减少贴障规划。
  // ------------------------------------------------------------------
  const float safe_lethal = static_cast<float>(std::max(1, static_cast<int>(lethal_cost_) - 1));
  const float c_norm = std::min(static_cast<float>(costs[next_i]), safe_lethal) / safe_lethal;
  const float C_obs = w_obs * static_cast<float>(neutral_cost_) * c_norm * c_norm;

  // ------------------------------------------------------------------
  // 核心公式 2：方向对齐惩罚（效率）
  // C_dir = w_dir * neutral_cost * (1 - cos(theta))
  // theta 为“当前移动方向”和“指向目标方向”夹角。
  // cos(theta) 越接近 1（越朝向目标），惩罚越小。
  // ------------------------------------------------------------------
  const float step_dx = static_cast<float>(next_x - current_x);
  const float step_dy = static_cast<float>(next_y - current_y);
  const float goal_dx = static_cast<float>(end_x - current_x);
  const float goal_dy = static_cast<float>(end_y - current_y);
  const float step_norm = std::sqrt(step_dx * step_dx + step_dy * step_dy);
  const float goal_norm = std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy);
  float cos_theta = 1.0f;
  if (step_norm > 1e-6f && goal_norm > 1e-6f) {
    cos_theta = (step_dx * goal_dx + step_dy * goal_dy) / (step_norm * goal_norm);
    cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));
  }
  const float C_dir = w_dir * static_cast<float>(neutral_cost_) * (1.0f - cos_theta);

  // 综合后的 g 代价
  const float adaptive_g = base_g + C_obs + C_dir;
  potential[next_i] = adaptive_g;

  // ------------------------------------------------------------------
  // 核心公式 3：加权启发项（速度）
  // 使用 Octile 距离（适配含对角扩展场景）：
  // h = max(dx,dy) + (sqrt(2)-1) * min(dx,dy)
  // f = g + alpha * h * neutral_cost
  // ------------------------------------------------------------------
  const float dx = std::fabs(static_cast<float>(end_x - next_x));
  const float dy = std::fabs(static_cast<float>(end_y - next_y));
  const float min_d = std::min(dx, dy);
  const float max_d = std::max(dx, dy);
  const float heuristic = max_d + 0.41421356f * min_d;

  const float f_cost = adaptive_g + alpha * heuristic * static_cast<float>(neutral_cost_);

  queue_.push_back(Index(next_i, f_cost));
  std::push_heap(queue_.begin(), queue_.end(), greater1());
}

}  // namespace improved_global_planner

