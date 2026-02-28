/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Based on the original global_planner A* implementation
 *  Copyright (c) 2008, 2013, Willow Garage
 *  All rights reserved.
 *
 *********************************************************************/
#ifndef _IMPROVED_ASTAR_H
#define _IMPROVED_ASTAR_H

#include <improved_global_planner/astar.h>

namespace improved_global_planner {

/**
 * @brief Experimental / improved A* expander.
 *
 * 当前实现与 AStarExpansion 行为保持一致，只是为后续“改进 A*”预留结构。
 * 你可以在 improved_astar.cpp 中修改启发函数、代价模型等，而不影响原始 AStarExpansion。
 */
class ImprovedAStarExpansion : public Expander {
  public:
    ImprovedAStarExpansion(PotentialCalculator* p_calc, int nx, int ny);
    virtual ~ImprovedAStarExpansion() {}

    // 设置“密度自适应 A*”参数，便于从 rosparam 注入并在线调参。
    void setAdaptiveParams(int density_radius,
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
                           float alpha_narrow);

    bool calculatePotentials(unsigned char* costs,
                             double start_x, double start_y,
                             double end_x,   double end_y,
                             int cycles, float* potential) override;

  private:
    void add(unsigned char* costs,
             float* potential,
             float prev_potential,
             int current_i,
             int next_i,
             int end_x,
             int end_y);

    // 与 AStarExpansion 共用 Index / greater1 定义
    std::vector<Index> queue_;

    // 局部障碍物密度窗口与计数权重
    int density_radius_;
    float inflation_lambda_;
    float threshold_open_;
    float threshold_narrow_;

    // 三模式权重（宽阔 / 常规 / 狭窄）
    float w_obs_open_;
    float w_obs_normal_;
    float w_obs_narrow_;
    float w_dir_open_;
    float w_dir_normal_;
    float w_dir_narrow_;
    float alpha_open_;
    float alpha_normal_;
    float alpha_narrow_;
};

}  // namespace improved_global_planner

#endif  // _IMPROVED_ASTAR_H

