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

    bool calculatePotentials(unsigned char* costs,
                             double start_x, double start_y,
                             double end_x,   double end_y,
                             int cycles, float* potential) override;

  private:
    void add(unsigned char* costs,
             float* potential,
             float prev_potential,
             int next_i,
             int end_x,
             int end_y);

    // 与 AStarExpansion 共用 Index / greater1 定义
    std::vector<Index> queue_;
};

}  // namespace improved_global_planner

#endif  // _IMPROVED_ASTAR_H

