/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
#include <vigir_footstep_planner/heuristics/path_cost_heuristic.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
PathCostHeuristic::PathCostHeuristic()
  : HeuristicPlugin("path_cost_heuristic")
  , ivpGrid(NULL)
  , ivGoalX(-1)
  , ivGoalY(-1)
{
}

PathCostHeuristic::~PathCostHeuristic()
{
  if (ivpGrid)
    resetGrid();
}

void PathCostHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  HeuristicPlugin::loadParams(params);

  params.getParam("collision_check/cell_size", ivCellSize);
  params.getParam("collision_check/num_angle_bins", ivNumAngleBins);
  ivAngleBinSize = 2.0*M_PI / static_cast<double>(ivNumAngleBins);

  params.getParam("const_step_cost_estimator/step_cost", ivStepCost, 0.1);
  params.getParam("diff_angle_cost", ivDiffAngleCost);
  params.getParam("max_step_dist/x", ivMaxStepWidth);
  /// TODO
  /// ivInflationRadius
}

double PathCostHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  assert(ivGoalX >= 0 && ivGoalY >= 0);

  if (from == to)
    return 0.0;

  unsigned int from_x;
  unsigned int from_y;
  // could be removed after more testing (then use ...noBounds... again)
  ivMapPtr->worldToMapNoBounds(cell_2_state(from.getX(), ivCellSize),
                               cell_2_state(from.getY(), ivCellSize),
                               from_x, from_y);

  unsigned int to_x;
  unsigned int to_y;
  // could be removed after more testing (then use ...noBounds... again)
  ivMapPtr->worldToMapNoBounds(cell_2_state(to.getX(), ivCellSize),
                               cell_2_state(to.getY(), ivCellSize),
                               to_x, to_y);

  // cast to unsigned int is safe since ivGoalX/ivGoalY are checked to be >= 0
  if ((unsigned int)ivGoalX != to_x || (unsigned int)ivGoalY != to_y)
  {
    ROS_ERROR("PathCostHeuristic::getHValue to a different value than "
              "precomputed, heuristic values will be wrong. You need to call "
              "calculateDistances() before!");
  }
  assert((unsigned int)ivGoalX == to_x && (unsigned int)ivGoalY == to_y);

  double dist = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(from_x, from_y)) / 1000.0;

  double expected_steps = dist / ivMaxStepWidth;
  double diff_angle = 0.0;
  if (ivDiffAngleCost > 0.0)
    diff_angle = std::abs(angles::shortest_angular_distance(to.getYaw(), from.getYaw()));

  return dist + expected_steps * ivStepCost + diff_angle * ivDiffAngleCost;
}

bool PathCostHeuristic::calculateDistances(const State& from, const State& to)
{
  assert(ivMapPtr);

  unsigned int from_x;
  unsigned int from_y;
  ivMapPtr->worldToMapNoBounds(cell_2_state(from.getX(), ivCellSize),
                               cell_2_state(from.getY(), ivCellSize),
                               from_x, from_y);

  unsigned int to_x;
  unsigned int to_y;
  ivMapPtr->worldToMapNoBounds(cell_2_state(to.getX(), ivCellSize),
                               cell_2_state(to.getY(), ivCellSize),
                               to_x, to_y);

  if ((int)to_x != ivGoalX || (int)to_y != ivGoalY)
  {
    ivGoalX = to_x;
    ivGoalY = to_y;
    ivGridSearchPtr->search(ivpGrid, cvObstacleThreshold,
                            ivGoalX, ivGoalY, from_x, from_y,
                            SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
  }

  return true;
}

void PathCostHeuristic::updateMap(vigir_gridmap_2d::GridMap2DPtr map)
{
  ivMapPtr.reset();
  ivMapPtr = map;

  ivGoalX = ivGoalY = -1;

  unsigned width = ivMapPtr->getInfo().width;
  unsigned height = ivMapPtr->getInfo().height;

  if (ivGridSearchPtr)
    ivGridSearchPtr->destroy();
  ivGridSearchPtr.reset(new SBPL2DGridSearch(width, height,
                                             ivMapPtr->getResolution()));
  if (ivpGrid)
    resetGrid();
  ivpGrid = new unsigned char* [width];

  for (unsigned x = 0; x < width; ++x)
    ivpGrid[x] = new unsigned char [height];
  for (unsigned y = 0; y < height; ++y)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      float dist = ivMapPtr->distanceMapAtCell(x,y);
      if (dist < 0.0f)
        ROS_ERROR("Distance map at %d %d out of bounds", x, y);
      else if (dist <= ivInflationRadius)
        ivpGrid[x][y] = 255;
      else
        ivpGrid[x][y] = 0;
    }
  }
}

void PathCostHeuristic::resetGrid()
{
  CvSize size = ivMapPtr->size();
  for (int x = 0; x < size.width; ++x)
  {
    if (ivpGrid[x])
    {
      delete[] ivpGrid[x];
      ivpGrid[x] = NULL;
    }
  }
  delete[] ivpGrid;
  ivpGrid = NULL;
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::PathCostHeuristic, vigir_footstep_planning::HeuristicPlugin)
