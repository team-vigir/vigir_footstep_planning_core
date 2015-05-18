#include <vigir_footstep_planner/heuristics/path_cost_heuristic.h>

namespace vigir_footstep_planning
{
PathCostHeuristic::PathCostHeuristic(const ParameterSet& params)
  : HeuristicPlugin("path_cost_heuristic", params)
  , ivpGrid(NULL)
  , ivGoalX(-1)
  , ivGoalY(-1)
{
}

PathCostHeuristic::PathCostHeuristic()
  : HeuristicPlugin("path_cost_heuristic")
{
}

PathCostHeuristic::~PathCostHeuristic()
{
  if (ivpGrid)
    resetGrid();
}

void PathCostHeuristic::loadParams(const ParameterSet& params)
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
