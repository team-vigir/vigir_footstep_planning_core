#include <vigir_footstep_planner/heuristics/occupancy_grid_map_heuristic.h>

namespace vigir_footstep_planning
{
OccupancyGridMapHeuristic::OccupancyGridMapHeuristic(const ParameterSet& params, ros::NodeHandle& nh, const std::string& topic)
: HeuristicPlugin("occupancy_grid_map_heuristic", params)
{
  occupancy_grid_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(topic, 1, &OccupancyGridMapHeuristic::mapCallback, this);
}

OccupancyGridMapHeuristic::OccupancyGridMapHeuristic(ros::NodeHandle& nh, const std::string& topic)
: HeuristicPlugin("occupancy_grid_map_heuristic")
{
  occupancy_grid_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(topic, 1, &OccupancyGridMapHeuristic::mapCallback, this);
}

void OccupancyGridMapHeuristic::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map)
{
  boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex);
  distance_map.setMap(occupancy_grid_map);
}

double OccupancyGridMapHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  boost::shared_lock<boost::shared_mutex> lock(grid_map_shared_mutex);

  double d = distance_map.distanceMapAt(from.getX(), from.getY());
  if (d < 0.0)
    return 0.0;

  if (d < 0.01)
    return max_heuristic_value;

  return 5.0*std::max(0.0, 0.5-d);
}
}
