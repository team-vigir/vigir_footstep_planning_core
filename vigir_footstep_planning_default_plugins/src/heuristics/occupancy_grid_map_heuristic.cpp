#include <vigir_footstep_planning_default_plugins/heuristics/occupancy_grid_map_heuristic.h>



namespace vigir_footstep_planning
{
OccupancyGridMapHeuristic::OccupancyGridMapHeuristic()
: HeuristicPlugin("occupancy_grid_map_heuristic")
{
}

bool OccupancyGridMapHeuristic::initialize(const vigir_generic_params::ParameterSet& global_params)
{
  if (!HeuristicPlugin::initialize(global_params))
    return false;

  // subscribe topics
  occupancy_grid_map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(grid_map_topic_, 1, &OccupancyGridMapHeuristic::mapCallback, this);

  return true;
}

bool OccupancyGridMapHeuristic::loadParams(const vigir_generic_params::ParameterSet& global_params)
{
  if (!HeuristicPlugin::loadParams(global_params))
    return false;

  getParam("grid_map_topic", grid_map_topic_, std::string());
  return true;
}

void OccupancyGridMapHeuristic::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map)
{
  boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex_);
  distance_map_.setMap(occupancy_grid_map);
}

double OccupancyGridMapHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  boost::shared_lock<boost::shared_mutex> lock(grid_map_shared_mutex_);

  double d = distance_map_.distanceMapAt(from.getX(), from.getY());
  if (d < 0.0)
    return 0.0;

  if (d < 0.01)
    return max_heuristic_value_;

  return 5.0*std::max(0.0, 0.5-d);
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::OccupancyGridMapHeuristic, vigir_footstep_planning::HeuristicPlugin)
