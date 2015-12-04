#include <vigir_footstep_planning_default_plugins/heuristics/occupancy_grid_map_heuristic.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
OccupancyGridMapHeuristic::OccupancyGridMapHeuristic()
: HeuristicPlugin("occupancy_grid_map_heuristic")
{
}

void OccupancyGridMapHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  HeuristicPlugin::loadParams(params);

  params.getParam("occupancy_grid_map_heuristic/grid_map_topic", grid_map_topic);
}

bool OccupancyGridMapHeuristic::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::initialize(nh, params))
    return false;

  // subscribe topics
  occupancy_grid_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(grid_map_topic, 1, &OccupancyGridMapHeuristic::mapCallback, this);

  return true;
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

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::OccupancyGridMapHeuristic, vigir_footstep_planning::HeuristicPlugin)
