#include <vigir_footstep_planner/heuristics/hot_map_heuristic.h>

namespace vigir_footstep_planning
{
HotMapHeuristic::HotMapHeuristic(const ParameterSet& params, ros::NodeHandle& nh)
: HeuristicPlugin("hot_map_heuristic", params)
{
  // publish topics
  hot_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("hot_map", 1);
  publish_timer = nh.createTimer(ros::Duration(1.0), &HotMapHeuristic::publishHotMap, this);
}

HotMapHeuristic::HotMapHeuristic(ros::NodeHandle& nh)
: HeuristicPlugin("hot_map_heuristic")
{
  // publish topics
  hot_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("hot_map", 1);
  publish_timer = nh.createTimer(ros::Duration(1.0), &HotMapHeuristic::publishHotMap, this);
}

void HotMapHeuristic::reset()
{
  boost::unique_lock<boost::shared_mutex> lock(hot_map_shared_mutex);
  hot_map.clear();
  total_hit_counter = 0;
}

void HotMapHeuristic::loadParams(const ParameterSet& params)
{
  HeuristicPlugin::loadParams(params);

  params.getParam("collision_check/cell_size", cell_size);
  int num_angle_bins;
  params.getParam("collision_check/num_angle_bins", num_angle_bins);
  angle_bin_size = 2.0*M_PI / static_cast<double>(num_angle_bins);
}

double HotMapHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  unsigned int hit = 0;
  {
    boost::unique_lock<boost::shared_mutex> lock(hot_map_shared_mutex);
    hit = hot_map[StateKey(from, cell_size, angle_bin_size)]++;
    total_hit_counter++;
  }

  return 1000.0 * static_cast<double>(hit)/static_cast<double>(total_hit_counter);
}

void HotMapHeuristic::publishHotMap(const ros::TimerEvent& /*publish_timer*/) const
{
  if (hot_map_pub.getNumSubscribers() > 0)
  {
    boost::shared_lock<boost::shared_mutex> lock(hot_map_shared_mutex);

    nav_msgs::OccupancyGrid grid_map;

    grid_map.header.stamp = ros::Time::now();
    grid_map.header.frame_id = "/world";

    /// TODO: Implement grid map conversion

    hot_map_pub.publish(grid_map);
  }
}
}
