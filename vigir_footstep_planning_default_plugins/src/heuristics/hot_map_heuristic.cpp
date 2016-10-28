#include <vigir_footstep_planning_default_plugins/heuristics/hot_map_heuristic.h>



namespace vigir_footstep_planning
{
HotMapHeuristic::HotMapHeuristic()
: HeuristicPlugin("hot_map_heuristic")
{
}

bool HotMapHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::loadParams(params))
    return false;

  params.getParam("collision_check/cell_size", cell_size_);
  int num_angle_bins;
  params.getParam("collision_check/num_angle_bins", num_angle_bins);
  angle_bin_size_ = 2.0*M_PI / static_cast<double>(num_angle_bins);
  return true;
}

bool HotMapHeuristic::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::initialize(params))
    return false;

  // publish topics
  hot_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("hot_map", 1);
  publish_timer_ = nh_.createTimer(ros::Duration(1.0), &HotMapHeuristic::publishHotMap, this);

  return true;
}

void HotMapHeuristic::reset()
{
  boost::unique_lock<boost::shared_mutex> lock(hot_map_shared_mutex_);
  hot_map_.clear();
  total_hit_counter_ = 0;
}

double HotMapHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  unsigned int hit = 0;
  {
    boost::unique_lock<boost::shared_mutex> lock(hot_map_shared_mutex_);
    hit = hot_map_[StateKey(from, cell_size_, angle_bin_size_)]++;
    total_hit_counter_++;
  }

  return 1000.0 * static_cast<double>(hit)/static_cast<double>(total_hit_counter_);
}

void HotMapHeuristic::publishHotMap(const ros::TimerEvent& /*publish_timer*/) const
{
  if (hot_map_pub_.getNumSubscribers() > 0)
  {
    boost::shared_lock<boost::shared_mutex> lock(hot_map_shared_mutex_);

    nav_msgs::OccupancyGrid grid_map;

    grid_map.header.stamp = ros::Time::now();
    grid_map.header.frame_id = "/world";

    /// TODO: Implement grid map conversion

    hot_map_pub_.publish(grid_map);
  }
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::HotMapHeuristic, vigir_footstep_planning::HeuristicPlugin)
