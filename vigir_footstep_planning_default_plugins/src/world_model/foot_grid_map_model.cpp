#include <vigir_footstep_planning_default_plugins/world_model/foot_grid_map_model.h>



namespace vigir_footstep_planning
{
FootGridMapModel::FootGridMapModel(const std::string& name)
  : GridMapModel(name)
{
}

bool FootGridMapModel::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapModel::initialize(params))
    return false;

  // get foot dimensions
  getFootSize(nh_, foot_size);

  return true;
}

bool FootGridMapModel::isAccessible(const State& s) const
{
  boost::shared_lock<boost::shared_mutex> lock(grid_map_shared_mutex_);

  if (!occupancy_grid_map_)
  {
    ROS_ERROR_THROTTLE(10, "[FootGridMapModel] No ground level grid map available yet.");
    return true;
  }

  double x = s.getX();
  double y = s.getY();

  double theta = s.getYaw();

  // collision check for the foot center
  return !collision_check(x, y, cos(theta), sin(theta), foot_size.x, foot_size.y);
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::FootGridMapModel, vigir_footstep_planning::CollisionCheckPlugin)
