#include <vigir_footstep_planner/world_model/foot_grid_map_model.h>

namespace vigir_footstep_planning
{
FootGridMapModel::FootGridMapModel(const std::string& name, const vigir_generic_params::ParameterSet& params, ros::NodeHandle& nh, const std::string& topic)
  : GridMapModel(name, params, FOOT, nh, topic)
{
  // get foot dimensions
  getFootSize(nh, foot_size);
}

FootGridMapModel::FootGridMapModel(const std::string& name, ros::NodeHandle& nh, const std::string& topic)
  : GridMapModel(name, FOOT, nh, topic)
{
  // get foot dimensions
  getFootSize(nh, foot_size);
}

bool FootGridMapModel::isAccessible(const State& s) const
{
  boost::shared_lock<boost::shared_mutex> lock(grid_map_shared_mutex);

  if (!occupancy_grid_map)
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
