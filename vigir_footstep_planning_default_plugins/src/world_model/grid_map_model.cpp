#include <vigir_footstep_planning_default_plugins/world_model/grid_map_model.h>



namespace vigir_footstep_planning
{
GridMapModel::GridMapModel(const std::string& name)
  : CollisionCheckGridMapPlugin(name)
{
}

bool GridMapModel::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!CollisionCheckGridMapPlugin::loadParams(params))
    return false;

  params.getParam("collision_check/collision_check_accuracy", (int&)collision_check_accuracy);
  return true;
}

void GridMapModel::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map)
{
  CollisionCheckGridMapPlugin::mapCallback(occupancy_grid_map);

  boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex_);
  distance_map.setMap(occupancy_grid_map);
}

bool GridMapModel::collision_check(double x, double y, double cos_theta, double sin_theta, double height, double width) const
{
  double d = distance_map.distanceMapAt(x, y);

  bool out_of_bounds = d < 0.0; // if out of bounds => assume no collision

  if (out_of_bounds && collision_check_accuracy != 2)
    return false;

  d = std::max(d-distance_map.getResolution(), 0.0);

  double r_o = width*width + height*height;

  if (!out_of_bounds)
  {
    if (d*d >= 0.25 * r_o)
      return false;
    else if (collision_check_accuracy == 0)
      return false;
  }

  double h_half = 0.5 * height;
  double w_half = 0.5 * width;
  double r_i = std::min(w_half, h_half);

  if (!out_of_bounds)
  {
    if (d <= r_i)
      return true;
    else if (collision_check_accuracy == 1)
      return true;
  }
  else if (r_i < distance_map.getResolution())
  {
    return false;
  }

  double h_new;
  double w_new;
  double delta_x;
  double delta_y;
  if (width < height)
  {
    double h_clear = out_of_bounds ? 0.0 : sqrt(d*d - w_half*w_half);
    h_new = h_half - h_clear;
    w_new = width;
    delta_x = h_clear + 0.5 * h_new;
    delta_y = 0.0;
  }
  else // footWidth >= footHeight
  {
    double w_clear = out_of_bounds ? 0.0 : sqrt(d*d - h_half*h_half);
    h_new = height;
    w_new = w_half - w_clear;
    delta_x = 0.0;
    delta_y = w_clear + 0.5 * w_new;
  }
  double x_shift = cos_theta*delta_x - sin_theta*delta_y;
  double y_shift = sin_theta*delta_x + cos_theta*delta_y;

  return (collision_check(x+x_shift, y+y_shift, cos_theta, sin_theta, h_new, w_new) ||
          collision_check(x-x_shift, y-y_shift, cos_theta, sin_theta, h_new, w_new));
}
}
