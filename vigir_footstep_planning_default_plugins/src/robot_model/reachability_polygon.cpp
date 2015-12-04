#include <vigir_footstep_planning_default_plugins/robot_model/reachability_polygon.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
ReachabilityPolygon::ReachabilityPolygon()
  : ReachabilityPlugin("reachability_polygon")
  , ivpStepRange(nullptr)
{
}

ReachabilityPolygon::~ReachabilityPolygon()
{
  if (ivpStepRange)
    delete[] ivpStepRange;
}

void ReachabilityPolygon::loadParams(const ParameterSet& params)
{
  int num_angle_bins;
  params.getParam("collision_check/cell_size", cell_size);
  params.getParam("collision_check/num_angle_bins", num_angle_bins);
  angle_bin_size = 2.0*M_PI / static_cast<double>(num_angle_bins);

  // step range
  std::vector<std::pair<int, int> > step_range;
  XmlRpc::XmlRpcValue step_range_x;
  XmlRpc::XmlRpcValue step_range_y;

  if (!params.getParam("step_range/x", step_range_x) || !params.getParam("step_range/y", step_range_y))
  {
    ROS_ERROR("Can't initialize ReachabilityPolygon due to missing step_range in parameter set!");
    return;
  }

  assert((step_range_x.size() == step_range_y.size()) && (step_range_x.size() > 0));

  // load step range
  step_range.clear();
  step_range.reserve(step_range_x.size());

  for (int i = 0; i < step_range_x.size(); i++)
  {
    double x = static_cast<double>(step_range_x[i]);
    double y = static_cast<double>(step_range_y[i]);

    step_range.push_back(std::pair<int, int>(disc_val(x, cell_size), disc_val(y, cell_size)));
  }

  // get step range limits
  params.getParam("max_step_range/x", max_step_range_x);
  params.getParam("max_step_range/y", max_step_range_y);
  params.getParam("max_step_range/yaw", max_step_range_yaw);

  params.getParam("max_step_range/inv_x", max_step_range_inv_x);
  params.getParam("max_step_range/inv_y", max_step_range_inv_y);
  params.getParam("max_step_range/inv_yaw", max_step_range_inv_yaw);

  params.getParam("max_step_range/width", max_step_range_width_sq);
  max_step_range_width_sq = max_step_range_width_sq*max_step_range_width_sq;

  int max_step_range_x_disc = disc_val(max_step_range_x, cell_size);
  int max_step_range_y_disc = disc_val(max_step_range_y, cell_size);

  int max_step_range_inv_x_disc = disc_val(max_step_range_inv_x, cell_size);
  int max_step_range_inv_y_disc = disc_val(max_step_range_inv_y, cell_size);

  step_range_size_x = max_step_range_x_disc - max_step_range_inv_x_disc + 1;
  step_range_size_y = max_step_range_y_disc - max_step_range_inv_y_disc + 1;
  step_range_size = step_range_size_x * step_range_size_y;

  if (ivpStepRange)
    delete[] ivpStepRange;
  ivpStepRange = new bool[step_range_size];

  // compute reachability polygon
  ROS_INFO("Reachability Polygon:");
  std::string msg;
  for (int y = max_step_range_inv_y_disc; y <= max_step_range_y_disc; y++)
  {
    msg += boost::lexical_cast<std::string>(y) + ": ";
    for (int x = max_step_range_inv_x_disc; x <= max_step_range_x_disc; x++)
    {
      ivpStepRange[(y - max_step_range_inv_y_disc) * step_range_size_x + (x - max_step_range_inv_x_disc)] = pointWithinPolygon(x, y, step_range);
      msg += pointWithinPolygon(x, y, step_range) ? "+ " : "- ";
    }
    ROS_INFO("%s", msg.c_str());
    msg.clear();
  }
}

bool ReachabilityPolygon::isReachable(const State& current, const State& next) const
{
  if (current.getLeg() == next.getLeg())
    return false;

  if (euclidean_distance_sq(current.getX(), current.getY(), next.getX(), next.getY()) > max_step_range_width_sq)
    return false;

  // reconstruct step primitive
  tf::Transform step = current.getPose().inverse() * next.getPose();
  double dx = step.getOrigin().x();
  double dy = step.getOrigin().y();
  double dyaw = angles::shortest_angular_distance(current.getYaw(), next.getYaw());

  // adjust for the left foot
  if (current.getLeg() == LEFT)
  {
    dy = -dy;
    dyaw = -dyaw;
  }

  // consider discretization error
  dx = pround(dx, cell_size);
  dy = pround(dy, cell_size);
  dyaw = pround(dyaw, angle_bin_size);

  // check if footstep_x is not within the executable range
  if (dx > max_step_range_x || dx < max_step_range_inv_x)
    return false;
  // check if footstep_y is not within the executable range
  if (dy > max_step_range_y || dy < max_step_range_inv_y)
    return false;
  // check if stance_yaw_diff is not within the executable range
  if (dyaw > max_step_range_yaw || dyaw < max_step_range_inv_yaw)
    return false;

  int footstep_x = disc_val(dx-max_step_range_inv_x, cell_size);
  int footstep_y = disc_val(dy-max_step_range_inv_y, cell_size);

  assert((footstep_x + footstep_y * step_range_size_x) < step_range_size);

  // check reachability polygon
  if (!ivpStepRange[footstep_x + footstep_y * step_range_size_x])
    return false;

  return true;
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::ReachabilityPolygon, vigir_footstep_planning::ReachabilityPlugin)
