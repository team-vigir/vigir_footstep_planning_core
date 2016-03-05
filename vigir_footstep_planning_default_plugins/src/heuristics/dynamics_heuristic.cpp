#include <vigir_footstep_planning_default_plugins/heuristics/dynamics_heuristic.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
DynamicsHeuristic::DynamicsHeuristic()
  : HeuristicPlugin("dynamics_heuristic")
{
}

void DynamicsHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  HeuristicPlugin::loadParams(params);

  params.getParam("dynamics/body/max_acc", max_body_acc_, 0.0);
}

double DynamicsHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  if (max_body_acc_ <= 0.0)
    return 0.0;

  // check if we can still retard in time
  const geometry_msgs::Vector3& v = from.getBodyVelocity();
  double d_min = (v.x*v.x + v.y*v.y + v.z*v.z)/(2.0*max_body_acc_);

  // if planner has issues finding a solution, increase the scaling of d_min or decrease cell size
  if (d_min*1.2 > euclidean_distance(from.getX(), from.getY(), from.getZ(), to.getX(), to.getY(), to.getZ()))
    return max_heuristic_value_;

  return 0.0;
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::DynamicsHeuristic, vigir_footstep_planning::HeuristicPlugin)
