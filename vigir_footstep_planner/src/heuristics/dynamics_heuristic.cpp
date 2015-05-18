#include <vigir_footstep_planner/heuristics/dynamics_heuristic.h>

namespace vigir_footstep_planning
{
DynamicsHeuristic::DynamicsHeuristic(const ParameterSet& params)
  : HeuristicPlugin("dynamics_heuristic", params)
{
}

DynamicsHeuristic::DynamicsHeuristic()
  : HeuristicPlugin("dynamics_heuristic")
{
}

void DynamicsHeuristic::loadParams(const ParameterSet& params)
{
  HeuristicPlugin::loadParams(params);

  params.getParam("dynamics/body/max_acc", max_body_acc, 0.0);
}

double DynamicsHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  if (max_body_acc <= 0.0)
    return 0.0;

  // check if we can still retard in time
  const geometry_msgs::Vector3& v = from.getBodyVelocity();
  double d_min = (v.x*v.x + v.y*v.y + v.z*v.z)/(2.0*max_body_acc);

  // if planner has issues finding a solution, increase the scaling of d_min or decrease cell size
  if (d_min*1.2 > euclidean_distance(from.getX(), from.getY(), from.getZ(), to.getX(), to.getY(), to.getZ()))
    return max_heuristic_value;

  return 0.0;
}
}
