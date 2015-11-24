#include <vigir_footstep_planner/heuristics/euclidean_heuristic.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
EuclideanHeuristic::EuclideanHeuristic()
: HeuristicPlugin("euclidean_heuristic")
{}

double EuclideanHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  return euclidean_distance(from.getX(), from.getY(), from.getZ(), to.getX(), to.getY(), to.getZ());
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::EuclideanHeuristic, vigir_footstep_planning::HeuristicPlugin)
