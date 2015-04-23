#include <vigir_footstep_planner/heuristics/euclidean_heuristic.h>

namespace vigir_footstep_planning
{
EuclideanHeuristic::EuclideanHeuristic(const ParameterSet& params)
: HeuristicPlugin("euclidean_heuristic", params)
{}

EuclideanHeuristic::EuclideanHeuristic()
: HeuristicPlugin("euclidean_heuristic")
{}

double EuclideanHeuristic::getHeuristicValue(const State& from, const State& to) const
{
  if (from == to)
    return 0.0;

  // distance in cell size
  return euclidean_distance(from.getX(), from.getY(), from.getZ(), to.getX(), to.getY(), to.getZ());
}
}
