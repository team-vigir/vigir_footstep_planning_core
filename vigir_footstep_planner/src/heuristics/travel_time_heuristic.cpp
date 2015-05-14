#include <vigir_footstep_planner/heuristics/travel_time_heuristic.h>

namespace vigir_footstep_planning
{
TravelTimeHeuristic::TravelTimeHeuristic(const ParameterSet& params)
  : HeuristicPlugin("travel_time_heuristic", params)
{
}

TravelTimeHeuristic::TravelTimeHeuristic()
  : HeuristicPlugin("travel_time_heuristic")
{
}

void TravelTimeHeuristic::loadParams(const ParameterSet& params)
{
  //params.getParam("const_step_cost_estimator/step_cost", step_cost, 0.1);
}

double TravelTimeHeuristic::getHeuristicValue(const State& from, const State& to) const
{
  if (from == to)
    return 0.0;

  return 0.0;
}
}
