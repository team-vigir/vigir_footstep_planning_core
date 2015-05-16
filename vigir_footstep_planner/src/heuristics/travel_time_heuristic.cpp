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
  double min_sway_duration, min_step_duration;
  params.getParam("travel_time_heuristic/min_sway_duration", min_sway_duration, 0.0);
  params.getParam("travel_time_heuristic/min_step_duration", min_step_duration, 0.1);
  min_duration_per_step = min_sway_duration + min_step_duration;

  params.getParam("max_step_dist/x", max_step_dist_x_inv);
  max_step_dist_x_inv = 1.0/max_step_dist_x_inv;
  params.getParam("max_step_dist/y", max_step_dist_y_inv);
  max_step_dist_y_inv = 1.0/max_step_dist_y_inv;
}

double TravelTimeHeuristic::getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const
{
  if (from == to)
    return 0.0;

  // expected steps
  tf::Transform step = from.getPose().inverse() * to.getPose();
  double expected_steps_x = std::abs(step.getOrigin().x()) * max_step_dist_x_inv;
  double expected_steps_y = std::abs(step.getOrigin().y()) * max_step_dist_x_inv;
  double expected_steps = expected_steps_x + expected_steps_y;

  //ROS_INFO_THROTTLE(1.0, "%f %f, ETA: %f", expected_steps_x, expected_steps_y, expected_steps * min_duration_per_step);

  //double expected_steps = euclidean_distance(from.getX(), from.getY(), from.getZ(), to.getX(), to.getY(), to.getZ()) * max_step_dist_x_inv;

  return expected_steps * min_duration_per_step;
}
}
