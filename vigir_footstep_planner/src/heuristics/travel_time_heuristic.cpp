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
  HeuristicPlugin::loadParams(params);

  params.getParam("max_step_dist/x", max_step_dist_x_inv);
  max_step_dist_x_inv = 1.0/max_step_dist_x_inv;
  params.getParam("max_step_dist/y", max_step_dist_y_inv);
  max_step_dist_y_inv = 1.0/max_step_dist_y_inv;

  params.getParam("travel_time_cost_estimator/sway/parabol_a", a_sway_inv, 0.0);
  a_sway_inv = 1.0/a_sway_inv;
  params.getParam("travel_time_cost_estimator/sway/parabol_b", b_sway_inv, 0.0);
  b_sway_inv = 1.0/b_sway_inv;
  params.getParam("travel_time_cost_estimator/sway/const_time", const_sway_time, 0.0);

  params.getParam("travel_time_cost_estimator/swing/parabol_a", a_swing_inv, 0.0);
  a_swing_inv = 1.0/a_swing_inv;
  params.getParam("travel_time_cost_estimator/swing/parabol_b", b_swing_inv, 0.0);
  b_swing_inv = 1.0/b_swing_inv;
  params.getParam("travel_time_cost_estimator/swing/const_time", const_swing_time, 0.0);
}

double TravelTimeHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  // expected steps
  tf::Transform step = from.getPose().inverse() * to.getPose();
  double expected_steps_x = std::abs(step.getOrigin().x()) * max_step_dist_x_inv;
  double expected_steps_y = std::abs(step.getOrigin().y()) * max_step_dist_y_inv;
  double expected_steps = std::max(expected_steps_x, expected_steps_y);


  //double sway_duration = parabol(sway.getOrigin().x(), sway.getOrigin().y(), a_sway_inv, b_sway_inv) + const_sway_time;
  double step_duration = parabol(step.getOrigin().x()/expected_steps, step.getOrigin().y()/expected_steps, a_swing_inv, b_swing_inv) + const_swing_time;

  //ROS_INFO_THROTTLE(1.0, "%f %f, ETA: %f", expected_steps_x, expected_steps_y, expected_steps * min_duration_per_step);

  //double expected_steps = euclidean_distance(from.getX(), from.getY(), from.getZ(), to.getX(), to.getY(), to.getZ()) * max_step_dist_x_inv;

  return expected_steps * (step_duration + const_sway_time);
}
}
