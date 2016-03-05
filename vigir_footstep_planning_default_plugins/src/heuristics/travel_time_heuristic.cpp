#include <vigir_footstep_planning_default_plugins/heuristics/travel_time_heuristic.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
TravelTimeHeuristic::TravelTimeHeuristic()
  : HeuristicPlugin("travel_time_heuristic")
{
}

void TravelTimeHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  HeuristicPlugin::loadParams(params);

  params.getParam("max_step_dist/x", max_step_dist_x_inv_);
  max_step_dist_x_inv_ = 1.0/max_step_dist_x_inv_;
  params.getParam("max_step_dist/y", max_step_dist_y_inv_);
  max_step_dist_y_inv_ = 1.0/max_step_dist_y_inv_;

  params.getParam("travel_time_cost_estimator/sway/parabol_a", a_sway_inv_, 0.0);
  a_sway_inv_ = 1.0/a_sway_inv_;
  params.getParam("travel_time_cost_estimator/sway/parabol_b", b_sway_inv_, 0.0);
  b_sway_inv_ = 1.0/b_sway_inv_;
  params.getParam("travel_time_cost_estimator/sway/const_time", const_sway_time_, 0.0);

  params.getParam("travel_time_cost_estimator/swing/parabol_a", a_swing_inv_, 0.0);
  a_swing_inv_ = 1.0/a_swing_inv_;
  params.getParam("travel_time_cost_estimator/swing/parabol_b", b_swing_inv_, 0.0);
  b_swing_inv_ = 1.0/b_swing_inv_;
  params.getParam("travel_time_cost_estimator/swing/const_time", const_swing_time_, 0.0);
}

double TravelTimeHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  // expected steps
  tf::Transform step = from.getPose().inverse() * to.getPose();
  double expected_steps_x = std::abs(step.getOrigin().x()) * max_step_dist_x_inv_;
  double expected_steps_y = std::abs(step.getOrigin().y()) * max_step_dist_y_inv_;
  double expected_steps = std::max(expected_steps_x, expected_steps_y);


  //double sway_duration = parabol(sway.getOrigin().x(), sway.getOrigin().y(), a_sway_inv, b_sway_inv) + const_sway_time;
  double step_duration = parabol(step.getOrigin().x()/expected_steps, step.getOrigin().y()/expected_steps, a_swing_inv_, b_swing_inv_) + const_swing_time_;

  //ROS_INFO_THROTTLE(1.0, "%f %f, ETA: %f", expected_steps_x, expected_steps_y, expected_steps * min_duration_per_step);

  //double expected_steps = euclidean_distance(from.getX(), from.getY(), from.getZ(), to.getX(), to.getY(), to.getZ()) * max_step_dist_x_inv;

  return expected_steps * (step_duration + const_sway_time_);
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::TravelTimeHeuristic, vigir_footstep_planning::HeuristicPlugin)
