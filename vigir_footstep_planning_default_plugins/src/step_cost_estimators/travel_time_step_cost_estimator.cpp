#include <vigir_footstep_planning_default_plugins/step_cost_estimators/travel_time_step_cost_estimator.h>



namespace vigir_footstep_planning
{
TravelTimeStepCostEstimator::TravelTimeStepCostEstimator()
  : StepCostEstimatorPlugin("travel_time_cost_estimator")
{
}

bool TravelTimeStepCostEstimator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!StepCostEstimatorPlugin::loadParams(params))
    return false;

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

  return true;
}

bool TravelTimeStepCostEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  const State& stand_foot = swing_foot.getLeg() == LEFT ? right_foot : left_foot;
  const State& swing_foot_before = swing_foot.getLeg() == LEFT ? left_foot : right_foot;

  tf::Transform sway = swing_foot_before.getPose().inverse() * stand_foot.getPose();
  tf::Transform swing = swing_foot_before.getPose().inverse() * swing_foot.getPose();

  double sway_duration = parabol(sway.getOrigin().x(), sway.getOrigin().y(), a_sway_inv, b_sway_inv) + const_sway_time;
  double step_duration = parabol(swing.getOrigin().x(), swing.getOrigin().y(), a_swing_inv, b_swing_inv) + const_swing_time;

  cost = sway_duration + step_duration;

  return true;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::TravelTimeStepCostEstimator, vigir_footstep_planning::StepCostEstimatorPlugin)
