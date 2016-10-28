#include <vigir_footstep_planning_default_plugins/step_cost_estimators/ground_contact_step_cost_estimator.h>

#include <angles/angles.h>



namespace vigir_footstep_planning
{
GroundContactStepCostEstimator::GroundContactStepCostEstimator()
  : StepCostEstimatorPlugin("ground_contact_step_cost_estimator")
{
}

bool GroundContactStepCostEstimator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!StepCostEstimatorPlugin::loadParams(params))
    return false;

  params.getParam("foot_contact_support/minimal_support", min_contact_support);
  return true;
}

bool GroundContactStepCostEstimator::getCost(const State& /*left_foot*/, const State& /*right_foot*/, const State& swing_foot, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = 0.0;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  if (swing_foot.getGroundContactSupport() < 1.0)
  {
    if (swing_foot.getGroundContactSupport() > min_contact_support)
      cost_multiplier = risk_multiplier = 1.0/swing_foot.getGroundContactSupport();
    else
      return false;
  }

  return true;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::GroundContactStepCostEstimator, vigir_footstep_planning::StepCostEstimatorPlugin)
