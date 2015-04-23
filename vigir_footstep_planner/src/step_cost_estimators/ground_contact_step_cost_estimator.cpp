#include <vigir_footstep_planner/step_cost_estimators/ground_contact_step_cost_estimator.h>

namespace vigir_footstep_planning
{
GroundContactStepCostEstimator::GroundContactStepCostEstimator(const ParameterSet& params)
  : StepCostEstimatorPlugin("ground_contact_step_cost_estimator", params)
{
}

GroundContactStepCostEstimator::GroundContactStepCostEstimator()
  : StepCostEstimatorPlugin("ground_contact_step_cost_estimator")
{
}

void GroundContactStepCostEstimator::loadParams(const ParameterSet& params)
{
  params.getParam("foot_contact_support/minimal_support", min_contact_support);
}

bool GroundContactStepCostEstimator::getCost(const State& /*left_foot*/, const State& /*right_foot*/, const State& swing_foot, double& cost, double& risk) const
{
  cost = 0.0;
  risk = 0.0;
  double scaling = 1.0;

  if (swing_foot.getGroundContactSupport() < 1.0)
  {
    if (swing_foot.getGroundContactSupport() > min_contact_support)
      scaling = 1.0/swing_foot.getGroundContactSupport();
    else
      return false;
  }

  /// TODO
  //return StepCostEstimator::getCost(left_foot, right_foot, swing_foot, risk_cost) * scaling;
  return true;
}
}
