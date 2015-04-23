#include <vigir_footstep_planner/step_cost_estimators/const_step_cost_estimator.h>

namespace vigir_footstep_planning
{
ConstStepCostEstimator::ConstStepCostEstimator(const ParameterSet& params)
  : StepCostEstimatorPlugin("const_step_cost_estimator", params)
{
}

ConstStepCostEstimator::ConstStepCostEstimator()
  : StepCostEstimatorPlugin("const_step_cost_estimator")
{
}

void ConstStepCostEstimator::loadParams(const ParameterSet& params)
{
  params.getParam("const_step_cost_estimator/step_cost", const_step_cost, 0.1);
}

bool ConstStepCostEstimator::getCost(const State& /*left_foot*/, const State& /*right_foot*/, const State& /*swing_foot*/, double& cost, double& risk) const
{
  cost = const_step_cost;
  risk = 0.0;
  return true;
}
}
