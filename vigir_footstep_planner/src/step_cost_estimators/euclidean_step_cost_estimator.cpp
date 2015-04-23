#include <vigir_footstep_planner/step_cost_estimators/euclidean_step_cost_estimator.h>

namespace vigir_footstep_planning
{
EuclideanStepCostEstimator::EuclideanStepCostEstimator(const ParameterSet& params)
  : StepCostEstimatorPlugin("euclidean_step_cost_estimator", params)
{
}

EuclideanStepCostEstimator::EuclideanStepCostEstimator()
  : StepCostEstimatorPlugin("euclidean_step_cost_estimator")
{
}

bool EuclideanStepCostEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& risk) const
{
  cost = 0.0;
  risk = 0.0;

  if (swing_foot == left_foot || swing_foot == right_foot)
    return true;

  const State& swing_foot_before = swing_foot.getLeg() == LEFT ? left_foot : right_foot;
  cost = 0.5*euclidean_distance(swing_foot_before.getX(), swing_foot_before.getY(), swing_foot.getX(), swing_foot.getY());

  return true;
}
}
