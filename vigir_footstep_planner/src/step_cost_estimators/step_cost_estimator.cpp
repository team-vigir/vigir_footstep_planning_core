#include <vigir_footstep_planner/step_cost_estimators/step_cost_estimator.h>

namespace vigir_footstep_planning
{
StepCostEstimator::Ptr StepCostEstimator::singelton = StepCostEstimator::Ptr();

StepCostEstimator::StepCostEstimator()
{
}

StepCostEstimator::Ptr& StepCostEstimator::Instance()
{
  if (!singelton)
    singelton.reset(new StepCostEstimator());
  return singelton;
}

void StepCostEstimator::loadPlugins()
{
  // get step cost estimator plugins
  PluginManager::getPluginsByType(Instance()->step_cost_estimators);

  if (Instance()->step_cost_estimators.empty())
    ROS_ERROR("[StepCostEstimator] loadPlugins: Couldn't find any step cost estimator. Fix it immediatly!");
  else
  {
    ROS_INFO("[StepCostEstimator] Plugins loaded:");
    for (std::vector<StepCostEstimatorPlugin::Ptr>::const_iterator itr = Instance()->step_cost_estimators.begin(); itr != Instance()->step_cost_estimators.end(); itr++)
    {
      const StepCostEstimatorPlugin::Ptr& step_cost_estimator = *itr;
      if (step_cost_estimator)
        ROS_INFO("    %s", step_cost_estimator->getName().c_str());
    }
  }
}

void StepCostEstimator::loadParams(const ParameterSet& params)
{
  for (std::vector<StepCostEstimatorPlugin::Ptr>::iterator itr = Instance()->step_cost_estimators.begin(); itr != Instance()->step_cost_estimators.end(); itr++)
  {
    StepCostEstimatorPlugin::Ptr& step_cost_estimator = *itr;
    if (step_cost_estimator)
      step_cost_estimator->loadParams(params);
  }
}

bool StepCostEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& risk)
{
  cost = 0.0;
  risk = 0.0;

  for (std::vector<StepCostEstimatorPlugin::Ptr>::const_iterator itr = Instance()->step_cost_estimators.begin(); itr != Instance()->step_cost_estimators.end(); itr++)
  {
    const StepCostEstimatorPlugin::Ptr& step_cost_estimator = *itr;
    double c, r;
    if (step_cost_estimator && !step_cost_estimator->getCost(left_foot, right_foot, swing_foot, c, r))
      return false;
    else
    {
      cost += c;
      risk += r;
    }
  }

  return true;
}

bool StepCostEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, float& cost, float& risk)
{
  double cost_d, risk_d;
  bool result = getCost(left_foot, right_foot, swing_foot, cost_d, risk_d);
  cost = static_cast<float>(cost_d);
  risk = static_cast<float>(risk_d);
  return result;
}
}
