/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
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
  vigir_pluginlib::PluginManager::getPluginsByType(Instance()->step_cost_estimators);

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

void StepCostEstimator::loadParams(const vigir_generic_params::ParameterSet& params)
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
  double cost_multiplier = 1.0;
  risk = 0.0;
  double risk_multiplier = 1.0;

  //ROS_INFO("-----------------------------------");

  for (std::vector<StepCostEstimatorPlugin::Ptr>::const_iterator itr = Instance()->step_cost_estimators.begin(); itr != Instance()->step_cost_estimators.end(); itr++)
  {
    const StepCostEstimatorPlugin::Ptr& step_cost_estimator = *itr;
    if (!step_cost_estimator)
      continue;

    double c, c_m, r, r_m;
    if (step_cost_estimator->getCost(left_foot, right_foot, swing_foot, c, c_m, r, r_m))
    {
      cost += c;
      cost_multiplier *= c_m;
      risk += r;
      risk_multiplier *= r_m;
      //ROS_INFO("[%s]: %.3f %.3f %.3f %.3f", step_cost_estimator->getName().c_str(), c, r, c_m, r_m);
    }
    else
      return false;
  }

  cost *= cost_multiplier;
  risk *= risk_multiplier;

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
