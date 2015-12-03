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
#include <vigir_footstep_planner/step_cost_estimators/travel_time_step_cost_estimator.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
TravelTimeStepCostEstimator::TravelTimeStepCostEstimator()
  : StepCostEstimatorPlugin("travel_time_cost_estimator")
{
}

void TravelTimeStepCostEstimator::loadParams(const vigir_generic_params::ParameterSet& params)
{
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

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::TravelTimeStepCostEstimator, vigir_footstep_planning::StepCostEstimatorPlugin)
