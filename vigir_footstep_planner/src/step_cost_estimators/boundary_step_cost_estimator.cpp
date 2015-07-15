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
#include <vigir_footstep_planner/step_cost_estimators/boundary_step_cost_estimator.h>

namespace vigir_footstep_planning
{
BoundaryStepCostEstimator::BoundaryStepCostEstimator(const ParameterSet& params)
  : StepCostEstimatorPlugin("boundary_step_cost_estimator", params)
{
}

BoundaryStepCostEstimator::BoundaryStepCostEstimator()
  : StepCostEstimatorPlugin("boundary_step_cost_estimator")
{
}

void BoundaryStepCostEstimator::loadParams(const ParameterSet& params)
{
  params.getParam("boundary_step_cost_estimator/max_diff_z", max_diff_z);
  params.getParam("boundary_step_cost_estimator/long_step_dist", long_step_dist);
  params.getParam("boundary_step_cost_estimator/min_yaw_seperation_enlargement", min_yaw_seperation_enlargement);
  params.getParam("boundary_step_cost_estimator/yaw_enlarged_min_seperation", yaw_enlarged_min_seperation);
  params.getParam("boundary_step_cost_estimator/cost_roll_abs", cost_roll_abs);
  params.getParam("boundary_step_cost_estimator/cost_pitch_abs", cost_pitch_abs);
  params.getParam("boundary_step_cost_estimator/cost_yaw_rel", cost_yaw_rel);
  params.getParam("boundary_step_cost_estimator/cost_height_diff_rel", cost_height_diff_rel);
}

bool BoundaryStepCostEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = 0.0;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  if (swing_foot == left_foot || swing_foot == right_foot)
    return true;

  const State& stand_foot = swing_foot.getLeg() == LEFT ? right_foot : left_foot;
  const State& swing_foot_before = swing_foot.getLeg() == LEFT ? left_foot : right_foot;

  double diff_z = std::abs(swing_foot.getZ()-stand_foot.getZ());

  // add costs
  if (diff_z >= max_diff_z)
    return false;

  // determine additional costs
  tf::Transform step = stand_foot.getPose().inverse() * swing_foot.getPose();

  // all long steps should be more expensive
  if (step.getOrigin().x() > long_step_dist)
    risk += step.getOrigin().x()-long_step_dist;

  // get yaw diffs
  double stance_yaw_diff = angles::shortest_angular_distance(swing_foot_before.getYaw(), stand_foot.getYaw()); /// TODO: expensive!
  double swing_foot_yaw_diff = angles::shortest_angular_distance(stand_foot.getYaw(), swing_foot.getYaw()); /// TODO: expensive!

  // if foot is turned step more outside
  if (std::abs(swing_foot_yaw_diff) >= min_yaw_seperation_enlargement && std::abs(step.getOrigin().y()) <= yaw_enlarged_min_seperation)
    return false;

  // determine costs changing yaw: increasing turn rate may be expensiv but decreasing is free
  double turn_rate_diff = swing_foot_yaw_diff - stance_yaw_diff;

  if (turn_rate_diff * swing_foot_yaw_diff <= 0.0) // check for different sign
    turn_rate_diff = 0.0; // decreasing turn rate towards zero is always free

  // determine risk
  risk += cost_roll_abs * std::abs(swing_foot.getRoll());
  risk += cost_pitch_abs * std::abs(swing_foot.getPitch());
  risk += cost_yaw_rel * std::abs(turn_rate_diff);
  risk += cost_height_diff_rel * diff_z;

  cost = risk*risk;

  return true;
}
}
