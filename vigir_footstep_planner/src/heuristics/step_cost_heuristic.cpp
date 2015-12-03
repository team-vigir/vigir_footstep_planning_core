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
#include <vigir_footstep_planner/heuristics/step_cost_heuristic.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
StepCostHeuristic::StepCostHeuristic()
  : HeuristicPlugin("step_cost_heuristic")
{
}

void StepCostHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  HeuristicPlugin::loadParams(params);

  params.getParam("const_step_cost_estimator/step_cost", step_cost, 0.1);
  params.getParam("diff_angle_cost", diff_angle_cost);
  params.getParam("max_step_dist/x", max_step_dist_x_inv);
  max_step_dist_x_inv = 1.0/max_step_dist_x_inv;
  params.getParam("max_step_dist/y", max_step_dist_y_inv);
  max_step_dist_y_inv = 1.0/max_step_dist_y_inv;
}

double StepCostHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  // expected steps
  tf::Transform dstep;
  getDeltaStep(from.getPose(), to.getPose(), dstep);
  double expected_steps_x = std::abs(dstep.getOrigin().x()) * max_step_dist_x_inv;
  double expected_steps_y = std::abs(dstep.getOrigin().y()) * max_step_dist_y_inv;
  double expected_steps = std::ceil(std::max(expected_steps_x, expected_steps_y));

  double diff_angle = 0.0;
  if (diff_angle_cost > 0.0)
    diff_angle = std::abs(angles::shortest_angular_distance(to.getYaw(), from.getYaw()));

//  ROS_WARN("-------------------------------");
//  ROS_INFO("x: %f %f %f", step.getOrigin().x(), ivMaxStepDistX, std::abs(step.getOrigin().x()) / ivMaxStepDistX);
//  ROS_INFO("y: %f %f %f", step.getOrigin().y(), ivMaxStepDistY, std::abs(step.getOrigin().y()) / ivMaxStepDistY);
//  ROS_INFO("steps: %f, dist: %f, cost: %f", expected_steps, dist, (dist + expected_steps * ivStepCost + diff_angle * ivDiffAngleCost));

  return expected_steps * step_cost + diff_angle * diff_angle_cost;
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::StepCostHeuristic, vigir_footstep_planning::HeuristicPlugin)
