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
#include <vigir_global_footstep_planner/global_footstep_planner.h>

namespace vigir_footstep_planning
{
GlobalFootstepPlanner::GlobalFootstepPlanner(FootstepPlanner::Ptr& footstep_planner)
  : footstep_planner(footstep_planner)
  , step_plan(new StepPlan())
{
}

GlobalFootstepPlanner::GlobalFootstepPlanner()
  : step_plan(new StepPlan())
{
}

GlobalFootstepPlanner::~GlobalFootstepPlanner()
{
}

void GlobalFootstepPlanner::init(ros::NodeHandle& nh)
{
  if (!footstep_planner)
    footstep_planner.reset(new FootstepPlanner(nh));

  // start service clients
  generate_feet_pose_client = nh.serviceClient<msgs::GenerateFeetPoseService>("generate_feet_pose");
  parameter_client = nh.serviceClient<msgs::SetParameterSetService>("set_planner_parameters");
}

msgs::ErrorStatus GlobalFootstepPlanner::setStepPlan(const msgs::StepPlan& step_plan)
{
  return this->step_plan->fromMsg(step_plan);
}

msgs::ErrorStatus GlobalFootstepPlanner::getStepPlan(msgs::StepPlan& step_plan) const
{
  return this->step_plan->toMsg(step_plan);
}

msgs::ErrorStatus GlobalFootstepPlanner::appendStepPlan(const std::vector<msgs::StepPlan>& step_plans, msgs::StepPlan& result) const
{
  msgs::ErrorStatus status;
  StepPlan temp_step_plan;

  // append all plans
  for (std::vector<msgs::StepPlan>::const_iterator itr = step_plans.begin(); itr != step_plans.end() && status.error == msgs::ErrorStatus::NO_ERROR; itr++)
    status += temp_step_plan.appendStepPlan(*itr);

  status += temp_step_plan.toMsg(result);
  return status;
}

msgs::ErrorStatus GlobalFootstepPlanner::stitchStepPlan(const std::vector<msgs::StepPlan>& step_plans, msgs::StepPlan &result) const
{
  msgs::ErrorStatus status;
  StepPlan temp_step_plan;

  // stitch all plans
  for (std::vector<msgs::StepPlan>::const_iterator itr = step_plans.begin(); itr != step_plans.end() && status.error == msgs::ErrorStatus::NO_ERROR; itr++)
    status += temp_step_plan.stitchStepPlan(*itr);

  status += temp_step_plan.toMsg(result);
  return status;
}

msgs::ErrorStatus GlobalFootstepPlanner::editStep(const msgs::EditStep& edit_step, const msgs::StepPlan& step_plan, std::vector<msgs::StepPlan>& result) const
{
  result.clear();

  msgs::ErrorStatus status;
  StepPlan temp_step_plan(step_plan);
  msgs::StepPlan temp_step_plan_msg;

  // dispatch edit mode
  msgs::UpdateMode update_mode;
  switch (edit_step.plan_mode)
  {
    case msgs::EditStep::EDIT_MODE_REMOVE:
      // remove step if found
      if (!temp_step_plan.hasStep(edit_step.step.step_index))
        status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "GlobalFootstepPlanner", "editStep: Step with index " + boost::lexical_cast<std::string>(edit_step.step.step_index) + " was not found!");
      else
        temp_step_plan.removeStep(edit_step.step.step_index);
      break;
    case msgs::EditStep::EDIT_MODE_2D:
      update_mode.mode = msgs::UpdateMode::UPDATE_MODE_Z;
      break;
    case msgs::EditStep::EDIT_MODE_3D:
      update_mode.mode = msgs::UpdateMode::UPDATE_MODE_3D;
      break;
    case msgs::EditStep::EDIT_MODE_FULL:
      break;
    default:
      status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "GlobalFootstepPlanner", "editStep: Invalid plan mode: " + boost::lexical_cast<std::string>(edit_step.plan_mode));
      return status;
  }

  // update step
  if (edit_step.plan_mode != msgs::EditStep::EDIT_MODE_REMOVE)
  {
    // update foot
    msgs::Step step = edit_step.step;
    if (update_mode.mode)
      status += footstep_planner->updateFoot(step.foot, update_mode.mode);

    // update plan
    status += temp_step_plan.updateStep(step);
    status += temp_step_plan.toMsg(temp_step_plan_msg);

    // check validity of entire plan
    footstep_planner->updateStepPlan(temp_step_plan_msg, msgs::UpdateMode::UPDATE_MODE_CHECK_VALIDITY);
  }
  else
    status += temp_step_plan.toMsg(temp_step_plan_msg);

  // split plan if needed
  if (temp_step_plan_msg.steps.size() && status.warning == msgs::ErrorStatus::WARN_INCOMPLETE_STEP_PLAN)
  {
    msgs::StepPlan sub_step_plan;
    sub_step_plan.header = temp_step_plan_msg.header;
    sub_step_plan.data = temp_step_plan_msg.data;

    // search split points
    unsigned int next_index = temp_step_plan_msg.steps.front().step_index;
    for (unsigned int i = 0; i < temp_step_plan_msg.steps.size(); i++)
    {
      msgs::Step& step = temp_step_plan_msg.steps[i];

      if (step.step_index != next_index++ || i == temp_step_plan_msg.steps.size()-1)
      {
        result.push_back(sub_step_plan);
        sub_step_plan.steps.clear();
        next_index = step.step_index+1;
      }

      sub_step_plan.steps.push_back(step);
    }
  }
  else
    result.push_back(temp_step_plan_msg);

  return status;
}
}
