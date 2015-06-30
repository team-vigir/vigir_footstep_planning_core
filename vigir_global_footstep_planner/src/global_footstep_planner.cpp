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
