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
#include <vigir_pattern_generator/pattern_generator.h>

namespace vigir_footstep_planning
{
PatternGenerator::PatternGenerator(ros::NodeHandle& nh)
{
  joystick_handler.reset(new JoystickHandler(nh));

  nh.param("world_frame_id", world_frame_id, std::string("/world"));
  nh.param("pattern_generator/number_of_steps", (int&)number_of_steps_needed, 5);

  ros::service::waitForService("step_plan_request");

  // start service clients: TODO use global footstep planner
  generate_feet_pose_client = nh.serviceClient<msgs::GenerateFeetPoseService>("generate_feet_pose");
  step_plan_request_client = nh.serviceClient<vigir_footstep_planning_msgs::StepPlanRequestService>("step_plan_request");

  // initialize action clients
  execute_step_plan_ac.reset(new actionlib::SimpleActionClient<msgs::ExecuteStepPlanAction>("execute_step_plan", true));

  reset();
}

PatternGenerator::~PatternGenerator()
{}

msgs::ErrorStatus PatternGenerator::generatePattern(const msgs::StepPlanRequest& step_plan_request, msgs::StepPlan& step_plan)
{
  // generate step plan request
  msgs::StepPlanRequestService step_plan_request_srv;
  step_plan_request_srv.request.plan_request = step_plan_request;

  // send request
  if (!step_plan_request_client.call(step_plan_request_srv.request, step_plan_request_srv.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "[PatternGenerator]", "Can't call footstep planner!");

  // return new step plan
  if (step_plan_request_srv.response.status.error == msgs::ErrorStatus::NO_ERROR)
    step_plan = step_plan_request_srv.response.step_plan;

  return step_plan_request_srv.response.status;
}

void PatternGenerator::reset()
{
  joy_d_step = geometry_msgs::Pose();
  joy_d_step.orientation = tf::createQuaternionMsgFromYaw(0.0);

  last_performed_step_index = 0;
  first_changeable_step_index = 0;
  next_step_index_needed = 0;

  start_feet_pose.reset();
  foot_start_step_index_left = 0;
  foot_start_step_index_right = 0;
  has_new_steps = false;

  clearStepPlan();

  params.enable = false;
}

void PatternGenerator::setParams(const msgs::PatternGeneratorParameters& params)
{
  if (isEnabled() && params.enable)
  {
    // trigger replanning
    bool result = true;
    if (isSimulationMode())
      result = setNextStartStepIndex(last_performed_step_index+1);
    else
      result = setNextStartStepIndex(first_changeable_step_index-1);

    if (!result)
    {
      ROS_ERROR("[PatternGenerator] setParams: Replanning failed due to unknown start step.");
      reset();
    }
  }
  else
  {
    // triggers reset when enabling
    setEnabled(params.enable);
  }

  this->params = params;
}

void PatternGenerator::setEnabled(bool enable)
{
  // activation
  if (!isEnabled() && enable)
  {
    reset();
  }
  // deactivation
  else if (isEnabled() && !enable)
  {
    bool result = true;
    if (isSimulationMode())
      result = setNextStartStepIndex(last_performed_step_index+1);
    else
      result = setNextStartStepIndex(first_changeable_step_index-1);

    if (result)
      generateSteps(0, true);
  }

  params.enable = enable;
}

bool PatternGenerator::isEnabled() const
{
  return params.enable;
}

bool PatternGenerator::isSimulationMode() const
{
  return params.simulation_mode;
}

bool PatternGenerator::hasSteps() const
{
  return !newest_step_plan.steps.empty();
}

bool PatternGenerator::hasNewSteps() const
{
  return !newest_step_plan.steps.empty() && has_new_steps;
}

void PatternGenerator::getCompleteStepPlan(msgs::StepPlan& step_plan) const
{
  step_plan = complete_step_plan;
}

void PatternGenerator::getNewestStepPlan(msgs::StepPlan& step_plan) const
{
  has_new_steps = false;
  step_plan = newest_step_plan;
}

bool PatternGenerator::setNextStartStepIndex(int step_index)
{
  if (step_index < 0)
    return false;
  else if (!updateFeetStartPoseByStepMap(step_map, step_index-1) || !updateFeetStartPoseByStepMap(step_map, step_index))
    return false;

  return true;
}

int PatternGenerator::getNextStartStepIndex() const
{
  return std::max(foot_start_step_index_left, foot_start_step_index_right);
}

void PatternGenerator::clearStepPlan()
{
  complete_step_plan = msgs::StepPlan();
  newest_step_plan = msgs::StepPlan();
  step_map.clear();
}

void PatternGenerator::update(const ros::TimerEvent& timer)
{
  // handle joystick input
  if (joystick_handler && params.joystick_mode)
  {
    double yaw = tf::getYaw(joy_d_step.orientation);

    bool enable;
    joystick_handler->updateJoystickCommands((timer.current_real - timer.last_real).toSec(), enable, joy_d_step.position.x, joy_d_step.position.y, yaw);
    joy_d_step.orientation = tf::createQuaternionMsgFromYaw(yaw);

    setEnabled(enable);
  }

  // if not enabled, just do nothing
  if (!isEnabled())
    return;

  // check if more steps are needed
  int queued_steps = getNextStartStepIndex() - last_performed_step_index;
  if (queued_steps < number_of_steps_needed)
    generateSteps(number_of_steps_needed-queued_steps);

  if (isSimulationMode())
    last_performed_step_index++;
}

void PatternGenerator::updateLastPerformedStepIndex(int last_performed_step_index)
{
  if (!isSimulationMode())
  {
    if (last_performed_step_index < 0)
    {
      ROS_ERROR("[PatternGenerator] updateLastPerformedStepIndex: Stopping due to invalid step indx %i", last_performed_step_index);
      reset();
    }

    this->last_performed_step_index = last_performed_step_index;
  }
}

void PatternGenerator::updateFirstChangeableStepIndex(int first_changeable_step_index)
{
  if (!isSimulationMode())
  {
    if (first_changeable_step_index < 0)
    {
      ROS_ERROR("[PatternGenerator] updateFirstChangeableStepIndex: Stopping due to invalid step indx %i", first_changeable_step_index);
      setEnabled(false);
    }

    this->first_changeable_step_index = first_changeable_step_index;
  }
}

void PatternGenerator::updateFeetStartPose(uint8_t foot_index, const geometry_msgs::Pose& pose)
{
  msgs::Foot foot;
  foot.header.frame_id = world_frame_id;
  foot.header.stamp = ros::Time::now();
  foot.foot_index = foot_index;
  foot.pose = pose;
  updateFeetStartPose(foot);
}

void PatternGenerator::updateFeetStartPose(const msgs::Foot& foot)
{
  if (!start_feet_pose)
  {
    start_feet_pose.reset(new msgs::Feet());
    start_feet_pose->header = foot.header;
    start_feet_pose->left.header = foot.header;
    start_feet_pose->left.foot_index = msgs::Foot::LEFT;
    start_feet_pose->right.header = foot.header;
    start_feet_pose->right.foot_index = msgs::Foot::RIGHT;
  }

  if (foot.foot_index == msgs::Foot::LEFT)
  {
    start_feet_pose->left.header = foot.header;
    start_feet_pose->left = foot;
  }
  if (foot.foot_index == msgs::Foot::RIGHT)
  {
    start_feet_pose->right.header = foot.header;
    start_feet_pose->right = foot;
  }
}

void PatternGenerator::updateFeetStartPose(const msgs::Feet& feet)
{
  updateFeetStartPose(feet.left);
  updateFeetStartPose(feet.right);

  start_feet_pose->header = feet.header;
}

void PatternGenerator::updateFeetStartPose(const msgs::Step& step)
{
  updateFeetStartPose(step.foot);

  start_feet_pose->header = step.header;
  if (step.foot.foot_index == msgs::Foot::LEFT)
    foot_start_step_index_left = step.step_index;
  if (step.foot.foot_index == msgs::Foot::RIGHT)
    foot_start_step_index_right = step.step_index;
}

bool PatternGenerator::updateFeetStartPoseByStepMap(const std::map<unsigned int, msgs::Step>& map, unsigned int step_index)
{
  std::map<unsigned int, msgs::Step>::const_iterator itr = map.find(step_index);
  if (itr != map.end())
  {
    updateFeetStartPose(itr->second);
    return true;
  }
  return false;
}

void PatternGenerator::updateFootstepMap(std::map<unsigned int, msgs::Step>& map, const std::vector<msgs::Step>& vec) const
{
  std::vector<msgs::Step>::const_iterator itr = vec.begin();

  if (!map.empty())
   itr++;

  for (; itr != vec.end(); itr++)
    map[itr->step_index] = *itr;
}

void PatternGenerator::mapToVectorIndexed(const std::map<unsigned int, msgs::Step>& map, std::vector<msgs::Step>& vec, unsigned int start_index, unsigned int end_index) const
{
  double foot_z = 0.0;

  vec.clear();
  for (unsigned int step_index = start_index; step_index <= end_index; step_index++)
  {
    std::map<unsigned int, msgs::Step>::const_iterator itr = map.find(step_index);

    // break if index is missing
    if (itr == map.end())
      break;

    msgs::Step step = itr->second;

    /// TODO: Workaround for avoiding z drift
    if (step_index == start_index)
      foot_z = step.foot.pose.position.z;
    else
      step.foot.pose.position.z = foot_z;
    vec.push_back(step);
  }
}

void PatternGenerator::generateSteps(unsigned int n, bool close_step)
{
  msgs::StepPlanRequestService step_plan_request_srv;
  msgs::StepPlanRequest& req = step_plan_request_srv.request.plan_request;

  if (!start_feet_pose)
  {
    std_msgs::Header header;
    header.frame_id = world_frame_id;
    header.stamp = ros::Time::now();
    msgs::Feet start_feet;
    determineStartFeetPose(start_feet, generate_feet_pose_client, header);
    updateFeetStartPose(start_feet);
  }

  // check command input
  geometry_msgs::Pose d_step = params.d_step;
  if (params.joystick_mode)
    d_step = joy_d_step;

  tf::Quaternion q_tf;
  tf::quaternionMsgToTF(d_step.orientation, q_tf);

  double roll, pitch, yaw;
  tf::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);

  // determine which foot has to move first
  unsigned int next_moving_foot_index = msgs::Foot::LEFT;
  if (getNextStartStepIndex() == 0)
  {
    if (d_step.position.y < 0.0)
      next_moving_foot_index = msgs::Foot::LEFT;
    else if (d_step.position.y == 0.0 && yaw < 0.0)
      next_moving_foot_index = msgs::Foot::RIGHT;
  }
  else
  {
    if (foot_start_step_index_left > foot_start_step_index_right)
      next_moving_foot_index = msgs::Foot::RIGHT;
    else
      next_moving_foot_index = msgs::Foot::LEFT;
  }

  // generate request message
  req.header = start_feet_pose->header;
  req.start = *start_feet_pose;
  switch (next_moving_foot_index)
  {
    case msgs::Foot::LEFT:
      req.start_step_index = foot_start_step_index_right; // reminder: first moving foot has start_index+1
      req.start_foot_selection = msgs::StepPlanRequest::LEFT;
      break;
    case msgs::Foot::RIGHT:
      req.start_step_index = foot_start_step_index_left; // reminder: first moving foot has start_index+1
      req.start_foot_selection = msgs::StepPlanRequest::RIGHT;
      break;
    default:
      ROS_ERROR("[PatternGenerator] Unknown foot index '%u'", next_moving_foot_index);
      return;
  }

  // special case for n = 0
  if (n == 0)
  {
    if (next_moving_foot_index == msgs::Foot::LEFT)
      req.pattern_parameters.mode = msgs::PatternParameters::FEET_REALIGN_ON_RIGHT;
    else
      req.pattern_parameters.mode = msgs::PatternParameters::FEET_REALIGN_ON_LEFT;
  }
  else
  {
    req.pattern_parameters.steps = n;
    req.pattern_parameters.step_distance_forward = d_step.position.x;
    req.pattern_parameters.step_distance_sideward = d_step.position.y;
    req.pattern_parameters.turn_angle = yaw;
    req.pattern_parameters.close_step = close_step;
    req.pattern_parameters.extra_seperation = false;
    req.pattern_parameters.override = false; // disable here, it will override too much by now
    req.pattern_parameters.roll = roll;
    req.pattern_parameters.pitch = pitch;
    req.pattern_parameters.mode = msgs::PatternParameters::SAMPLING;
  }

  req.planning_mode = msgs::StepPlanRequest::PLANNING_MODE_PATTERN;
  req.parameter_set_name = params.parameter_set_name;

  // send request
  if (!step_plan_request_client.call(step_plan_request_srv.request, step_plan_request_srv.response))
  {
    ROS_ERROR("[PatternGenerator] Can't call footstep planner!");
    return;
  }

  // handle new step plan
  if (!step_plan_request_srv.response.step_plan.steps.size())
    return;

  updateFootstepMap(step_map, step_plan_request_srv.response.step_plan.steps);

  if (complete_step_plan.steps.empty()) // received step plan first time
    complete_step_plan = step_plan_request_srv.response.step_plan;
  else // just update steps
    mapToVectorIndexed(step_map, complete_step_plan.steps, 0, step_plan_request_srv.response.step_plan.steps.back().step_index);

  newest_step_plan = step_plan_request_srv.response.step_plan;
  newest_step_plan.header.stamp = ros::Time::now();
  newest_step_plan.header.seq++;
  has_new_steps = true;

  if (!setNextStartStepIndex(newest_step_plan.steps.back().step_index))
  {
    ROS_ERROR("[PatternGenerator] generateSteps: Last step index of recent pattern was wrong. Resetting now!");
    reset();
  }
}
}
