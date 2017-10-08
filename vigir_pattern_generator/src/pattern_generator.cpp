#include <vigir_pattern_generator/pattern_generator.h>

namespace vigir_footstep_planning
{
PatternGenerator::PatternGenerator(ros::NodeHandle& nh)
{
  joystick_handler_.reset(new JoystickHandler(nh));

  nh.param("world_frame_id", world_frame_id_, std::string("/world"));
  nh.param("pattern_generator/number_of_steps", (int&)number_of_steps_needed_, 5);

  //ros::service::waitForService("step_plan_request");

  // start service clients: TODO use global footstep planner
  generate_feet_pose_client_ = nh.serviceClient<msgs::GenerateFeetPoseService>("generate_feet_pose");
  step_plan_request_client_ = nh.serviceClient<vigir_footstep_planning_msgs::StepPlanRequestService>("step_plan_request");

  // initialize action clients
  execute_step_plan_ac_.reset(new actionlib::SimpleActionClient<msgs::ExecuteStepPlanAction>("execute_step_plan", true));

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
  if (!step_plan_request_client_.call(step_plan_request_srv.request, step_plan_request_srv.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "[PatternGenerator]", "Can't call footstep planner!");

  // return new step plan
  if (step_plan_request_srv.response.status.error == msgs::ErrorStatus::NO_ERROR)
    step_plan = step_plan_request_srv.response.step_plan;

  return step_plan_request_srv.response.status;
}

void PatternGenerator::reset()
{
  joystick_cmd_ = geometry_msgs::Twist();

  last_performed_step_index_ = 0;
  first_changeable_step_index_ = 0;
  next_step_index_needed_ = 0;

  start_feet_pose_.reset();
  foot_start_step_index_left_ = 0;
  foot_start_step_index_right_ = 0;
  has_new_steps_ = false;

  clearStepPlan();

  params_.enable = false;
}

void PatternGenerator::setParams(const msgs::PatternGeneratorParameters& params)
{
  if (isEnabled() && params.enable)
  {
    // trigger replanning
    bool result = true;
    if (isSimulationMode())
      result = setNextStartStepIndex(last_performed_step_index_+1);
    else
      result = setNextStartStepIndex(first_changeable_step_index_-1);

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

  this->params_ = params;
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
      result = setNextStartStepIndex(last_performed_step_index_+1);
    else
      result = setNextStartStepIndex(first_changeable_step_index_-1);

    if (result)
      generateSteps(0, true);
  }

  params_.enable = enable;
}

bool PatternGenerator::isEnabled() const
{
  return params_.enable;
}

bool PatternGenerator::isSimulationMode() const
{
  return params_.simulation_mode;
}

bool PatternGenerator::hasSteps() const
{
  return !newest_step_plan_.steps.empty();
}

bool PatternGenerator::hasNewSteps() const
{
  return !newest_step_plan_.steps.empty() && has_new_steps_;
}

void PatternGenerator::getCompleteStepPlan(msgs::StepPlan& step_plan) const
{
  step_plan = complete_step_plan_;
}

void PatternGenerator::getNewestStepPlan(msgs::StepPlan& step_plan) const
{
  has_new_steps_ = false;
  step_plan = newest_step_plan_;
}

bool PatternGenerator::setNextStartStepIndex(int step_index)
{
  if (step_index < 0)
    return false;
  else if (!updateFeetStartPoseByStepMap(step_map_, step_index-1) || !updateFeetStartPoseByStepMap(step_map_, step_index))
    return false;

  return true;
}

int PatternGenerator::getNextStartStepIndex() const
{
  return std::max(foot_start_step_index_left_, foot_start_step_index_right_);
}

void PatternGenerator::clearStepPlan()
{
  complete_step_plan_ = msgs::StepPlan();
  newest_step_plan_ = msgs::StepPlan();
  step_map_.clear();
}

void PatternGenerator::update(const ros::TimerEvent& timer)
{
  // handle joystick input
  if (joystick_handler_ /*&& params_.joystick_mode*/)
  {
    bool enable;
    joystick_handler_->getJoystickCommand((timer.current_real - timer.last_real).toSec(), enable, joystick_cmd_);

    setEnabled(enable);
  }

  // if not enabled, just do nothing
  if (!isEnabled())
    return;

  // check if more steps are needed
  int queued_steps = getNextStartStepIndex() - last_performed_step_index_;
  if (queued_steps < static_cast<int>(number_of_steps_needed_))
    generateSteps(number_of_steps_needed_-queued_steps);

  if (isSimulationMode())
    last_performed_step_index_++;
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

    this->last_performed_step_index_ = last_performed_step_index;
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

    this->first_changeable_step_index_ = first_changeable_step_index;
  }
}

void PatternGenerator::updateFeetStartPose(uint8_t foot_index, const geometry_msgs::Pose& pose)
{
  msgs::Foot foot;
  foot.header.frame_id = world_frame_id_;
  foot.header.stamp = ros::Time::now();
  foot.foot_index = foot_index;
  foot.pose = pose;
  updateFeetStartPose(foot);
}

void PatternGenerator::updateFeetStartPose(const msgs::Foot& foot)
{
  if (!start_feet_pose_)
  {
    start_feet_pose_.reset(new msgs::Feet());
    start_feet_pose_->header = foot.header;
    start_feet_pose_->left.header = foot.header;
    start_feet_pose_->left.foot_index = msgs::Foot::LEFT;
    start_feet_pose_->right.header = foot.header;
    start_feet_pose_->right.foot_index = msgs::Foot::RIGHT;
  }

  if (foot.foot_index == msgs::Foot::LEFT)
  {
    start_feet_pose_->left.header = foot.header;
    start_feet_pose_->left = foot;
  }
  if (foot.foot_index == msgs::Foot::RIGHT)
  {
    start_feet_pose_->right.header = foot.header;
    start_feet_pose_->right = foot;
  }
}

void PatternGenerator::updateFeetStartPose(const msgs::Feet& feet)
{
  updateFeetStartPose(feet.left);
  updateFeetStartPose(feet.right);

  start_feet_pose_->header = feet.header;
}

void PatternGenerator::updateFeetStartPose(const msgs::Step& step)
{
  updateFeetStartPose(step.foot);

  start_feet_pose_->header = step.header;
  if (step.foot.foot_index == msgs::Foot::LEFT)
    foot_start_step_index_left_ = step.step_index;
  if (step.foot.foot_index == msgs::Foot::RIGHT)
    foot_start_step_index_right_ = step.step_index;
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

  if (!start_feet_pose_)
  {
    std_msgs::Header header;
    header.frame_id = world_frame_id_;
    header.stamp = ros::Time::now();
    msgs::Feet start_feet;
    determineStartFeetPose(start_feet, generate_feet_pose_client_, header);
    updateFeetStartPose(start_feet);
  }

  // check command input
  const geometry_msgs::Twist& cmd = params_.joystick_mode ? joystick_cmd_ : params_.cmd;

  // determine which foot has to move first
  unsigned int next_moving_foot_index = msgs::Foot::LEFT;
  if (getNextStartStepIndex() == 0)
  {
    if (cmd.linear.y < 0.0)
      next_moving_foot_index = msgs::Foot::LEFT;
    else if (cmd.linear.y == 0.0 && cmd.angular.z < 0.0)
      next_moving_foot_index = msgs::Foot::RIGHT;
  }
  else
  {
    if (foot_start_step_index_left_ > foot_start_step_index_right_)
      next_moving_foot_index = msgs::Foot::RIGHT;
    else
      next_moving_foot_index = msgs::Foot::LEFT;
  }

  // generate request message
  req.header = start_feet_pose_->header;
  req.start = *start_feet_pose_;
  switch (next_moving_foot_index)
  {
    case msgs::Foot::LEFT:
      req.start_step_index = foot_start_step_index_right_; // reminder: first moving foot has start_index+1
      req.start_foot_selection = msgs::StepPlanRequest::LEFT;
      break;
    case msgs::Foot::RIGHT:
      req.start_step_index = foot_start_step_index_left_; // reminder: first moving foot has start_index+1
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
    req.pattern_parameters.step_distance_forward = cmd.linear.x;
    req.pattern_parameters.step_distance_sideward = cmd.linear.y;
    req.pattern_parameters.turn_angle = cmd.angular.z;
    req.pattern_parameters.close_step = close_step;
    req.pattern_parameters.extra_seperation = false;
    req.pattern_parameters.override = false; // disable here, it will override too much by now
    req.pattern_parameters.roll = 0.0;
    req.pattern_parameters.pitch = 0.0;
    req.pattern_parameters.mode = msgs::PatternParameters::SAMPLING;
  }

  req.planning_mode = msgs::StepPlanRequest::PLANNING_MODE_PATTERN;
  req.parameter_set_name = params_.parameter_set_name;

  // send request
  if (!step_plan_request_client_.call(step_plan_request_srv.request, step_plan_request_srv.response))
  {
    ROS_ERROR("[PatternGenerator] Can't call footstep planner!");
    return;
  }

  // handle new step plan
  if (!step_plan_request_srv.response.step_plan.steps.size())
    return;

  updateFootstepMap(step_map_, step_plan_request_srv.response.step_plan.steps);

  if (complete_step_plan_.steps.empty()) // received step plan first time
    complete_step_plan_ = step_plan_request_srv.response.step_plan;
  else // just update steps
    mapToVectorIndexed(step_map_, complete_step_plan_.steps, 0, step_plan_request_srv.response.step_plan.steps.back().step_index);

  newest_step_plan_ = step_plan_request_srv.response.step_plan;
  newest_step_plan_.header.stamp = ros::Time::now();
  newest_step_plan_.header.seq++;
  has_new_steps_ = true;

  if (!setNextStartStepIndex(newest_step_plan_.steps.back().step_index))
  {
    ROS_ERROR("[PatternGenerator] generateSteps: Last step index of recent pattern was wrong. Resetting now!");
    reset();
  }
}
}
