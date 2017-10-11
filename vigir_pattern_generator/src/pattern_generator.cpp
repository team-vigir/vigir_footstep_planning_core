#include <vigir_pattern_generator/pattern_generator.h>

namespace vigir_footstep_planning
{
PatternGenerator::PatternGenerator(ros::NodeHandle& nh)
{
  joystick_handler_.reset(new JoystickHandler(nh));

  nh.param("world_frame_id", world_frame_id_, std::string("/world"));
  nh.param("pattern_generator/number_of_steps", (int&)number_of_steps_needed_, 5);

  min_vel_x_ = nh.param("pattern_generator/limits/x/min", -0.1);
  max_vel_x_ = nh.param("pattern_generator/limits/x/max",  0.1);
  max_vel_y_ = nh.param("pattern_generator/limits/y",  0.1);
  max_vel_yaw_ = nh.param("pattern_generator/limits/yaw",  0.1);

  ros::service::waitForService("step_plan_request");

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
//    if (!setNextStartStepIndex(first_changeable_step_index_))
//    {
//      ROS_ERROR("[PatternGenerator] setParams: Replanning failed due to unknown start step.");
//      reset();
//    }
  }
  else
  {
    // triggers reset when enabling
    setEnabled(params.enable);
  }

  params_ = params;
}

void PatternGenerator::setEnabled(bool enable)
{
  // activation
  if (!isEnabled() && enable)
    reset();
  // deactivation
  else if (isEnabled() && !enable)
    generateSteps(0);

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
  return !last_step_sequence_.steps.empty();
}

bool PatternGenerator::hasNewSteps() const
{
  return !last_step_sequence_.steps.empty() && has_new_steps_;
}

void PatternGenerator::getCompleteStepPlan(msgs::StepPlan& step_plan) const
{
  step_plan_.toMsg(step_plan);
}

void PatternGenerator::getLastStepSequence(msgs::StepPlan& step_plan) const
{
  has_new_steps_ = false;
  step_plan = last_step_sequence_;
}

int PatternGenerator::getNextStartStepIndex() const
{
  return std::max(foot_start_step_index_left_, foot_start_step_index_right_);
}

void PatternGenerator::clearStepPlan()
{
  step_plan_.clear();
  last_step_sequence_ = msgs::StepPlan();
}

void PatternGenerator::update(const ros::TimerEvent& /*timer*/)
{
  // (timer.current_real - timer.last_real).toSec()

  // handle joystick input
  if (joystick_handler_ && params_.joystick_mode)
  {
    bool enable;
    joystick_handler_->getJoystickCommand(joystick_cmd_, enable);

    setEnabled(enable);
  }

  // if not enabled, just do nothing
  if (!isEnabled())
    return;

  // check if more steps are needed
  generateSteps(number_of_steps_needed_);

  if (isSimulationMode())
    updateFirstChangeableStepIndex(first_changeable_step_index_+1);
}

void PatternGenerator::updateFirstChangeableStepIndex(int first_changeable_step_index)
{
  if (!isEnabled())
    return;

  if (first_changeable_step_index < 0)
  {
    ROS_ERROR("[PatternGenerator] updateFirstChangeableStepIndex: Stopping due to invalid step index %i", first_changeable_step_index);
    setEnabled(false);
  }

  first_changeable_step_index_ = first_changeable_step_index;

  msgs::Step step;
  if (first_changeable_step_index_ > 0)
  {
    if (step_plan_.getStep(step, first_changeable_step_index_-1))
      updateFeetStartPose(step);
    else
    {
      ROS_ERROR("[PatternGenerator] updateFirstChangeableStepIndex: Internal error; step %i isn't in step plan.", first_changeable_step_index);
      setEnabled(false);
    }
  }

  if (step_plan_.getStep(step, first_changeable_step_index_))
    updateFeetStartPose(step);
  else
  {
    ROS_ERROR("[PatternGenerator] updateFirstChangeableStepIndex: Internal error; step %i isn't in step plan.", first_changeable_step_index);
    setEnabled(false);
  }
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

void PatternGenerator::generateSteps(unsigned int n)
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
  geometry_msgs::Twist cmd = params_.joystick_mode ? joystick_cmd_ : params_.cmd;

  if (cmd.linear.x > 0.0)
    cmd.linear.x *= max_vel_x_;
  else
    cmd.linear.x *= -min_vel_x_;

  cmd.linear.y *= max_vel_y_;
  cmd.angular.z *= max_vel_yaw_;

  // determine which foot has to move first (note: For the planner the start foot is fixed, thus, the first moved foot is start+1)
  unsigned int next_moving_foot_index = msgs::Foot::LEFT;
  if (getNextStartStepIndex() == 0 || ((cmd.linear.y != 0.0 || cmd.angular.z != 0.0) && number_of_steps_needed_ > 2))
  {
    if (cmd.linear.y < 0.0 || (cmd.linear.y == 0.0 && cmd.angular.z < 0.0))
      next_moving_foot_index = msgs::Foot::RIGHT;
    else
      next_moving_foot_index = msgs::Foot::LEFT;
  }
  else
  {
    if (foot_start_step_index_left_ < foot_start_step_index_right_) // the higher index is pointing on first changeable index
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
    req.pattern_parameters.close_step = true;
    req.pattern_parameters.extra_seperation = false;
    req.pattern_parameters.override = false; // disable here, it will override too much by now
    req.pattern_parameters.roll = 0.0;
    req.pattern_parameters.pitch = 0.0;
    req.pattern_parameters.mode = msgs::PatternParameters::SAMPLING;
  }

  //ROS_INFO_STREAM(req);

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
  {
    ROS_ERROR("[PatternGenerator] Received empty plan!");
    return;
  }

  last_step_sequence_ = step_plan_request_srv.response.step_plan;
  has_new_steps_ = true;

  if (params_.ignore_invalid_steps)
  {
    for (msgs::Step& step : last_step_sequence_.steps)
      step.valid = true;
  }

  step_plan_.updateStepPlan(last_step_sequence_);
  step_plan_.removeSteps(last_step_sequence_.steps.back().step_index+1);
}
}
