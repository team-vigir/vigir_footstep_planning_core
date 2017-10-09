#include <vigir_pattern_generator/joystick_handler.h>



namespace vigir_footstep_planning
{
JoystickHandler::JoystickHandler(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue params;

  if (nh.getParam("joystick", params))
  {
    initJoystickInput(params, "x", x_axis_);
    initJoystickInput(params, "y", y_axis_);
    initJoystickInput(params, "yaw", yaw_axis_);
    initJoystickInput(params, "enable", enable_);
  }
  else
    ROS_ERROR("[JoystickHandler] Input parameters missing in config!");

  joy_sub = nh.subscribe("/joy", 1,& JoystickHandler::joyCallback, this);
}

JoystickHandler::~JoystickHandler()
{
}

void JoystickHandler::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (x_axis_)
    x_axis_->joyCallback(joy);

  if (y_axis_)
    y_axis_->joyCallback(joy);

  if (yaw_axis_)
    yaw_axis_->joyCallback(joy);

  if (enable_)
    enable_->joyCallback(joy);
}

void JoystickHandler::getJoystickCommand(geometry_msgs::Twist& twist, bool& enable) const
{
  ROS_INFO("%f/%f/%f %f %i", x_axis_->getValue(), y_axis_->getValue(), yaw_axis_->getValue(), enable_->getValue(), enable_->isPressed());

  twist = geometry_msgs::Twist();

  if (x_axis_ && x_axis_->isPressed())
    twist.linear.x = x_axis_->getValue();

  if (y_axis_ && y_axis_->isPressed())
    twist.linear.y = y_axis_->getValue();

  if (yaw_axis_ && yaw_axis_->isPressed())
    twist.angular.z = yaw_axis_->getValue();

  if (enable_)
    enable = enable_->isPressed();
}

void JoystickHandler::initJoystickInput(XmlRpc::XmlRpcValue& params, std::string name, JoystickInputHandle::Ptr& handle) const
{
  if (params.hasMember(name))
  {
    XmlRpc::XmlRpcValue& input_params = params[name];

    if (input_params.hasMember("button"))
      handle.reset(new JoystickButton(input_params));
    else if (input_params.hasMember("axis"))
      handle.reset(new JoystickAxis(input_params));
    else
      ROS_ERROR("[JoystickHandler] Couldn't handle '%s' input type!", name.c_str());
  }
  else
    ROS_ERROR("[JoystickHandler] '%s' input parameter missing in config!", name.c_str());
}

void JoystickHandler::updateJoystickCommand(double elapsed_time_sec, double joy_val, double& val, double min_vel, double max_vel, double min_acc, double max_acc, double sensivity) const
{
  // determine acceleration
  double acc = convertJoyAxisToAcc(elapsed_time_sec, joy_val, val, min_vel, max_vel) * sensivity;
  acc = std::min(max_acc, std::max(min_acc, acc));

  // update value
  val = std::min(max_vel, std::max(min_vel, val+acc));
}

double JoystickHandler::convertJoyAxisToAcc(double elapsed_time_sec, double joy_val, double val, double min_vel, double max_vel) const
{
  // using current val to determine joy_val scale
  if (val >= 0.0)
    return (joy_val*std::abs(max_vel) - val)*elapsed_time_sec;
  else
    return (joy_val*std::abs(min_vel) - val)*elapsed_time_sec;
}
}
