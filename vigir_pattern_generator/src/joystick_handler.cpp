#include <vigir_pattern_generator/joystick_handler.h>



namespace vigir_footstep_planning
{
JoystickHandler::JoystickHandler(ros::NodeHandle& nh)
  : enable_generator(false)
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

void JoystickHandler::getJoystickCommand(double elapsed_time_sec, bool& enable, geometry_msgs::Twist& twist) const
{
  enable = enable_generator;

  ROS_INFO("%f/%f/%f %f %i", x_axis_->getValue(), y_axis_->getValue(), yaw_axis_->getValue(), enable_->getValue(), enable_->isPressed());

//  // check x axis
//  double joy_x = last_joy_msg->axes[4];
//  joy_x = std::abs(joy_x) > thresh_lin.x ? joy_x : 0.0;
//  updateJoystickCommand(elapsed_time_sec, joy_x, d_x, limits_min_lin_vel.x, limits_max_lin_vel.x, limits_min_lin_acc.x, limits_max_lin_acc.x, sensivity_lin.x);

//  // check y axis
//  double joy_y = last_joy_msg->axes[0];
//  joy_y =  std::abs(joy_y) > thresh_lin.y ? joy_y : 0.0;
//  updateJoystickCommand(elapsed_time_sec, joy_y, d_y, limits_min_lin_vel.y, limits_max_lin_vel.y, limits_min_lin_acc.y, limits_max_lin_acc.y, sensivity_lin.y);

//  // check yaw axis
//  double joy_yaw = last_joy_msg->axes[3];
//  joy_yaw = std::abs(joy_yaw) > thresh_rot.z ? joy_yaw : 0.0;
//  if (joy_x < 0.0)
//    joy_yaw = -joy_yaw;
//  updateJoystickCommand(elapsed_time_sec, joy_yaw, d_yaw, limits_min_rot_vel.z, limits_max_rot_vel.z, limits_min_rot_acc.z, limits_max_rot_acc.z, sensivity_rot.z);

  //ROS_INFO("%f (%f) / %f (%f) / %f (%f)", d_x, last_joy_msg->axes[4], d_y, last_joy_msg->axes[0], d_yaw, last_joy_msg->axes[3]);
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
