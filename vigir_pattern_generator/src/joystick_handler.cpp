#include <vigir_pattern_generator/joystick_handler.h>

namespace vigir_footstep_planning
{
JoystickHandler::JoystickHandler(ros::NodeHandle& nh)
  : enable_generator(false)
{
  // get threshold values for joystick control
  nh.param("joystick/thresh_x", thresh_lin.x, 0.0);
  nh.param("joystick/thresh_y", thresh_lin.y, 0.0);
  nh.param("joystick/thresh_yaw", thresh_rot.z, 0.0);

  // get acceleration factors for joystick control
  nh.param("joystick/sensivity_x", sensivity_lin.x, 0.0);
  nh.param("joystick/sensivity_y", sensivity_lin.y, 0.0);
  nh.param("joystick/sensivity_yaw", sensivity_rot.z, 0.0);

  // get velocity limits for joystick control
  nh.param("joystick/min_vel_x", limits_min_lin_vel.x, 0.0);
  nh.param("joystick/max_vel_x", limits_max_lin_vel.x, 0.0);
  nh.param("joystick/min_vel_y", limits_min_lin_vel.y, 0.0);
  nh.param("joystick/max_vel_y", limits_max_lin_vel.y, 0.0);
  nh.param("joystick/min_vel_yaw", limits_min_rot_vel.z, 0.0); limits_min_rot_vel.z *= (M_PI / 180.0);
  nh.param("joystick/max_vel_yaw", limits_max_rot_vel.z, 0.0); limits_max_rot_vel.z *= (M_PI / 180.0);

  // get acceleration limits for joystick control
  nh.param("joystick/min_acc_x", limits_min_lin_acc.x, 0.0);
  nh.param("joystick/max_acc_x", limits_max_lin_acc.x, 0.0);
  nh.param("joystick/min_acc_y", limits_min_lin_acc.y, 0.0);
  nh.param("joystick/max_acc_y", limits_max_lin_acc.y, 0.0);
  nh.param("joystick/min_acc_yaw", limits_min_rot_acc.z, 0.0); limits_min_rot_acc.z *= (M_PI / 180.0);
  nh.param("joystick/max_acc_yaw", limits_max_rot_acc.z, 0.0); limits_max_rot_acc.z *= (M_PI / 180.0);

  // get joystick control mapping
  nh.param("joystick/x_axis", x_axis, 0);
  nh.param("joystick/y_axis", y_axis, 1);
  nh.param("joystick/yaw_axis", yaw_axis, 3);
  nh.param("joystick/enable_generator_btn", enable_generator_btn, 0);
  nh.param("joystick/disable_generator_btn", disable_generator_btn, 1);

  joy_sub = nh.subscribe("/joy", 1,& JoystickHandler::joyCallback, this);
}

JoystickHandler::~JoystickHandler()
{
}

void JoystickHandler::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  last_joy_msg = joy;

  if (last_joy_msg->buttons.size() < std::max(enable_generator_btn, disable_generator_btn)+1)
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "Controller doesn't have the needed button count of " << std::max(enable_generator_btn, disable_generator_btn)+1 <<
                       ". Please check your config file.");
    return;
  }
  int axis_max = std::max(std::max(x_axis, y_axis), yaw_axis)+1;
  if (last_joy_msg->axes.size() < axis_max) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "Controller doesn't have the needed axis count of " << axis_max << ". Please check your config file.");
    return;
  }

  // check for (de-)activation
  if (last_joy_msg->buttons[enable_generator_btn])
    enable_generator = true;
  else if (last_joy_msg->buttons[disable_generator_btn])
    enable_generator = false;
}

void JoystickHandler::updateJoystickCommands(double elapsed_time_sec, bool& enable, double& d_x, double& d_y, double& d_yaw) const
{
  if (!last_joy_msg)
  {
    ROS_WARN_THROTTLE(1.0, "Didn't get joystick input so far. Check if joystick is connected and joy node running!");
    return;
  }

  enable = enable_generator;

  // check x axis
  double joy_x = last_joy_msg->axes[x_axis];
  joy_x = std::abs(joy_x) > thresh_lin.x ? joy_x : 0.0;
  updateJoystickCommand(elapsed_time_sec, joy_x, d_x, limits_min_lin_vel.x, limits_max_lin_vel.x, limits_min_lin_acc.x, limits_max_lin_acc.x, sensivity_lin.x);

  // check y axis
  double joy_y = last_joy_msg->axes[y_axis];
  joy_y =  std::abs(joy_y) > thresh_lin.y ? joy_y : 0.0;
  updateJoystickCommand(elapsed_time_sec, joy_y, d_y, limits_min_lin_vel.y, limits_max_lin_vel.y, limits_min_lin_acc.y, limits_max_lin_acc.y, sensivity_lin.y);

  // check yaw axis
  double joy_yaw = last_joy_msg->axes[yaw_axis];
  joy_yaw = std::abs(joy_yaw) > thresh_rot.z ? joy_yaw : 0.0;
  if (joy_x < 0.0)
    joy_yaw = -joy_yaw;
  updateJoystickCommand(elapsed_time_sec, joy_yaw, d_yaw, limits_min_rot_vel.z, limits_max_rot_vel.z, limits_min_rot_acc.z, limits_max_rot_acc.z, sensivity_rot.z);
  //ROS_INFO_STREAM_THROTTLE(0.25, "Current command: " << joy_x << ", " << joy_y << ", " << joy_yaw);
  ROS_INFO_STREAM_THROTTLE(0.25, "Current deltas: " << d_x << ", " << d_y << ", " << d_yaw);
}


void JoystickHandler::updateJoystickCommand(double elapsed_time_sec, double joy_val, double& val, double min_vel, double max_vel, double min_acc, double max_acc, double sensivity) const
{
  // determine acceleration
  double acc = convertJoyAxisToAcc(elapsed_time_sec, joy_val, val, min_vel, max_vel) * sensivity;
  acc = std::min(max_acc, std::max(min_acc, acc));

  // update value
  val += acc;
  val = std::min(max_vel, std::max(min_vel, val));
  // ROS_INFO_STREAM_THROTTLE(0.25, "Current acceleration: " << acc);
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
