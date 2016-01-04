//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef JOYSTICK_HANDLER_H__
#define JOYSTICK_HANDLER_H__

#include <ros/ros.h>

#include <geometry_msgs/Point.h>

#include <sensor_msgs/Joy.h>



namespace vigir_footstep_planning
{
class JoystickHandler
{
public:
  JoystickHandler(ros::NodeHandle& nh);
  virtual ~JoystickHandler();

  // joystick input
  void joyCallback(const sensor_msgs::Joy::ConstPtr& last_joy_msg);

  void updateJoystickCommands(double elapsed_time_sec, bool& enable, double& d_x, double& d_y, double& d_yaw) const;

  typedef boost::shared_ptr<JoystickHandler> Ptr;
  typedef boost::shared_ptr<JoystickHandler> ConstPtr;

protected:
  void updateJoystickCommand(double elapsed_time_sec, double joy_val, double& val, double min_vel, double max_vel, double min_acc, double max_acc, double sensivity) const;
  double convertJoyAxisToAcc(double elapsed_time_sec, double joy_val, double val, double min_vel, double max_vel) const;

  // subscriber
  ros::Subscriber joy_sub;

  // joystick settings
  geometry_msgs::Point thresh_lin;
  geometry_msgs::Point thresh_rot;
  geometry_msgs::Point sensivity_lin;
  geometry_msgs::Point sensivity_rot;
  geometry_msgs::Point limits_min_lin_vel;
  geometry_msgs::Point limits_max_lin_vel;
  geometry_msgs::Point limits_min_lin_acc;
  geometry_msgs::Point limits_max_lin_acc;
  geometry_msgs::Point limits_min_rot_vel;
  geometry_msgs::Point limits_max_rot_vel;
  geometry_msgs::Point limits_min_rot_acc;
  geometry_msgs::Point limits_max_rot_acc;

  // joystick input
  sensor_msgs::Joy::ConstPtr last_joy_msg;
  bool enable_generator;
};
}

#endif
