//=================================================================================================
// Copyright (c) 2017, Alexander Stumpf, TU Darmstadt
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
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Joy.h>



namespace vigir_footstep_planning
{
class JoystickInputHandle
{
public:
  JoystickInputHandle(double thresh = 1.0)
    : val_(0.0)
    , thresh_(thresh)
  {}

  virtual void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) = 0;

  virtual double getValue() const { return val_; }
  virtual bool isPressed() const { return std::abs(val_) >= thresh_; }

  typedef boost::shared_ptr<JoystickInputHandle> Ptr;
  typedef boost::shared_ptr<JoystickInputHandle> ConstPtr;

protected:
  double val_;
  double thresh_;
};

class JoystickButton
  : public JoystickInputHandle
{
public:
  JoystickButton(int button, double thresh = 1.0)
    : JoystickInputHandle(thresh)
    , button_id_(button)
  {
    ROS_INFO("[Button %i]", button_id_);
  }

  JoystickButton(XmlRpc::XmlRpcValue& params)
    : JoystickButton(params["button"], params["thresh"])
  {}

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) override
  {
    if (joy_msg->buttons.size() <= button_id_)
      ROS_ERROR_THROTTLE(1.0, "Couldn't read button %i as only %lu buttons exist. Maybe wrong controller connected?", button_id_, joy_msg->buttons.size());
    else
      val_ = joy_msg->buttons[button_id_];
  }

protected:
  int button_id_;
};

class JoystickAxis
  : public JoystickInputHandle
{
public:
  JoystickAxis(int axis, double thresh = 1.0, double min_val = -1.0, double max_val = 1.0, double zero_val = 0.0, double calib_offset = 0.0, bool invert = false)
    : JoystickInputHandle(thresh)
    , axis_id_(axis)
    , min_(min_val)
    , max_(max_val)
    , zero_val_(zero_val)
    , calib_offset_(calib_offset)
    , invert_(invert)
    , scale_factor_(2.0/(max_-min_))
    , scale_offset_(0.5*(max_+min_))
  {
    ROS_INFO("[Axis %i] %f - (%f, %f) - %f", axis, thresh, min_val, max_val, calib_offset);
  }

  JoystickAxis(XmlRpc::XmlRpcValue& params)
    : JoystickAxis(params["axis"], params["thresh"], params["min"], params["max"], params["zero"], params["calib"], params["invert"])
  {}

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) override
  {
    if (joy_msg->axes.size() <= axis_id_)
      ROS_ERROR_THROTTLE(1.0, "Couldn't read axis %i as only %lu axes exist. Maybe wrong controller connected?", axis_id_, joy_msg->axes.size());
    else
    {
      // read value und remove offset
      val_ = joy_msg->axes[axis_id_] - calib_offset_;

      // invert if set
      if (invert_)
        val_ = -val_;

      // rescale to [-1; 1] based on min max borders
      val_ = std::min(std::max(val_, min_), max_); // clamping in [min; max]
      val_ = val_ * scale_factor_ + scale_offset_; // rescale
    }
  }

  bool isPressed() const override { return std::abs(val_-zero_val_) >= thresh_; }

protected:
  int axis_id_;

  double min_;
  double max_;
  double zero_val_;

  double calib_offset_;

  bool invert_;

  double scale_factor_;
  double scale_offset_;
};

class JoystickHandler
{
public:
  JoystickHandler(ros::NodeHandle& nh);
  virtual ~JoystickHandler();

  // joystick input
  void joyCallback(const sensor_msgs::Joy::ConstPtr& last_joy_msg);

  void getJoystickCommand(geometry_msgs::Twist& twist, bool& enable) const;

  typedef boost::shared_ptr<JoystickHandler> Ptr;
  typedef boost::shared_ptr<JoystickHandler> ConstPtr;

protected:
  void initJoystickInput(XmlRpc::XmlRpcValue& params, std::string name, JoystickInputHandle::Ptr& handle) const;

  void updateJoystickCommand(double elapsed_time_sec, double joy_val, double& val, double min_vel, double max_vel, double min_acc, double max_acc, double sensivity) const;
  double convertJoyAxisToAcc(double elapsed_time_sec, double joy_val, double val, double min_vel, double max_vel) const;

  // joystick settings
  JoystickInputHandle::Ptr x_axis_;
  JoystickInputHandle::Ptr y_axis_;
  JoystickInputHandle::Ptr yaw_axis_;

  JoystickInputHandle::Ptr enable_;

  bool has_joy_msg_;

  // subscriber
  ros::Subscriber joy_sub_;
};
}

#endif
