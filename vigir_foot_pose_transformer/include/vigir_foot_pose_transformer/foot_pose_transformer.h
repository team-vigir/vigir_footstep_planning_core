//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
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

#ifndef VIGIR_FOOTSTEP_PLAN_TRANSFORMER_H__
#define VIGIR_FOOTSTEP_PLAN_TRANSFORMER_H__

#include <ros/ros.h>
#include <tf/tf.h>

#include <vigir_footstep_planning_lib/helper.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>



namespace vigir_footstep_planning
{
/**
 * Transforms footstep plans from one frame to another
 */
class FootPoseTransformer
{
public:
  FootPoseTransformer(ros::NodeHandle& nh);
  ~FootPoseTransformer();

  // generic interface for transformation
  msgs::ErrorStatus transform(geometry_msgs::Pose& pose, const tf::Transform& transform) const;
  msgs::ErrorStatus transform(msgs::Foot& foot, const std::string& target_frame) const;
  msgs::ErrorStatus transform(msgs::Feet& feet, const std::string& target_frame) const;
  msgs::ErrorStatus transform(msgs::Step& step, const std::string& target_frame) const;
  msgs::ErrorStatus transform(msgs::StepPlan& step_plan, const std::string& target_frame) const;

  template<template <typename...> class Container>
  msgs::ErrorStatus transform(Container<msgs::Step>& cont, const std::string& target_frame) const
  {
    msgs::ErrorStatus status;
    for (typename Container<msgs::Step>::iterator itr = cont.begin(); itr != cont.end(); itr++)
      status += transform(itr->foot, target_frame);
    return status;
  }

  // specialized transforms
  template <typename T>
  msgs::ErrorStatus transformToPlannerFrame(T& p) const
  {
    return transform(p, "planner");
  }

  template <typename T>
  msgs::ErrorStatus transformToRobotFrame(T& p) const
  {
    return transform(p, "robot");
  }

  // typedefs
  typedef boost::shared_ptr<FootPoseTransformer> Ptr;
  typedef boost::shared_ptr<const FootPoseTransformer> ConstPtr;

protected:
  // Transformation: robot's "foot" tf frame -> planner foot frame (center of sole)
  tf::Transform left_foot_frame_transform;
  tf::Transform right_foot_frame_transform;
};
}

#endif
