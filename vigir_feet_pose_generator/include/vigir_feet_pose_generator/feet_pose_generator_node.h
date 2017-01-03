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

#ifndef VIGIR_FEET_POSE_GENERATOR_NODE_H__
#define VIGIR_FEET_POSE_GENERATOR_NODE_H__

#include <ros/ros.h>

#include <vigir_footstep_planning_lib/helper.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#include <vigir_feet_pose_generator/feet_pose_generator.h>



namespace vigir_footstep_planning
{
class FeetPoseGeneratorNode
{
public:
  FeetPoseGeneratorNode(ros::NodeHandle& nh);
  virtual ~FeetPoseGeneratorNode();

protected:
  // service calls
  bool generateFeetPoseService(msgs::GenerateFeetPoseService::Request& req, msgs::GenerateFeetPoseService::Response& resp);

  // action server calls
  void generateFeetPoseAction(SimpleActionServer<msgs::GenerateFeetPoseAction>::Ptr& as);

  // subscriber
  ros::Subscriber robot_pose_sub;
  ros::Subscriber robot_pose_with_cov_sub;
  ros::Subscriber terrain_model_sub;

  // service servers
  ros::ServiceServer generate_feet_pose_srv;

  // action servers
  SimpleActionServer<msgs::GenerateFeetPoseAction>::Ptr generate_feet_pose_as;

  FeetPoseGenerator feet_pose_generator;
};
}

#endif
