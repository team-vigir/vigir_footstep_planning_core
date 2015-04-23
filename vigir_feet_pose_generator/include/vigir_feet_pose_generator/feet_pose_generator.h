//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_FEET_POSE_GENERATOR_H__
#define VIGIR_FEET_POSE_GENERATOR_H__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_lib/helper.h>

#include <vigir_terrain_classifier/terrain_model.h>



namespace vigir_footstep_planning
{
class FeetPoseGenerator
{
public:
  FeetPoseGenerator(ros::NodeHandle& nh);
  virtual ~FeetPoseGenerator();

  void setRobotPose(const geometry_msgs::PoseStampedConstPtr &robot_pose);
  void setRobotPoseWithCovariance(const geometry_msgs::PoseWithCovarianceStampedConstPtr &robot_pose);

  void setTerrainModel(const vigir_terrain_classifier::TerrainModelMsg::ConstPtr& terrain_model);

  msgs::ErrorStatus generateFeetPose(const msgs::FeetPoseRequest& request, msgs::Feet& feet);

  // typedefs
  typedef boost::shared_ptr<FeetPoseGenerator> Ptr;
  typedef boost::shared_ptr<const FeetPoseGenerator> ConstPtr;

protected:
  msgs::ErrorStatus updateFeetPose(msgs::Feet& feet) const;

  bool getCurrentFeetPose(msgs::Feet& feet);

  /**
   * This method assumes that the given pose is in center between both feet.
   */
  msgs::Feet generateFeetPose(const geometry_msgs::PoseStamped& pose);

  void pelvisToGroundTransform(msgs::Feet& feet) const;

  // service clients
  ros::ServiceClient transform_feet_poses_client;

  vigir_terrain_classifier::TerrainModel::Ptr terrain_model;

  std::string world_frame_id;
  std::string left_foot_frame_id;
  std::string right_foot_frame_id;

  double foot_separation;
  double left_foot_shift_z;
  double right_foot_shift_z;

  geometry_msgs::Vector3 pelvis_to_feet_center;

  bool has_robot_pose;
  geometry_msgs::PoseStamped robot_pose;

  tf::TransformListener tf_listener;
};
}

#endif
