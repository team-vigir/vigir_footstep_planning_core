/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
#include <vigir_feet_pose_generator/feet_pose_generator.h>

namespace vigir_footstep_planning
{
FeetPoseGenerator::FeetPoseGenerator(ros::NodeHandle& nh)
  : has_robot_pose(false)
{
  nh.getParam("foot/separation", foot_separation);

  nh.getParam("foot/left/frame_id", left_foot_frame_id);
  nh.getParam("foot/right/frame_id", right_foot_frame_id);

  nh.getParam("foot/left/foot_frame/z", left_foot_shift_z);
  nh.getParam("foot/right/foot_frame/z", right_foot_shift_z);

  // strip '/'
  strip(left_foot_frame_id, '/');
  strip(right_foot_frame_id, '/');

  nh.getParam("pelvis_to_feet_center/x", pelvis_to_feet_center.x);
  nh.getParam("pelvis_to_feet_center/y", pelvis_to_feet_center.y);
  nh.getParam("pelvis_to_feet_center/z", pelvis_to_feet_center.z);

  // start service clients
  transform_feet_poses_client = nh.serviceClient<msgs::TransformFeetPosesService>("transform_feet_poses");
}

FeetPoseGenerator::~FeetPoseGenerator()
{
}

void FeetPoseGenerator::setRobotPose(const geometry_msgs::PoseStampedConstPtr &robot_pose)
{
  this->robot_pose.header = robot_pose->header;
  this->robot_pose = *robot_pose;
  has_robot_pose = true;
}

void FeetPoseGenerator::setRobotPoseWithCovariance(const geometry_msgs::PoseWithCovarianceStampedConstPtr &robot_pose)
{
  this->robot_pose.header = robot_pose->header;
  this->robot_pose.pose = robot_pose->pose.pose;
  has_robot_pose = true;
}

void FeetPoseGenerator::setTerrainModel(const vigir_terrain_classifier::TerrainModelMsg::ConstPtr& terrain_model)
{
  // update terrain model
  if (this->terrain_model)
    this->terrain_model->fromMsg(*terrain_model);
  else
    this->terrain_model.reset(new vigir_terrain_classifier::TerrainModel(*terrain_model));
}

msgs::ErrorStatus FeetPoseGenerator::generateFeetPose(const msgs::FeetPoseRequest& request, msgs::Feet& feet)
{
  msgs::ErrorStatus status;

  // strip '/'
  std::string request_frame_id = request.header.frame_id;
  strip(request_frame_id, '/');

  if (request.flags & msgs::FeetPoseRequest::FLAG_CURRENT)
  {
    // try to get current feet pose
    if (getCurrentFeetPose(feet, request_frame_id))
    {
      return status;
    }
    // project robot (pelvis) pose to the ground
    else if (has_robot_pose)
    {
      std::string robot_pose_frame_id = strip_const(robot_pose.header.frame_id, '/');
      if (request_frame_id != robot_pose_frame_id)
        status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FeetPoseGenerator", "generateFeetPose: Frame ID of robot pose ('" + robot_pose_frame_id + "') and request ('" + request_frame_id + "') mismatch, automatic transformation is not implemented yet!");

      feet = generateFeetPose(robot_pose);
      //pelvisToGroundTransform(feet);
    }
    else
    {
      return status + ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FeetPoseGenerator", "generateFeetPose: No robot pose available for handling FLAG_CURRENT!");
    }
  }
  else
  {
    geometry_msgs::PoseStamped pose;
    pose.header = request.header;
    pose.pose = request.pose;
    feet = generateFeetPose(pose);
  }

  if (request.flags & msgs::FeetPoseRequest::FLAG_PELVIS_FRAME)
  {
    pelvisToGroundTransform(feet);
  }

  if (request.flags & msgs::FeetPoseRequest::FLAG_3D)
  {
    msgs::ErrorStatus temp_status;
    msgs::Feet temp_feet = feet;

    temp_status = updateFeetPose(temp_feet);
    if (hasError(temp_status))
      status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FeetPoseGenerator", "generateFeetPose: Couldn't determine either 3D nor height of feet!", false);
    else
    {
      status += temp_status;
      feet = temp_feet;
    }
  }

  if (request.flags & msgs::FeetPoseRequest::FLAG_CURRENT_Z)
  {
    msgs::Feet current_feet;
    if (getCurrentFeetPose(current_feet, request_frame_id))
    {
      feet.left.pose.position.z = current_feet.left.pose.position.z+left_foot_shift_z; // adding shift which is eliminated by transformToRobotFrame
      feet.right.pose.position.z = current_feet.right.pose.position.z+right_foot_shift_z; // adding shift which is eliminated by transformToRobotFrame
    }
    else
      status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FeetPoseGenerator", "generateFeetPose:  Couldn't determine height of current feet!");
  }

  transformToRobotFrame(feet, transform_feet_poses_client);

  return status;
}

msgs::ErrorStatus FeetPoseGenerator::updateFeetPose(msgs::Feet& feet) const
{
  if (!terrain_model || !terrain_model->hasTerrainModel())
    return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL, "FeetPoseGenerator", "updateFeetPose: No terrain model available!", false);

  msgs::ErrorStatus status;

  if (!terrain_model->update3DData(feet.left.pose) && !terrain_model->getHeight(feet.left.pose.position.x, feet.left.pose.position.y, feet.left.pose.position.z))
    status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FeetPoseGenerator", "updateFeetPose: Couldn't determine either 3D nor height of left foot!", false);

  if (!terrain_model->update3DData(feet.right.pose) && !terrain_model->getHeight(feet.right.pose.position.x, feet.right.pose.position.y, feet.right.pose.position.z))
    status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FeetPoseGenerator", "updateFeetPose: Couldn't determine either 3D nor height of right foot!", false);

  return status;
}

bool FeetPoseGenerator::getCurrentFeetPose(msgs::Feet& feet, const std::string& request_frame)
{
  if (tf_listener.canTransform(request_frame, left_foot_frame_id, ros::Time(0)) &&
      tf_listener.canTransform(request_frame, right_foot_frame_id, ros::Time(0)))
  {
    tf::StampedTransform t;
    tf_listener.lookupTransform(request_frame, left_foot_frame_id, ros::Time(0), t);
    tf::poseTFToMsg(t, feet.left.pose);
    feet.left.header.stamp = t.stamp_;
    feet.left.foot_index = msgs::Foot::LEFT;

    tf_listener.lookupTransform(request_frame, right_foot_frame_id, ros::Time(0), t);
    tf::poseTFToMsg(t, feet.right.pose);
    feet.right.header.stamp = t.stamp_;
    feet.right.foot_index = msgs::Foot::RIGHT;

    feet.header.stamp = ros::Time::now();
    feet.header.frame_id = feet.left.header.frame_id = feet.right.header.frame_id = request_frame;

    return true;
  }
  return false;
}

msgs::Feet FeetPoseGenerator::generateFeetPose(const geometry_msgs::PoseStamped& pose)
{
  msgs::Feet feet;
  double yaw = tf::getYaw(pose.pose.orientation);
  double shift_x = -sin(yaw) * (0.5 * foot_separation);
  double shift_y =  cos(yaw) * (0.5 * foot_separation);

  feet.header = pose.header;

  feet.left.header = pose.header;
  feet.left.foot_index = msgs::Foot::LEFT;
  feet.left.pose.position.x = pose.pose.position.x + shift_x;
  feet.left.pose.position.y = pose.pose.position.y + shift_y;
  feet.left.pose.position.z = pose.pose.position.z;
  feet.left.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  feet.right.header = pose.header;
  feet.right.foot_index = msgs::Foot::RIGHT;
  feet.right.pose.position.x = pose.pose.position.x - shift_x;
  feet.right.pose.position.y = pose.pose.position.y - shift_y;
  feet.right.pose.position.z = pose.pose.position.z;
  feet.right.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  return feet;
}

void FeetPoseGenerator::pelvisToGroundTransform(msgs::Feet& feet) const
{
  /// TODO: Transform pelvis_to_feet_center to world frame!
  feet.left.pose.position.x += pelvis_to_feet_center.x;
  feet.left.pose.position.y += pelvis_to_feet_center.y;
  feet.left.pose.position.z += pelvis_to_feet_center.z;

  feet.right.pose.position.x += pelvis_to_feet_center.x;
  feet.right.pose.position.y += pelvis_to_feet_center.y;
  feet.right.pose.position.z += pelvis_to_feet_center.z;
}
}
