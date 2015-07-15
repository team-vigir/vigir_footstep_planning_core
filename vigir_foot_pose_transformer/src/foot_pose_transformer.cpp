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
#include <vigir_foot_pose_transformer/foot_pose_transformer.h>

namespace vigir_footstep_planning
{
FootPoseTransformer::FootPoseTransformer(ros::NodeHandle& nh)
{
  // obtain left foot transformation
  geometry_msgs::Vector3 left_origin;
  getXYZ(nh, "foot/left/foot_frame", left_origin);
  tf::Vector3 left_origin_tf;
  tf::vector3MsgToTF(left_origin, left_origin_tf);
  left_foot_frame_transform.setOrigin(left_origin_tf);

  geometry_msgs::Vector3 left_orientation;
  getRPY(nh, "foot/left/foot_frame", left_orientation);
  left_foot_frame_transform.setRotation(tf::createQuaternionFromRPY(left_orientation.x, left_orientation.y, left_orientation.z));

  // obtain right foot transformation
  geometry_msgs::Vector3 right_origin;
  getXYZ(nh, "foot/right/foot_frame", right_origin);
  tf::Vector3 right_origin_tf;
  tf::vector3MsgToTF(right_origin, right_origin_tf);
  right_foot_frame_transform.setOrigin(right_origin_tf);

  geometry_msgs::Vector3 right_orientation;
  getRPY(nh, "foot/right/foot_frame", right_orientation);
  right_foot_frame_transform.setRotation(tf::createQuaternionFromRPY(right_orientation.x, right_orientation.y, right_orientation.z));
}

FootPoseTransformer::~FootPoseTransformer()
{
}

msgs::ErrorStatus FootPoseTransformer::transform(geometry_msgs::Pose& pose, const tf::Transform& transform) const
{
  tf::Pose pose_tf;
  tf::poseMsgToTF(pose, pose_tf);
  pose_tf = pose_tf * transform;
  tf::poseTFToMsg(pose_tf, pose);
  return msgs::ErrorStatus();
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::Foot& foot, const std::string& target_frame) const
{
  if (foot.foot_index != msgs::Foot::LEFT && foot.foot_index != msgs::Foot::RIGHT)
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootPoseTransformer", "Foot index '" + boost::lexical_cast<std::string>(foot.foot_index) + "' is unknown!");

  if (target_frame == "planner") // to sole frame
    return transform(foot.pose, foot.foot_index == msgs::Foot::LEFT ? left_foot_frame_transform : right_foot_frame_transform);
  else if (target_frame == "robot") // to tf "foot" frame
    return transform(foot.pose, foot.foot_index == msgs::Foot::LEFT ? left_foot_frame_transform.inverse() : right_foot_frame_transform.inverse());
  else
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootPoseTransformer", "Target frame '" + target_frame + "' is unknown!");
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::Feet& feet, const std::string& target_frame) const
{
  msgs::ErrorStatus status;
  status += transform(feet.left, target_frame);
  status += transform(feet.right, target_frame);
  return status;
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::Step& step, const std::string& target_frame) const
{
  return transform(step.foot, target_frame);
}

msgs::ErrorStatus FootPoseTransformer::transform(msgs::StepPlan& step_plan, const std::string& target_frame) const
{
  msgs::ErrorStatus status;
  status += transform(step_plan.steps, target_frame);
  status += transform(step_plan.start.left, target_frame);
  status += transform(step_plan.start.right, target_frame);
  status += transform(step_plan.goal.left, target_frame);
  status += transform(step_plan.goal.right, target_frame);
  return status;
}
}
