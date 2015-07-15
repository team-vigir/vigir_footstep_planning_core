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
#include <vigir_foot_pose_transformer/foot_pose_transformer_node.h>

namespace vigir_footstep_planning
{
FootPoseTransformerNode::FootPoseTransformerNode(ros::NodeHandle& nh)
  : foot_pose_transformer(nh)
{
  // start own services
  transform_foot_pose_srv = nh.advertiseService("transform_foot_pose", &FootPoseTransformerNode::transformFootPoseService, this);
  transform_feet_poses_srv = nh.advertiseService("transform_feet_poses", &FootPoseTransformerNode::transformFeetPosesService, this);
  transform_step_plan_srv = nh.advertiseService("transform_step_plan", &FootPoseTransformerNode::transformStepPlanService, this);
}

FootPoseTransformerNode::~FootPoseTransformerNode()
{
}

bool FootPoseTransformerNode::transformFootPoseService(msgs::TransformFootPoseService::Request& req, msgs::TransformFootPoseService::Response& resp)
{
  resp.foot = req.foot;
  resp.status = foot_pose_transformer.transform(resp.foot, req.target_frame.data);
  return true; // return always true so the message is returned
}

bool FootPoseTransformerNode::transformFeetPosesService(msgs::TransformFeetPosesService::Request& req, msgs::TransformFeetPosesService::Response& resp)
{
  resp.feet = req.feet;
  resp.status = foot_pose_transformer.transform(resp.feet, req.target_frame.data);
  return true; // return always true so the message is returned
}

bool FootPoseTransformerNode::transformStepPlanService(msgs::TransformStepPlanService::Request& req, msgs::TransformStepPlanService::Response& resp)
{
  resp.step_plan = req.step_plan;
  resp.status = foot_pose_transformer.transform(resp.step_plan, req.target_frame.data);
  return true; // return always true so the message is returned
}
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "foot_pose_transformer_node");
  ros::NodeHandle nh;
  vigir_footstep_planning::FootPoseTransformerNode foot_pose_transformer_node(nh);
  ros::spin();

  return 0;
}
