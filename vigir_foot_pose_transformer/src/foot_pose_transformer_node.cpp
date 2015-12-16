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
