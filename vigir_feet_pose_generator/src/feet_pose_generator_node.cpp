#include <vigir_feet_pose_generator/feet_pose_generator_node.h>

namespace vigir_footstep_planning
{
FeetPoseGeneratorNode::FeetPoseGeneratorNode(ros::NodeHandle& nh)
  : feet_pose_generator(nh)
{
  // subscribe topics
  robot_pose_sub = nh.subscribe("/robot_pose", 1, &FeetPoseGenerator::setRobotPose, &feet_pose_generator);
  robot_pose_with_cov_sub = nh.subscribe("/robot_pose_with_cov", 1, &FeetPoseGenerator::setRobotPoseWithCovariance, &feet_pose_generator);
  terrain_model_sub = nh.subscribe("/terrain_model", 1, &FeetPoseGenerator::setTerrainModel, &feet_pose_generator);

  // start own services
  generate_feet_pose_srv = nh.advertiseService("generate_feet_pose", &FeetPoseGeneratorNode::generateFeetPoseService, this);

  // init action servers
  generate_feet_pose_as = SimpleActionServer<msgs::GenerateFeetPoseAction>::create(nh, "generate_feet_pose", true, boost::bind(&FeetPoseGeneratorNode::generateFeetPoseAction, this, boost::ref(generate_feet_pose_as)));
}

FeetPoseGeneratorNode::~FeetPoseGeneratorNode()
{
}

// --- service calls ---

bool FeetPoseGeneratorNode::generateFeetPoseService(msgs::GenerateFeetPoseService::Request& req, msgs::GenerateFeetPoseService::Response& resp)
{
  resp.status = feet_pose_generator.generateFeetPose(req.request, resp.feet);
  return true; // return always true so the message is returned
}

//--- action server calls ---

void FeetPoseGeneratorNode::generateFeetPoseAction(SimpleActionServer<msgs::GenerateFeetPoseAction>::Ptr& as)
{
  const msgs::GenerateFeetPoseGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::GenerateFeetPoseResult result;
  result.header = goal->request.header;
  result.status = feet_pose_generator.generateFeetPose(goal->request, result.feet);

  as->finish(result);
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feet_pose_generator");
  ros::NodeHandle nh;
  vigir_footstep_planning::FeetPoseGeneratorNode generator(nh);
  ros::spin();

  return 0;
}
