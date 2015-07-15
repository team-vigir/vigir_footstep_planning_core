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
