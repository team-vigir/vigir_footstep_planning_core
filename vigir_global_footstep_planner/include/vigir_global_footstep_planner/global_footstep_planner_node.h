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

#ifndef GLOBAL_FOOTSTEP_PLANNER_NODE_H__
#define GLOBAL_FOOTSTEP_PLANNER_NODE_H__

#include <ros/ros.h>

#include <vigir_footstep_planning_lib/helper.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planner/footstep_planner_node.h>
#include <vigir_global_footstep_planner/global_footstep_planner.h>



namespace vigir_footstep_planning
{
class GlobalFootstepPlannerNode
  : public FootstepPlannerNode
{
public:
  GlobalFootstepPlannerNode(ros::NodeHandle& nh);
  virtual ~GlobalFootstepPlannerNode();

  void init(ros::NodeHandle& nh);

protected:
  // subscriber
  void editStep(const msgs::EditStepConstPtr& edit_step);
  void setStepPlan(const msgs::StepPlanConstPtr& step_plan);
  void stitchStepPlan(const std::vector<msgs::StepPlan>& step_plans);
  void stitchStepPlan(const msgs::StepPlanConstPtr& step_plan);

  // service calls
  bool editStepService(msgs::EditStepService::Request& req, msgs::EditStepService::Response& resp);
  bool setStepPlanService(msgs::SetStepPlanService::Request& req, msgs::SetStepPlanService::Response& resp);
  bool getStepPlanService(msgs::GetStepPlanService::Request& req, msgs::GetStepPlanService::Response& resp);
  bool stitchStepPlanService(msgs::StitchStepPlanService::Request& req, msgs::StitchStepPlanService::Response& resp);

  // action server calls
  void editStepAction(SimpleActionServer<msgs::EditStepAction>::Ptr& as);
  void setStepPlanAction(SimpleActionServer<msgs::SetStepPlanAction>::Ptr& as);
  void getStepPlanAction(SimpleActionServer<msgs::GetStepPlanAction>::Ptr& as);
  void stitchStepPlanAction(SimpleActionServer<msgs::StitchStepPlanAction>::Ptr& as);

  // subscriber
  ros::Subscriber edit_step_sub;
  ros::Subscriber set_step_plan_sub;
  ros::Subscriber stitch_step_plan_sub;

  // service servers
  ros::ServiceServer edit_step_srv;
  ros::ServiceServer set_step_plan_srv;
  ros::ServiceServer get_step_plan_srv;
  ros::ServiceServer stitch_step_plan_srv;

  // action servers
  SimpleActionServer<msgs::EditStepAction>::Ptr edit_step_as;
  SimpleActionServer<msgs::SetStepPlanAction>::Ptr set_step_plan_as;
  SimpleActionServer<msgs::GetStepPlanAction>::Ptr get_step_plan_as;
  SimpleActionServer<msgs::StitchStepPlanAction>::Ptr stitch_step_plan_as;

  GlobalFootstepPlanner::Ptr global_footstep_planner;
};
}

#endif
