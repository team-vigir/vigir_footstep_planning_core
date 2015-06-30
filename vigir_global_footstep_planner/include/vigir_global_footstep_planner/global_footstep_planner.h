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

#ifndef GLOBAL_FOOTSTEP_PLANNER_H__
#define GLOBAL_FOOTSTEP_PLANNER_H__

#include <ros/ros.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_msgs/step_plan.h>

#include <vigir_footstep_planner/footstep_planner.h>



namespace vigir_footstep_planning
{
class GlobalFootstepPlanner
{
public:
  GlobalFootstepPlanner(FootstepPlanner::Ptr& footstep_planner);
  GlobalFootstepPlanner();
  virtual ~GlobalFootstepPlanner();

  void init(ros::NodeHandle& nh);

  msgs::ErrorStatus setStepPlan(const msgs::StepPlan &result);
  msgs::ErrorStatus getStepPlan(msgs::StepPlan& result) const;

  msgs::ErrorStatus appendStepPlan(const std::vector<msgs::StepPlan>& step_plans, msgs::StepPlan& result) const;
  msgs::ErrorStatus stitchStepPlan(const std::vector<msgs::StepPlan>& step_plans, msgs::StepPlan& result) const;

  msgs::ErrorStatus editStep(const msgs::EditStep& edit_step, const msgs::StepPlan& step_plan, std::vector<msgs::StepPlan>& result) const;

  // typedefs
  typedef boost::shared_ptr<GlobalFootstepPlanner> Ptr;
  typedef boost::shared_ptr<const GlobalFootstepPlanner> ConstPtr;

protected:
  // local footstep planner
  FootstepPlanner::Ptr footstep_planner;

  // service clients
  ros::ServiceClient generate_feet_pose_client;

  // internal data structure of step plan
  StepPlan::Ptr step_plan;
};
}

#endif
