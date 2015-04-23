//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
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

#ifndef VIGIR_WALK_ANALYZER_H__
#define VIGIR_WALK_ANALYZER_H__

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <time.h>

#include <ros/ros.h>

#include <actionlib/action_definition.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#include <vigir_footstep_planning_lib/math.h>



namespace vigir_footstep_planning
{
class WalkAnalyzer
{
public:
  WalkAnalyzer(ros::NodeHandle& nh);
  ~WalkAnalyzer();

  void reset();

  void executeStepPlanGoalCallback(const msgs::ExecuteStepPlanActionGoal& execute_goal);
  void executeStepPlanResultCallback(const msgs::ExecuteStepPlanActionResult& execute_result);
  void stepFeedbackCallback(const msgs::StepPlanFeedback& feedback);

  // typedefs
  typedef boost::shared_ptr<WalkAnalyzer> Ptr;
  typedef boost::shared_ptr<const WalkAnalyzer> ConstPtr;

protected:
  virtual bool analyze(const msgs::Step& previous, const msgs::Step& current, const msgs::Step& next, std::string& result) const;
  bool analyzeData(const std::map<int, msgs::Step>& data, std::string& result) const;

  void writeResult(const std::string& logging_dir, const std::string& data_header);

  std::string currentDateTime() const;

  std::string logging_dir;
  std::string data_header;

  actionlib_msgs::GoalID current_goal_id;

  msgs::StepPlan current_step_plan;
  std::map<int, msgs::Step> step_feedback_map;
};
}

#endif
