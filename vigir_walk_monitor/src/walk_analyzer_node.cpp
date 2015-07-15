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
#include <vigir_walk_monitor/walk_analyzer_node.h>

namespace vigir_footstep_planning
{
WalkAnalyzerNode::WalkAnalyzerNode(ros::NodeHandle& nh, WalkAnalyzer::Ptr walk_anaylzer)
  : walk_analyzer(walk_anaylzer)
{
  // subscribe topics, note: set large queue size to enable replaying bag files instant
  execute_step_plan_goal_sub = nh.subscribe("execute_step_plan/goal", 0, &WalkAnalyzer::executeStepPlanGoalCallback, walk_analyzer.get());
  execute_step_plan_result_sub = nh.subscribe("execute_step_plan/result", 0, &WalkAnalyzer::executeStepPlanResultCallback, walk_analyzer.get());
  step_feedback_sub = nh.subscribe("step_feedback", 0, &WalkAnalyzer::stepFeedbackCallback, walk_analyzer.get());
}

WalkAnalyzerNode::WalkAnalyzerNode(ros::NodeHandle& nh)
  : WalkAnalyzerNode(nh, WalkAnalyzer::Ptr(new WalkAnalyzer(nh)))
{
}

WalkAnalyzerNode::~WalkAnalyzerNode()
{
}
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "walk_analyzer_node");
  ros::NodeHandle nh;
  vigir_footstep_planning::WalkAnalyzerNode walk_analyzer_node(nh);
  ros::spin();

  return 0;
}
