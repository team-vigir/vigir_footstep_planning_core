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

#ifndef VIGIR_PATTERN_GENERATOR_NODE_H__
#define VIGIR_PATTERN_GENERATOR_NODE_H__

#include <ros/ros.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#include <vigir_pattern_generator/pattern_generator.h>



namespace vigir_footstep_planning
{
class PatternGeneratorNode
{
public:
  PatternGeneratorNode(ros::NodeHandle& nh);
  virtual ~PatternGeneratorNode();

  void update(const ros::TimerEvent& timer);

protected:
  void generatePattern(const msgs::StepPlanRequestConstPtr& step_plan_request);
  void setParams(const msgs::PatternGeneratorParametersConstPtr& params);

  bool generatePatternService(msgs::GeneratePatternService::Request& req, msgs::GeneratePatternService::Response& resp);
  bool setParamsService(msgs::PatternGeneratorParametersService::Request& req, msgs::PatternGeneratorParametersService::Response& resp);

  void generatePatternAction(SimpleActionServer<msgs::GeneratePatternAction>::Ptr& as);

  void executeStepPlanFeedback(const msgs::ExecuteStepPlanFeedbackConstPtr& feedback);

  // subscriber
  ros::Subscriber generate_pattern_sub;
  ros::Subscriber set_params_sub;

  // publisher
  ros::Publisher step_plan_pub;
  ros::Publisher step_plan_vis_pub;

  // service servers
  ros::ServiceServer generate_pattern_srv;
  ros::ServiceServer set_params_srv;

  // action clients
  SimpleActionClient<msgs::ExecuteStepPlanAction>::Ptr execute_step_plan_ac;

  // action servers
  SimpleActionServer<msgs::GeneratePatternAction>::Ptr generate_pattern_as;

  ros::Duration update_intervall;
  ros::Timer timer;

  PatternGenerator pattern_generator;

  bool republish_complete_step_plan;
};
}

#endif
