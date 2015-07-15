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
#include <vigir_pattern_generator/pattern_generator_node.h>

namespace vigir_footstep_planning
{
PatternGeneratorNode::PatternGeneratorNode(ros::NodeHandle& nh)
  : pattern_generator(nh)
{
  double update_rate;
  nh.param("pattern_generator/update_rate", update_rate, 10.0);
  nh.param("pattern_generator/republish_complete_step_plan", republish_complete_step_plan, false);

  // subscribe topics
  generate_pattern_sub = nh.subscribe("pattern_generator/generate_pattern", 1, &PatternGeneratorNode::generatePattern, this);
  set_params_sub = nh.subscribe("pattern_generator/set_params", 1, &PatternGeneratorNode::setParams, this);

  // publish topics
  step_plan_pub = nh.advertise<msgs::StepPlan>("step_plan", 1);
  step_plan_vis_pub = nh.advertise<msgs::StepPlan>("vis/step_plan", 1);

  // start own services
  generate_pattern_srv = nh.advertiseService("pattern_generator/generate_pattern", &PatternGeneratorNode::generatePatternService, this);
  set_params_srv = nh.advertiseService("pattern_generator/set_params", &PatternGeneratorNode::setParamsService, this);

  // init action client
  execute_step_plan_ac = SimpleActionClient<msgs::ExecuteStepPlanAction>::create(nh, "/execute_step_plan");

  // init action servers
  generate_pattern_as = SimpleActionServer<msgs::GeneratePatternAction>::create(nh, "pattern_generator/generate_pattern", true, boost::bind(&PatternGeneratorNode::generatePatternAction, this, boost::ref(generate_pattern_as)));

  // init timer to periodically call the pattern generator
  update_intervall = ros::Duration(1.0/update_rate);
  timer = nh.createTimer(update_intervall, &PatternGeneratorNode::update, this);
}

PatternGeneratorNode::~PatternGeneratorNode()
{
}

void PatternGeneratorNode::generatePattern(const msgs::StepPlanRequestConstPtr& step_plan_request)
{
  msgs::StepPlan step_plan;
  msgs::ErrorStatus status = pattern_generator.generatePattern(*step_plan_request, step_plan);

  if (status.error == msgs::ErrorStatus::NO_ERROR)
  {
    step_plan_pub.publish(step_plan);
    step_plan_vis_pub.publish(step_plan);
  }
}

void PatternGeneratorNode::setParams(const msgs::PatternGeneratorParametersConstPtr& params)
{
  pattern_generator.setParams(*params);
}

bool PatternGeneratorNode::generatePatternService(msgs::GeneratePatternService::Request& req, msgs::GeneratePatternService::Response& resp)
{
  resp.status = pattern_generator.generatePattern(req.plan_request, resp.step_plan);

  if (resp.status.error == msgs::ErrorStatus::NO_ERROR)
    step_plan_vis_pub.publish(resp.step_plan);

  return true; // return always true so the message is returned
}

bool PatternGeneratorNode::setParamsService(msgs::PatternGeneratorParametersService::Request& req, msgs::PatternGeneratorParametersService::Response& /*resp*/)
{
  pattern_generator.setParams(req.params);
  return true; // return always true so the message is returned
}

void PatternGeneratorNode::generatePatternAction(SimpleActionServer<msgs::GeneratePatternAction>::Ptr& as)
{
  const msgs::GeneratePatternGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::GeneratePatternResult result;
  result.status = pattern_generator.generatePattern(goal->plan_request, result.step_plan);

  if (result.status.error == msgs::ErrorStatus::NO_ERROR)
    step_plan_vis_pub.publish(result.step_plan);

  as->finish(result);
}

void PatternGeneratorNode::executeStepPlanFeedback(const msgs::ExecuteStepPlanFeedbackConstPtr& feedback)
{
  pattern_generator.updateLastPerformedStepIndex(feedback->last_performed_step_index);
  pattern_generator.updateFirstChangeableStepIndex(feedback->first_changeable_step_index);
}

void PatternGeneratorNode::update(const ros::TimerEvent& timer)
{
  pattern_generator.update(timer);

  if (!pattern_generator.hasNewSteps())
    return;

  // publish vis of entire plan
  msgs::StepPlan complete_step_plan;
  pattern_generator.getCompleteStepPlan(complete_step_plan);
  step_plan_vis_pub.publish(complete_step_plan);

  msgs::ExecuteStepPlanGoal goal;
  pattern_generator.getNewestStepPlan(goal.step_plan); // must be getted to reset hasNew flag

  if (republish_complete_step_plan)
    goal.step_plan = complete_step_plan;

  // send to controller if available
  if (!pattern_generator.isSimulationMode() && execute_step_plan_ac->waitForServer(update_intervall))
  {
    execute_step_plan_ac->sendGoal(goal,
                                   SimpleActionClient<msgs::ExecuteStepPlanAction>::SimpleDoneCallback(),
                                   SimpleActionClient<msgs::ExecuteStepPlanAction>::SimpleActiveCallback(),
                                   boost::bind(&PatternGeneratorNode::executeStepPlanFeedback, this, _1));
  }
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vigir_pattern_generator");
  ros::NodeHandle nh;
  vigir_footstep_planning::PatternGeneratorNode patternGeneratorNode(nh);
  ros::spin();

  return 0;
}
