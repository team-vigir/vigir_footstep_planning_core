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
  set_params_sub = nh.subscribe("pattern_generator/set_params", 1, &PatternGeneratorNode::setParams, this);

  // publish topics
  step_plan_pub = nh.advertise<msgs::StepPlan>("step_plan", 1);
  step_plan_vis_pub = nh.advertise<msgs::StepPlan>("vis/step_plan", 1);

  // start own services
  set_params_srv = nh.advertiseService("pattern_generator/set_params", &PatternGeneratorNode::setParamsService, this);

  // init action client
  execute_step_plan_ac = SimpleActionClient<msgs::ExecuteStepPlanAction>::create(nh, "execute_step_plan");

  // init timer to periodically call the pattern generator
  update_intervall = ros::Duration(1.0/update_rate);
  timer = nh.createTimer(update_intervall, &PatternGeneratorNode::update, this);
}

PatternGeneratorNode::~PatternGeneratorNode()
{
}

void PatternGeneratorNode::setParams(const msgs::PatternGeneratorParametersConstPtr& params)
{
  pattern_generator.setParams(*params);
}

bool PatternGeneratorNode::setParamsService(msgs::PatternGeneratorParametersService::Request& req, msgs::PatternGeneratorParametersService::Response& /*resp*/)
{
  pattern_generator.setParams(req.params);
  return true; // return always true so the message is returned
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
