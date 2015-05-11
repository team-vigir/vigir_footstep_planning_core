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
