#include <vigir_adaptive_footstep_planner/adaptive_footstep_planner_node.h>

namespace vigir_footstep_planning
{
AdaptiveFootstepPlannerNode::AdaptiveFootstepPlannerNode(ros::NodeHandle& nh)
  : adaptive_footstep_planner(nh)
{
}

AdaptiveFootstepPlannerNode::~AdaptiveFootstepPlannerNode()
{
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "adaptive_footstep_planner");
  ros::NodeHandle nh;
  vigir_footstep_planning::AdaptiveFootstepPlannerNode planner(nh);
  ros::spin();

  return 0;
}
