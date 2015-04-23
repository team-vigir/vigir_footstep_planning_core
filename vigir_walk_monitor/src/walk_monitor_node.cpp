#include <vigir_walk_monitor/walk_monitor_node.h>

namespace vigir_footstep_planning
{
WalkMonitorNode::WalkMonitorNode(ros::NodeHandle& nh)
  : walk_monitor(nh)
{
}

WalkMonitorNode::~WalkMonitorNode()
{
}
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "walk_monitor_node");
  ros::NodeHandle nh;
  vigir_footstep_planning::WalkMonitorNode walk_monitor_node(nh);
  ros::spin();

  return 0;
}
