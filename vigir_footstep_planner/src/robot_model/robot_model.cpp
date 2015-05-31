#include <vigir_footstep_planner/robot_model/robot_model.h>

namespace vigir_footstep_planning
{
RobotModel::Ptr RobotModel::singelton = RobotModel::Ptr();

RobotModel::RobotModel()
{
}

RobotModel::Ptr& RobotModel::Instance()
{
  if (!singelton)
    singelton.reset(new RobotModel());
  return singelton;
}

void RobotModel::loadPlugins()
{
  // get collision check plugins
  PluginManager::getPluginsByType(Instance()->reachability_plugins);

  ROS_INFO("[RobotModel] Plugins loaded:");
  for (std::vector<ReachabilityPlugin::Ptr>::const_iterator itr = Instance()->reachability_plugins.begin(); itr != Instance()->reachability_plugins.end(); itr++)
  {
    const ReachabilityPlugin::Ptr& reachability_plugin = *itr;
    if (reachability_plugin)
      ROS_INFO("    %s (%s)", reachability_plugin->getName().c_str(), reachability_plugin->getTypeId().c_str());
  }
}

void RobotModel::loadParams(const ParameterSet& params)
{
  for (std::vector<ReachabilityPlugin::Ptr>::iterator itr = Instance()->reachability_plugins.begin(); itr != Instance()->reachability_plugins.end(); itr++)
  {
    ReachabilityPlugin::Ptr& reachability_plugin = *itr;
    if (reachability_plugin)
      reachability_plugin->loadParams(params);
  }
}

bool RobotModel::isReachable(const State& current, const State& next)
{
  for (std::vector<ReachabilityPlugin::Ptr>::const_iterator itr = Instance()->reachability_plugins.begin(); itr != Instance()->reachability_plugins.end(); itr++)
  {
    const ReachabilityPlugin::Ptr& reachability_plugin = *itr;
    if (reachability_plugin && !reachability_plugin->isReachable(current, next))
      return false;
  }
  return true;
}

bool RobotModel::isReachable(const State& left, const State& right, const State& swing)
{
  for (std::vector<ReachabilityPlugin::Ptr>::const_iterator itr = Instance()->reachability_plugins.begin(); itr != Instance()->reachability_plugins.end(); itr++)
  {
    const ReachabilityPlugin::Ptr& reachability_plugin = *itr;
    if (reachability_plugin && !reachability_plugin->isReachable(left, right, swing))
      return false;
  }
  return true;
}
}
