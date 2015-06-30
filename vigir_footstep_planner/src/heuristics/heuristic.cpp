#include <vigir_footstep_planner/heuristics/heuristic.h>

namespace vigir_footstep_planning
{
Heuristic::Ptr Heuristic::singelton = Heuristic::Ptr();

Heuristic::Heuristic()
{
}

Heuristic::Ptr& Heuristic::Instance()
{
  if (!singelton)
    singelton.reset(new Heuristic());
  return singelton;
}

void Heuristic::loadPlugins()
{
  // get step cost estimator plugins
  vigir_pluginlib::PluginManager::getPluginsByType(Instance()->heuristics);

  if (Instance()->heuristics.empty())
    ROS_ERROR("[Heuristic] loadPlugins: Couldn't find any heuristic. Fix it immediatly!");
  else
  {
    ROS_INFO("[Heuristic] Plugins loaded:");
    for (std::vector<HeuristicPlugin::Ptr>::const_iterator itr = Instance()->heuristics.begin(); itr != Instance()->heuristics.end(); itr++)
    {
      const HeuristicPlugin::Ptr& heuristic = *itr;
      if (heuristic)
        ROS_INFO("    %s", heuristic->getName().c_str());
    }
  }
}

void Heuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  for (std::vector<HeuristicPlugin::Ptr>::iterator itr = Instance()->heuristics.begin(); itr != Instance()->heuristics.end(); itr++)
  {
    HeuristicPlugin::Ptr& heuristic = *itr;
    if (heuristic)
    {
      heuristic->loadParams(params);
      heuristic->reset();
    }
  }
}

void Heuristic::resetPlugins()
{
  for (std::vector<HeuristicPlugin::Ptr>::iterator itr = Instance()->heuristics.begin(); itr != Instance()->heuristics.end(); itr++)
  {
    HeuristicPlugin::Ptr& heuristic = *itr;
    if (heuristic)
      heuristic->reset();
  }
}

double Heuristic::getHeuristicValue(const State& from, const State& to, const State& start, const State& goal)
{
  double h = 0.0;

  for (std::vector<HeuristicPlugin::Ptr>::const_iterator itr = Instance()->heuristics.begin(); itr != Instance()->heuristics.end(); itr++)
  {
    const HeuristicPlugin::Ptr& heuristic = *itr;
    if (heuristic)
      h += heuristic->getHeuristicValue(from, to, start, goal);
  }

  return h;
}
}
