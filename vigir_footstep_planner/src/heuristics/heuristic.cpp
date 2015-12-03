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
