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
