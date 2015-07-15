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
#include <vigir_footstep_planner/world_model/world_model.h>

namespace vigir_footstep_planning
{
WorldModel::Ptr WorldModel::singelton = WorldModel::Ptr();

WorldModel::WorldModel()
{
}

WorldModel::Ptr& WorldModel::Instance()
{
  if (!singelton)
    singelton.reset(new WorldModel());
  return singelton;
}

void WorldModel::loadPlugins()
{
  // get collision check plugins
  PluginManager::getPluginsByType(Instance()->collision_check_models);

  ROS_INFO("[WorldModel] Plugins loaded:");
  for (std::vector<CollisionCheckPlugin::Ptr>::const_iterator itr = Instance()->collision_check_models.begin(); itr != Instance()->collision_check_models.end(); itr++)
  {
    const CollisionCheckPlugin::Ptr& collision_check_model = *itr;
    if (collision_check_model)
      ROS_INFO("    %s (%s)", collision_check_model->getName().c_str(), collision_check_model->getTypeId().c_str());
  }

  // get terrain model
  PluginManager::getPlugin(Instance()->terrain_model);
  if (Instance()->terrain_model)
  {
    ROS_INFO("[WorldModel] Found terrain model:");
    ROS_INFO("    %s (%s)", Instance()->terrain_model->getName().c_str(), Instance()->terrain_model->getTypeId().c_str());
  }
}

void WorldModel::loadParams(const ParameterSet& params)
{
  for (std::vector<CollisionCheckPlugin::Ptr>::iterator itr = Instance()->collision_check_models.begin(); itr != Instance()->collision_check_models.end(); itr++)
  {
    CollisionCheckPlugin::Ptr& collision_check_model = *itr;
    if (collision_check_model)
      collision_check_model->loadParams(params);
  }
}

bool WorldModel::isAccessible(const State& s)
{
  for (std::vector<CollisionCheckPlugin::Ptr>::const_iterator itr = Instance()->collision_check_models.begin(); itr != Instance()->collision_check_models.end(); itr++)
  {
    const CollisionCheckPlugin::Ptr& collision_check_model = *itr;
    if (collision_check_model && collision_check_model->isCollisionCheckAvailable() && !collision_check_model->isAccessible(s))
      return false;
  }
  return true;
}

bool WorldModel::isAccessible(const State& next, const State& current)
{
  for (std::vector<CollisionCheckPlugin::Ptr>::const_iterator itr = Instance()->collision_check_models.begin(); itr != Instance()->collision_check_models.end(); itr++)
  {
    const CollisionCheckPlugin::Ptr& collision_check_model = *itr;
    if (collision_check_model && collision_check_model->isCollisionCheckAvailable() && !collision_check_model->isAccessible(next, current))
      return false;
  }
  return true;
}

void WorldModel::useTerrainModel(bool enabled)
{
  Instance()->use_terrain_model = enabled;
}

bool WorldModel::isTerrainModelAvailable()
{
  return Instance()->terrain_model && Instance()->terrain_model->isTerrainModelAvailable();
}

TerrainModelPlugin::Ptr WorldModel::getTerrainModel()
{
  return Instance()->terrain_model;
}

bool WorldModel::getHeight(double x, double y, double& height)
{
  if (!Instance()->use_terrain_model || !isTerrainModelAvailable())
    return true;

  return Instance()->terrain_model->getHeight(x, y, height);
}

bool WorldModel::update3DData(State& s)
{
  if (!Instance()->use_terrain_model || !isTerrainModelAvailable())
    return true;

  return Instance()->terrain_model->update3DData(s);
}
}
