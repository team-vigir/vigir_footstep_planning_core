//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_FOOTSTEP_PLANNING_WORLD_MODEL_H__
#define VIGIR_FOOTSTEP_PLANNING_WORLD_MODEL_H__

#include <ros/ros.h>

#include <boost/noncopyable.hpp>

#include <vigir_pluginlib/plugin_manager.h>

#include <vigir_footstep_planning_lib/plugins/collision_check_plugin.h>
#include <vigir_footstep_planning_lib/plugins/terrain_model_plugin.h>

#include <vigir_footstep_planner/world_model/foot_grid_map_model.h>
#include <vigir_footstep_planner/world_model/upper_body_grid_map_model.h>



namespace vigir_footstep_planning
{
class WorldModel
  : boost::noncopyable
{
public:
  static void loadPlugins();
  static void loadParams(const vigir_generic_params::ParameterSet& params);

  // evaluation will be done in alphabetical order of plugin names
  static bool isAccessible(const State& s);
  static bool isAccessible(const State& next, const State& current);

  static void useTerrainModel(bool enabled);
  static bool isTerrainModelAvailable();
  static TerrainModelPlugin::Ptr getTerrainModel();

  /**
  /* @brief update z, roll and pitch of state based on terrain model
   * @return false if terrain model is available but has no data for given state,
   *         otherwise true even if no terrain model is available
   **/
  static bool getHeight(double x, double y, double& height);
  static bool update3DData(State& s);

  // typedefs
  typedef boost::shared_ptr<WorldModel> Ptr;
  typedef boost::shared_ptr<const WorldModel> ConstPtr;

protected:
  WorldModel();

  static WorldModel::Ptr& Instance();

  static WorldModel::Ptr singelton;

  bool use_terrain_model;

  std::vector<CollisionCheckPlugin::Ptr> collision_check_models;
  TerrainModelPlugin::Ptr terrain_model;
};
}

typedef vigir_footstep_planning::WorldModel PlannerWorldModel;

#endif
