//=================================================================================================
// Copyright (c) 2017, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_FOOTSTEP_PLANNING_TERRAIN_MODEL_H__
#define VIGIR_FOOTSTEP_PLANNING_TERRAIN_MODEL_H__

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <pcl/point_cloud.h>

#include <vigir_footstep_planning_lib/modeling/state.h>

#include <vigir_footstep_planning_plugins/plugins/terrain_model_plugin.h>

#include <vigir_terrain_classifier/terrain_model.h>



namespace vigir_footstep_planning
{
class TerrainModel
  : public TerrainModelPlugin
{
public:
  TerrainModel(const std::string& name = "terrain_model");

  bool initialize(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) override;

  bool loadParams(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) override;

  void reset();

  bool isAccessible(const State& s) const override;
  bool isAccessible(const State& next, const State& current) const override;

  bool isTerrainModelAvailable() const override;

  void setTerrainModel(const vigir_terrain_classifier::TerrainModelMsg::ConstPtr& terrain_model);

  double getResolution() const override;

  bool getPointWithNormal(const pcl::PointNormal& p_search, pcl::PointNormal& p_result) const override;
  bool getHeight(double x, double y, double& height) const override;
  bool getFootContactSupport(const geometry_msgs::Pose& p, double& support, pcl::PointCloud<pcl::PointXYZI>::Ptr checked_positions = pcl::PointCloud<pcl::PointXYZI>::Ptr()) const override;
  bool getFootContactSupport(const tf::Pose& p, double& support, pcl::PointCloud<pcl::PointXYZI>::Ptr checked_positions = pcl::PointCloud<pcl::PointXYZI>::Ptr()) const;

  bool update3DData(geometry_msgs::Pose& p) const override;
  bool update3DData(State& s) const override;

  // typedefs
  typedef boost::shared_ptr<TerrainModel> Ptr;
  typedef boost::shared_ptr<TerrainModel> ConstPtr;

protected:
  bool getFootContactSupport(const tf::Pose& p, double &support, unsigned int sampling_steps_x, unsigned int sampling_steps_y, pcl::PointCloud<pcl::PointXYZI>::Ptr checked_positions = pcl::PointCloud<pcl::PointXYZI>::Ptr()) const;

  // subscribers
  ros::Subscriber terrain_model_sub;

  // mutex
  mutable boost::shared_mutex terrain_model_shared_mutex;

  vigir_terrain_classifier::TerrainModel::Ptr terrain_model;

  // ground contact support estimation parameters
  unsigned int min_sampling_steps_x;  // min number of sampling steps in y
  unsigned int min_sampling_steps_y;  // min number of sampling steps in y
  unsigned int max_sampling_steps_x;  // max number of sampling steps in y
  unsigned int max_sampling_steps_y;  // max number of sampling steps in y
  double max_intrusion_z;             // how deep the foot may intrude into other objects (z axis only)#
  double max_ground_clearance;        // maximal distance before a point is treated as "in the air"
  double minimal_support;             // minimal percentage of foot sole area which must touch ground

  // robot params
  geometry_msgs::Vector3 foot_size;
};
}

#endif
