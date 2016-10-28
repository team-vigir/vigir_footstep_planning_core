//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
// Based on http://wiki.ros.org/footstep_planner by Johannes Garimort and Armin Hornung
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

#ifndef VIGIR_FOOTSTEP_PLANNING_GRID_MAP_MODEL_H__
#define VIGIR_FOOTSTEP_PLANNING_GRID_MAP_MODEL_H__

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include <vigir_footstep_planning_plugins/plugins/collision_check_grid_map_plugin.h>

#include <vigir_footstep_planning_default_plugins/world_model/grid_map_2d.h>



namespace vigir_footstep_planning
{
class GridMapModel
  : public CollisionCheckGridMapPlugin
{
public:
  GridMapModel(const std::string& name);

  bool loadParams(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) override;

protected:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map_);

  /**
   * @brief Checks if a footstep (represented by its center and orientation)
   * collides with an obstacle. The check is done by recursively testing if
   * either the circumcircle of the foot, the inner circle of the foot or the
   * area in between has an appropriate distance to the nearest obstacle.
   *
   * @param x Global position of the foot in x direction.
   * @param y Global position of the foot in y direction.
   * @param theta Global orientation of the foot.
   * @param height Size of the foot in x direction.
   * @param width Size of the foot in y direction.
   * obstacle.
   *
   * @return True if the footstep collides with an obstacle.
   */
  bool collision_check(double x, double y, double cos_theta, double sin_theta, double height, double width) const;



  vigir_gridmap_2d::GridMap2D distance_map;

 /**
  * accuracy: (0) circumcircle of the foot; (1) incircle of the foot;
  * (2) circumcircle and incircle recursivly checked for the whole foot
  */
  int collision_check_accuracy;
};
}

#endif
