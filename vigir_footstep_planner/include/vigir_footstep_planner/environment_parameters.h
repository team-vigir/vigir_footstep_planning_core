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

#ifndef ENVIRONMENT_PARAMETERS_H__
#define ENVIRONMENT_PARAMETERS_H__

#include <ros/ros.h>

#include <vigir_footstep_planning_lib/math.h>
#include <vigir_footstep_planning_lib/parameter_manager.h>

#include <vigir_footstep_planner/state_space/footstep.h>



namespace vigir_footstep_planning
{
struct EnvironmentParameters
{
  EnvironmentParameters(const ParameterSet& params);
  virtual ~EnvironmentParameters();

  // typedefs
  typedef boost::shared_ptr<EnvironmentParameters> Ptr;
  typedef boost::shared_ptr<EnvironmentParameters> ConstPtr;

  // parameters
  std::vector<Footstep> footstep_set;

  /// Defines the area of performable (discrete) steps.
  std::vector<std::pair<int, int> > step_range;

  // foot paramaters
  geometry_msgs::Vector3 foot_size;
  double foot_seperation;

  // upper body parameters
  geometry_msgs::Vector3 upper_body_size;
  geometry_msgs::Vector3 upper_body_origin_shift;

  double max_risk;

  double max_step_range_x, max_step_range_y, max_step_range_theta;
  double max_inverse_step_range_x, max_inverse_step_range_y, max_inverse_step_range_theta;
  double max_step_range_width;        // maximal step distance based on step polygon
  double max_step_dist;               // maximal distance in footstep primitives

  int    hash_table_size;             // Size of the hash table storing the planning states expanded during the search. (Also referred to by max_hash_size.)
  double cell_size;                   // The size of each grid cell used to discretize the robot positions.
  int    num_angle_bins;              // The number of bins used to discretize the robot orientations.
  double angle_bin_size;
  bool   forward_search;              // Whether to use forward search (1) or backward search (0).
  int    num_random_nodes;            // number of random neighbors for R*
  double random_node_distance;

  double heuristic_scale;             // Scaling factor of heuristic, in case it underestimates by a constant factor.



  /// The maximal translation in x, y direction (based on step range polygon and discretized in cell size).
  int ivMaxStepRangeX;
  int ivMaxStepRangeY;
  /// The maximal rotation (discretized into bins).
  int ivMaxStepRangeTheta;

  /// The minimal translation in x, y direction (based on step range polygon and discretized in cell size).
  int ivMaxInvStepRangeX;
  int ivMaxInvStepRangeY;
  /// The minimal rotation (discretized into bins).
  int ivMaxInvStepRangeTheta;

  /// The maximal step range based on given step range polygon
  double ivMaxStepRangeWidth;

  /// The global max step distance based on the step range polygon and footstep set
  double ivMaxStepDist;

  bool   ivSearchUntilFirstSolution;

  /// default max planning time if not given by request
  double max_planning_time;
  double initial_eps;
  double decrease_eps;

  /**
   * @brief If limit of changed cells is reached the planner starts a new
   * task from the scratch.
   */
  int ivChangedCellsLimit;

  std::string ivPlannerType;

  double feedback_rate;
  int threads;
  unsigned int jobs_per_thread;
};
}

#endif
