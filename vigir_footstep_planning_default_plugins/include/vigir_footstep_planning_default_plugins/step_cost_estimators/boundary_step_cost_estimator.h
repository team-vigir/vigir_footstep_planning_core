//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_FOOTSTEP_PLANNING_BOUNDARY_STEP_COST_ESTIMATOR_H
#define VIGIR_FOOTSTEP_PLANNING_BOUNDARY_STEP_COST_ESTIMATOR_H

#include <vigir_footstep_planning_plugins/step_cost_estimator_plugin.h>



namespace vigir_footstep_planning
{
class BoundaryStepCostEstimator
  : public StepCostEstimatorPlugin
{
public:
  BoundaryStepCostEstimator();

  bool loadParams(const vigir_generic_params::ParameterSet& global_params = vigir_generic_params::ParameterSet()) override;

  bool getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const override;

protected:
  double max_diff_z;                      // maximum amount for step up/down
  double long_step_dist;                  // all larger distances are treated as long distance [m]
  double min_yaw_seperation_enlargement;  // min yaw before seperation must be increased
  double yaw_enlarged_min_seperation;     // min separation needed when foot turned in
  double cost_roll_abs;                   // cost for roll angle (absolute) [1/rad]
  double cost_pitch_abs;                  // cost for pitch angle (absolute) [1/rad]
  double cost_yaw_rel;                    // cost for yaw angle (relative to previous foot) [1/rad]
  double cost_height_diff_rel;            // cost for height diff (relative to previous foot) [1/m]
};
}

#endif
