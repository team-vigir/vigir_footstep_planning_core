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

#ifndef VIGIR_FOOTSTEP_PLANNING_STEP_COST_KEY_H
#define VIGIR_FOOTSTEP_PLANNING_STEP_COST_KEY_H

#include <boost/unordered_set.hpp>
#include <boost/functional/hash.hpp>

#include <tf/tf.h>

#include <vigir_footstep_planning_lib/modeling/state.h>

#define STATE_DIM 6



namespace vigir_footstep_planning
{
class StepCostKey
  : public std::vector<int>
{
public:
  StepCostKey(const std::vector<double> &state, double cell_size, double angle_bin_size);
  StepCostKey(const State &left_foot, const State &right_foot, const State &swing_foot, double cell_size, double angle_bin_size);
  StepCostKey(const StepCostKey &other);

  bool operator== (const StepCostKey &other) const;
  bool operator< (const StepCostKey &other) const;

  void setState(const std::vector<double> &state);
  void getState(std::vector<double> &state) const;

protected:
  void transformStates(const State &left_foot, const State &right_foot, const State &swing_foot, std::vector<double> &state) const;
  void mirrorPoseOnXPlane(tf::Pose &mirror, const tf::Pose &orig) const;

  /// @brief continuous to discrete
  inline int cont_2_disc(double value, double cell_size) const
  {
    return int(round(value / cell_size));
  }

  /// @brief discrete to continuous
  inline double disc_2_cont(int value, double cell_size) const
  {
    return double(value) * cell_size;
  }

  const double cell_size, angle_bin_size;
};
}

std::size_t hash_value(const vigir_footstep_planning::StepCostKey &key);

#endif
