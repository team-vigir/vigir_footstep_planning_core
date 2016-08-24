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

#ifndef VIGIR_FOOTSTEP_PLANNING_LIB_STEP_DYNAMICS_POST_PROCESS_H__
#define VIGIR_FOOTSTEP_PLANNING_LIB_STEP_DYNAMICS_POST_PROCESS_H__

#include <ros/ros.h>

#include <vigir_footstep_planning_plugins/plugins/post_process_plugin.h>



namespace vigir_footstep_planning
{
class StepDynamicsPostProcess
  : public PostProcessPlugin
{
public:
  StepDynamicsPostProcess();

  bool loadParams(const vigir_generic_params::ParameterSet& global_params = vigir_generic_params::ParameterSet()) override;

  void postProcessStepForward(const State& left_foot, const State& right_foot, State& swing_foot) const override;
  void postProcessStepBackward(const State& left_foot, const State& right_foot, State& swing_foot) const override;

  // typedefs
  typedef boost::shared_ptr<StepDynamicsPostProcess> Ptr;
  typedef boost::shared_ptr<const StepDynamicsPostProcess> ConstPtr;

protected:
  // following methods represents single processing steps which will be execute in the given order
  virtual void determineStepAttributes(const State& swing_foot_before, const State& stand_foot, State& swing_foot_after) const;
  virtual void determineTravelDistance(const State& swing_foot_before, const State& stand_foot, State& swing_foot_after) const;
  virtual void determineTimings(const State& swing_foot_before, const State& stand_foot, State& swing_foot_after) const;
  virtual void determineDynamics(const State& swing_foot_before, const State& stand_foot, State& swing_foot_after) const;

  double default_swing_height;
  double default_sway_duration;
  double default_step_duration;
};
}

#endif
