//=================================================================================================
// Copyright (c) 2018, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_FOOTSTEP_PLANNING_HOT_MAP_HEURISTIC_H__
#define VIGIR_FOOTSTEP_PLANNING_HOT_MAP_HEURISTIC_H__

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <nav_msgs/OccupancyGrid.h>

#include <vigir_footstep_planning_lib/math.h>

#include <vigir_footstep_planning_plugins/plugins/heuristic_plugin.h>



namespace vigir_footstep_planning
{
class HotMapHeuristic
  : public HeuristicPlugin
{
struct StateKey
{
  StateKey(const State& s, double cell_size, double angle_bin_size)
    : x(state_2_cell(s.getX(), cell_size))
    , y(state_2_cell(s.getY(), cell_size))
    , yaw(angle_state_2_cell(s.getYaw(), angle_bin_size))
  {
  }

  bool operator<(const StateKey& key) const
  {
    if (x < key.x) return true;
    if (x > key.x) return false;
    if (y < key.y) return true;
    if (y > key.y) return false;
    if (yaw < key.yaw) return true;
    if (yaw > key.yaw) return false;
    return false;
  }

  int x;
  int y;
  int yaw;
};

public:
  HotMapHeuristic();

  bool loadParams(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) override;

  bool initialize(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) override;

  void reset();

  double getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const override;

  // typedefs
  typedef std::map<StateKey, unsigned int> HotMap;

protected:
  void publishHotMap(const ros::TimerEvent& publish_timer) const;

  // publisher
  ros::Publisher hot_map_pub_;

  ros::Timer publish_timer_;

  // mutex
  mutable boost::shared_mutex hot_map_shared_mutex_;

  mutable HotMap hot_map_;
  mutable unsigned int total_hit_counter_;

  double cell_size_;
  double angle_bin_size_;
};
}

#endif
