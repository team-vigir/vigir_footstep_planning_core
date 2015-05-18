//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_FOOTSTEP_PLANNING_STATE_SPACE_H__
#define VIGIR_FOOTSTEP_PLANNING_STATE_SPACE_H__

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <tr1/unordered_set>
#include <tr1/hashtable.h>

#include <sbpl/headers.h>

#include <vigir_footstep_planning_lib/math.h>
#include <vigir_footstep_planning_lib/modeling/state.h>

#include <vigir_footstep_planner/environment_parameters.h>
#include <vigir_footstep_planner/state_space/footstep.h>
#include <vigir_footstep_planner/state_space/planning_state.h>
#include <vigir_footstep_planner/world_model/terrain_model.h>
#include <vigir_footstep_planner/world_model/grid_map_2d.h>

#include <vigir_footstep_planner/robot_model/robot_model.h>
#include <vigir_footstep_planner/post_processor/post_processor.h>
#include <vigir_footstep_planner/step_cost_estimators/step_cost_estimator.h>
#include <vigir_footstep_planner/heuristics/heuristic.h>



namespace vigir_footstep_planning
{
// specialization of hash<int,int>, similar to standard boost::hash on pairs?
struct IntPairHash{
public:
  size_t operator()(std::pair<int, int> x) const throw() {
    size_t seed = std::tr1::hash<int>()(x.first);
    return std::tr1::hash<int>()(x.second) + 0x9e3779b9 + (seed<<6) + (seed>>2);
  }
};

struct StepCostPair {
  StepCostPair(const PlanningState* state, const int cost)
    : state(state)
    , cost(cost)
  {}

  bool operator < (const StepCostPair& other)
  {
    return cost < other.cost;
  }

  const PlanningState* state;
  const int cost;
};

typedef std::vector<int> exp_states_t;
typedef exp_states_t::const_iterator exp_states_iter_t;
typedef std::tr1::unordered_set<std::pair<int,int>, IntPairHash > exp_states_2d_t;
typedef exp_states_2d_t::const_iterator exp_states_2d_iter_t;



class StateSpace
{
public:
  StateSpace(const EnvironmentParameters& params, std::vector<int*>& state_ID2_index_mapping);

  virtual ~StateSpace();

  void reset();

  // typedefs
  typedef boost::shared_ptr<StateSpace> Ptr;
  typedef boost::shared_ptr<StateSpace> ConstPtr;

  void setFrameId(const std::string& frame_id);

  std::pair<int, int> updateGoal(const State& foot_left, const State& foot_right);
  std::pair<int, int> updateStart(const State& foot_left, const State& right_right);
  void setPlannerStartAndGoal(unsigned int start_foot_selection);

  /**
   * @brief Try to receive a state with a certain ID.
   *
   * @return True iff there is a state with such an ID.
   */
  bool getState(unsigned int id, State &s) const;

  bool getStartState(State &left, State &right) const;
  bool getStartState(State &robot) const;
  bool getGoalState(State &left, State &right) const;
  bool getGoalState(State &robot) const;

  exp_states_iter_t getRandomStatesStart()
  {
    return ivRandomStates.begin();
  }

  exp_states_iter_t getRandomStatesEnd()
  {
    return ivRandomStates.end();
  }


  /// Wrapper for footstep_planner_environment::createNewHashEntry(PlanningState).
  PlanningState* createNewHashEntry(const State& s);

  /**
   * @brief Creates a new planning state for 's' and inserts it into the
   * maps (PlanningState::ivStateId2State,
   * PlanningState::ivpStateHash2State)
   *
   * @return A pointer to the newly created PlanningState.
   */
  PlanningState *createNewHashEntry(const PlanningState& s);

  /// Wrapper for footstep_planner_environment::getHashEntry(PlanningState).
  PlanningState* getHashEntry(const State& s);

  /**
   * @return The pointer to the planning state 's' stored in
   * footstep_planner_environment::ivpStateHash2State.
   */
  PlanningState* getHashEntry(const PlanningState& s);

  PlanningState *createHashEntryIfNotExists(const PlanningState& s);


  /**
   * @return True iff 'swing_foot_after' can be reached by an arbitrary footstep that
   * can be performed by the robot from within 'stand_foot'. This check is based
   * on given step range polygon in config. (This method is used to check
   * whether the goal/start can be reached from within the current state.)
   */
  bool reachable(const State& stand_foot, const State& State) const;

  /**
   * @return True iff 'goal' can be reached by an arbitrary footstep.
   * (Used for forward planning.)
   */
  bool closeToGoal(const PlanningState& from) const;

  /**
   * @return True iff 'start' can be reached by an arbitrary footstep.
   * (Used for backward planning.)
   */
  bool closeToStart(const PlanningState& from) const;


  /// @return computes step cost based on the swing_foot from 'before' to 'after' while standing on stand_foot
  bool getStepCost(const State& stand_foot, const State& swing_foot_before, const State& swing_foot_after, double& cost, double& risk) const;
  bool getStepCost(const State& stand_foot, const State& swing_foot_before, const State& swing_foot_after, int& cost, int& risk) const;
  bool getStepCost(const State& stand_foot, const State& swing_foot_before, const State& swing_foot_after, int& cost) const;

  const EnvironmentParameters& params;
  std::string frame_id;

  /// ID of Start and Goal foot chosen by planner
  int ivIdPlanningStart;
  int ivIdPlanningGoal;

  PlanningState* start_foot_left;
  PlanningState* start_foot_right;

  int ivIdStartFootLeft;
  int ivIdStartFootRight;

  PlanningState* goal_foot_left;
  PlanningState* goal_foot_right;

  int ivIdGoalFootLeft;
  int ivIdGoalFootRight;

  std::vector<int> ivStateArea;

  std::vector<int*>& state_ID2_index_mapping;

  /**
   * @brief Maps from an ID to the corresponding PlanningState. (Used in
   * the SBPL to access a certain PlanningState.)
   */
  std::vector<const PlanningState*> ivStateId2State;

  /**
   * @brief Maps from a hash tag to a list of corresponding planning
   * states. (Used in footstep_planner_environment to identify a certain
   * PlanningState.)
   */
  std::vector<PlanningState*>* ivpStateHash2State;

  /// The set of footsteps used.
  std::vector<Footstep> ivContFootstepSet;

  /// The heuristic function used by the planner.
  bool ivHeuristicExpired;

  /// distance of random neighbors for R* (discretized in cells)
  const int ivRandomNodeDist;

  exp_states_t ivRandomStates;  ///< random intermediate states for R*

  mutable boost::shared_mutex hash_table_shared_mutex;
};
}

#endif
