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

#ifndef VIGIR_FOOTSTEP_PLANNING_FOOTSTEP_PLANNER_ENVIRONMENT_H__
#define VIGIR_FOOTSTEP_PLANNING_FOOTSTEP_PLANNER_ENVIRONMENT_H__

#include <math.h>

#include <vector>

#include <boost/function.hpp>

#include <sbpl/headers.h>

#include <vigir_foot_pose_transformer/foot_pose_transformer.h>

#include <vigir_footstep_planning_lib/threading/threading_manager.h>
#include <vigir_footstep_planner/threading/expand_state_job.h>

#include <vigir_footstep_planner/environment_parameters.h>
#include <vigir_footstep_planner/state_space/state_space.h>

#include <vigir_pluginlib/plugin_manager.h>

#include <vigir_footstep_planning_lib/helper.h>

#include <vigir_footstep_planner/step_cost_estimator.h>

#include <vigir_footstep_planner/heuristic.h>



namespace vigir_footstep_planning
{
/**
 * @brief A class defining a footstep planner environment for humanoid
 * robots used by the SBPL to perform planning tasks.
 *
 * The environment keeps track of all the planning states expanded during
 * the search. Each planning state can be accessed via its ID. Furthermore
 */
class FootstepPlannerEnvironment
  : public DiscreteSpaceInformation
{
public:
  FootstepPlannerEnvironment(const EnvironmentParameters& params, const FootPoseTransformer& foot_pose_transformer, boost::function<void (msgs::PlanningFeedback)>& feedback_cb);

  virtual ~FootstepPlannerEnvironment();

  void setFrameId(const std::string& frame_id);

  const StateSpace::Ptr& getStateSpace() { return state_space; }

  /// @return The number of expanded states during the search.
  int getNumExpandedStates() { return ivNumExpandedStates; }

  /**
   * @brief Resets the current planning task (i.e. the start and goal
   * poses).
   */
  void reset();

  void stateExpanded(const PlanningState& s);
  void stateVisited(const PlanningState& s);

  exp_states_2d_iter_t getExpandedStatesStart()
  {
    return ivExpandedStates.begin();
  }

  exp_states_2d_iter_t getExpandedStatesEnd()
  {
    return ivExpandedStates.end();
  }

  /**
   * @return The costs (in mm, truncated as int) to reach the
   * planning state ToStateID from within planning state FromStateID.
   */
  int GetFromToHeuristic(int FromStateID, int ToStateID);

  /**
   * @return The heuristic value to reach the goal from within the
   * planning state stateID (used for forward planning).
   */
  int GetGoalHeuristic(int stateID);

  /**
   * @return The heuristic value to reach the start from within
   * the planning state stateID. (Used for backward planning.)
   */
  int GetStartHeuristic(int stateID);

  /**
   * @brief Calculates the successor states and the corresponding costs
   * when performing the footstep set on the planning state SourceStateID.
   * (Used for forward planning.)
   */
  void GetSuccs(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV);

  /**
   * @brief Calculates the predecessor states and the corresponding costs
   * when reversing the footstep set on the planning state TargetStateID.
   * (Used for backward planning.)
   */
  void GetPreds(int TargetStateID, std::vector<int> *PredIDV, std::vector<int> *CostV);

  /**
   * @brief Used for RStar: generate succs/preds at some
   * domain-dependent distance. The number of generated succs/preds is up
   * to the environment.
   */
  virtual void GetRandomSuccsatDistance(int SourceStateID,
                                        std::vector<int>* SuccIDV,
                                        std::vector<int>* CLowV);

  /**
   * @brief Used for RStar: generate succs/preds at some
   * domain-dependent distance. The number of generated succs/preds is up
   * to the environment.
   */
  virtual void GetRandomPredsatDistance(int TargetStateID,
                                        std::vector<int>* PredIDV,
                                        std::vector<int>* CLowV);

  /// @return True if two states meet the same condition. Used for R*.
  bool AreEquivalent(int StateID1, int StateID2);

  bool InitializeEnv(const char *sEnvFile);

  bool InitializeMDPCfg(MDPConfig *MDPCfg);

  void PrintEnv_Config(FILE *fOut);

  void PrintState(int stateID, bool bVerbose, FILE *fOut);

  void SetAllActionsandAllOutcomes(CMDPSTATE *state);

  void SetAllPreds(CMDPSTATE *state);

  int SizeofCreatedEnv();

  /**
   * @brief Update the heuristic values (e.g. after the map has changed).
   * The environment takes care that the update is only done when it is
   * necessary.
   */
  void updateHeuristicValues();

protected:
  /**
   * @return The costs (in mm, truncated as int) to reach the
   * planning state ToStateID from within planning state FromStateID.
   */
  int GetFromToHeuristic(const PlanningState& from, const PlanningState& to, const PlanningState& start, const PlanningState& goal);

  /// < operator for planning states.
  struct less
  {
    bool operator()(const PlanningState* a, const PlanningState* b) const;
  };

  StateSpace::Ptr state_space;

  // local instance of foot pose transformer
  const FootPoseTransformer& foot_pose_transformer;

  // feedback callback
  boost::function<void (msgs::PlanningFeedback)>& feedback_cb;

  /// Parameters
  const EnvironmentParameters& params;

  exp_states_2d_t ivExpandedStates;
  size_t ivNumExpandedStates;

  std::string frame_id;
  std::vector<msgs::Step> visited_steps;
  ros::Time last_feedback_flush;

  threading::ThreadingManager<threading::ExpandStateJob>::Ptr expand_states_manager;
};
}

#endif
