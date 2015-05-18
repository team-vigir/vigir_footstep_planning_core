// SVN $HeadURL: http://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/include/footstep_planner/planning_state.h $
// SVN $Id: planning_state.h 3298 2012-09-28 11:37:38Z hornunga@informatik.uni-freiburg.de $

/*
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FOOTSTEP_PLANNER_PLANNINGSTATE_H_
#define FOOTSTEP_PLANNER_PLANNINGSTATE_H_

#include <vigir_footstep_planning_lib/math.h>
#include <vigir_footstep_planning_lib/modeling/state.h>



namespace vigir_footstep_planning
{
/**
 * @brief A class representing the robot's pose (i.e. position and
 * orientation) in the underlying SBPL. More precisely a planning state
 * is a discrete representation of the robot's supporting leg.
 *
 * Since SBPL is working on discretized states the planning states are also
 * discretized positions and orientations. This is done by fitting the
 * positions into a grid and the orientations into bins.
 * (NOTE: the resolution of the planning cells is likely to differ from the
 * resolution of the grid map.)
 *
 * The SBPL can access each planning state via an unique ID. Furthermore
 * each planning state can be identified by an (ununique) hash tag generated
 * from its position, location and supporting leg.
 */
class PlanningState
{
public:
  /**
   * @brief x, y and theta represent the global (continuous) position and
   * orientation of the robot's support leg.
   *
   * @param leg The supporting leg.
   * @param cell_size The size of each grid cell discretizing the
   * position.
   * @param num_angle_bins The number of bins discretizing the
   * orientation.
   * @param max_hash_size
   */
  PlanningState(double x, double y, double z, double roll, double pitch, double yaw,
                Leg leg, double cell_size, double angle_bin_size, int max_hash_size, const PlanningState *pred_state = nullptr, const PlanningState *succ_state = nullptr);

  /**
   * @brief x, y and theta as discrete bin values (as used internally by
   * the planner).
   */
  PlanningState(int x, int y, double z, double roll, double pitch, int yaw,
                Leg leg, double cell_size, double angle_bin_size, int max_hash_size, const PlanningState *pred_state = nullptr, const PlanningState *succ_state = nullptr);

  PlanningState(const geometry_msgs::Pose& pose, Leg leg, double cell_size, double angle_bin_size, int max_hash_size, const PlanningState *pred_state = nullptr, const PlanningState *succ_state = nullptr);

  /// Create a (discrete) PlanningState from a (continuous) State.
  PlanningState(const State& s, double cell_size, double angle_bin_size, int max_hash_size, const PlanningState *pred_state = nullptr, const PlanningState *succ_state = nullptr);

  /// Copy constructor.
  PlanningState(const PlanningState& s);

  ~PlanningState();

  /**
   * @brief Compare two states on equality of x, y, theta, leg. Makes
   * first use of the non-unique hash tag to rule out unequal states.
   */
  bool operator==(const PlanningState& s2) const;

  /**
   * @brief Compare two states on inequality of x, y, theta, leg by
   * comparing the hash tags of the states.
   */
  bool operator!=(const PlanningState& s2) const;

  /**
   * @brief Used to attach such an unique ID to the planning state. (This
   * cannot be done in the constructor since often such an ID is not known
   * when the planning state is created.)
   */
  void setId(unsigned int id) { ivId = id; }

  int getX() const { return ivX; }
  int getY() const { return ivY; }
  int getYaw() const { return ivYaw; }

  double getSwingHeight() const { return ivState.getSwingHeight(); }

  double getSwayDuration() const { return ivState.getSwayDuration(); }
  double getStepDuration() const { return ivState.getStepDuration(); }

  Leg getLeg() const { return ivState.getLeg(); }

  void setPredState(const PlanningState* pred_state) { ivpPredState = pred_state; }
  const PlanningState* getPredState() const { return ivpPredState; }

  void setSuccState(const PlanningState* succ_state) { ivpSuccState = succ_state; }
  const PlanningState* getSuccState() const { return ivpSuccState; }

  /**
   * @return The (non-unique) hash tag used to identify the planning
   * state.
   */
  unsigned int getHashTag() const { return ivHashTag; }

  /**
   * @return The (unique) ID used within the SBPL to access the
   * planning state.
   */
  int getId() const { return ivId; }

  /// @brief gets the continuous State the PlanningState represents.
  const State& getState() const;
  State& getState();

private:
  /// pointer to original state
  State ivState;

  /// Value of the grid cell the position's x value is fitted into.
  int ivX;
  /// Value of the grid cell the position's y value is fitted into.
  int ivY;
  /// The robot's orientation.
  int ivYaw;

  const PlanningState *ivpPredState;
  const PlanningState *ivpSuccState;

  /// The (unique) ID of the planning state.
  int ivId;

  /**
   * The (non-unique) hash tag of the planning state. Different hash tags
   * imply that the states differ in x, y, theta, leg.
   */
  unsigned int ivHashTag;
};
}
#endif  // FOOTSTEP_PLANNER_PLANNINGSTATE_H_
