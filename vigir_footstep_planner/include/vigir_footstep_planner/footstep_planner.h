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

#ifndef FOOTSTEP_PLANNER_H__
#define FOOTSTEP_PLANNER_H__

#include <ros/ros.h>
#include <tf/tf.h>

#include <assert.h>
#include <time.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_msgs/parameter_set.h>

#include <vigir_terrain_classifier/TerrainModelService.h>

#include <vigir_footstep_planning_lib/math.h>
#include <vigir_footstep_planning_lib/parameter_manager.h>
#include <vigir_footstep_planning_lib/modeling/state.h>
#include <vigir_footstep_planning_lib/plugins/plugin_manager.h>
#include <vigir_footstep_planning_lib/plugins/step_plan_msg_plugin.h>

#include <vigir_foot_pose_transformer/foot_pose_transformer.h>

#include <vigir_footstep_planner/environment_parameters.h>
#include <vigir_footstep_planner/footstep_planner_environment.h>
#include <vigir_footstep_planner/state_space/footstep.h>
#include <vigir_footstep_planner/step_cost_estimators/step_cost_estimator.h>
#include <vigir_footstep_planner/heuristics/heuristic.h>

#include <vigir_footstep_planner/post_processor/post_processor.h>

#include <vigir_footstep_planner/robot_model/robot_model.h>
#include <vigir_footstep_planner/world_model/world_model.h>



namespace vigir_footstep_planning
{
typedef std::vector<State>::const_iterator state_iter_t;

/**
 * @brief A class to control the interaction between ROS and the footstep
 * planner.
 */
class FootstepPlanner
{
public:
  typedef boost::function<void (msgs::StepPlanRequestService::Response)> ResultCB;
  typedef boost::function<void (msgs::PlanningFeedback)> FeedbackCB;
  typedef boost::function<void ()> PreemptCB;

  FootstepPlanner(ros::NodeHandle &nh);
  virtual ~FootstepPlanner();

  bool isPlanning() const;

  bool setParams(const ParameterSet& params);

  msgs::ErrorStatus updateFoot(msgs::Foot& foot, uint8_t mode, bool transform = true) const;
  msgs::ErrorStatus updateFeet(msgs::Feet& feet, uint8_t mode, bool transform = true) const;
  msgs::ErrorStatus updateStepPlan(msgs::StepPlan& result, uint8_t mode, const std::string& param_set_name = std::string(), bool transform = true) const;

  msgs::ErrorStatus stepPlanRequest(msgs::StepPlanRequestService::Request& req, ResultCB result_cb = ResultCB(), FeedbackCB feedback_cb = FeedbackCB(), PreemptCB preempt_cb = PreemptCB());
  /// @brief Service handle to plan footsteps.
  bool stepPlanRequestService(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp);

  /// @brief stops thread running planning
  void preemptPlanning();

  // typedefs
  typedef boost::shared_ptr<FootstepPlanner> Ptr;
  typedef boost::shared_ptr<const FootstepPlanner> ConstPtr;

protected:
  /**
   * @brief Start a planning task from scratch (will delete information
   * of previous planning tasks). Map and start, goal poses need to be
   * set beforehand.
   *
   * @return Success of planning.
   */
  msgs::ErrorStatus planSteps(msgs::StepPlanRequestService::Request& req);

  /// @brief plans stepping
  msgs::ErrorStatus planPattern(msgs::StepPlanRequestService::Request& req);
  msgs::ErrorStatus finalizeAndAddStepToPlan(State& s, const msgs::PatternParameters& env_params, bool change_z = true);

  /// @brief extracts step plan response from planning result
  bool finalizeStepPlan(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp, bool post_process);

  /// @brief: starts planning in a new thread
  void startPlanning(msgs::StepPlanRequestService::Request& req);
  /// @brief: method used in seperate thread
  void doPlanning(msgs::StepPlanRequestService::Request& req);

  bool findNearestValidState(State& s) const;

  bool checkRobotCollision(const State& left_foot, const State& right_foot, bool& left, bool& right) const;

  /**
   * @brief Sets the start pose as position of left and right footsteps.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setStart(const State& left_foot, const State& right_foot, bool ignore_collision = false);

  /**
   * @brief Sets the start pose
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setStart(const msgs::StepPlanRequest& req, bool ignore_collision = false);

  /**
   * @brief Sets the goal pose as position of left and right footsteps.
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setGoal(const State& left_foot, const State& right_foot, bool ignore_collision = false);

  /**
   * @brief Sets the goal pose
   *
   * @return True if the two foot poses have been set successfully.
   */
  bool setGoal(const msgs::StepPlanRequest& req, bool ignore_collision = false);

  /// @return Costs of the planned footstep path.
  double getPathCosts() const { return ivPathCost; }

  /// @return Number of expanded states.
  size_t getNumExpandedStates() const
  {
    return ivPlannerPtr->get_n_expands();
  }

  /// @return Number of planned foot poses.
  size_t getNumFootPoses() const { return ivPath.size(); }

  state_iter_t getPathBegin() const { return ivPath.begin(); }
  state_iter_t getPathEnd() const { return ivPath.end(); }

  /// @return Size of the planned path.
  int getPathSize() { return ivPath.size(); }

  State getStartFootLeft() { return ivStartFootLeft; }
  State getStartFootRight() { return ivStartFootRight; }

  /// @brief Reset the previous planning information.
  void reset();

  /// @brief Reset and reinitialize the environment.
  void resetTotally();

  /// @return True if for the current start and goal pose a path exists.
  bool pathExists() { return (bool)ivPath.size(); }

  /**
   * @return True if the newly calculated path is different from the existing
   * one (if one exists).
   */
  bool pathIsNew(const std::vector<int>& new_path);

  /**
   * @brief Extracts the path (list of foot poses) from a list of state
   * IDs calculated by the SBPL.
   */
  bool extractPath(const std::vector<int>& state_ids);

  /**
   * @brief Starts the planning task in the underlying SBPL.
   *
   * NOTE: Never call this directly. Always use either plan() or replan() to
   * invoke this method.
   */
  bool plan(ReplanParams& params);

  /// @brief Returns the foot pose of a leg for a given robot pose.
  State getFootPose(const State& robot, Leg leg, double dx, double dy, double dyaw);
  State getFootPose(const State& robot, Leg leg);

  /// @brief Sets the planning algorithm used by SBPL.
  void setPlanner();

  /// @brief Updates the environment in case of a changed map.
  void updateEnvironment(const vigir_gridmap_2d::GridMap2DPtr old_map);

  // publisher
  ros::Publisher ivCheckedFootContactSupportPub;

  mutable boost::recursive_mutex planner_mutex;
  boost::thread planning_thread;

  ResultCB result_cb;
  FeedbackCB feedback_cb;
  PreemptCB preempt_cb;

  boost::shared_ptr<FootstepPlannerEnvironment> ivPlannerEnvironmentPtr;
  boost::shared_ptr<SBPLPlanner> ivPlannerPtr;

  std::vector<State> ivPath;

  State ivStartFootLeft;
  State ivStartFootRight;
  State ivGoalFootLeft;
  State ivGoalFootRight;

  // local instance of foot pose transformer
  FootPoseTransformer foot_pose_transformer;

  // Parameters
  std::string frame_id;
  EnvironmentParameters::Ptr env_params;
  unsigned int start_foot_selection;
  bool start_pose_set_up, goal_pose_set_up;

  double max_number_steps;

  // robot parameters
  double max_path_length_ratio;

  double ivPathCost;

  std::vector<int> ivPlanningStatesIds;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ivCheckedFootContactSupport;

  // counter to be used as sequence number
  unsigned int step_plan_uid;
};
}

#endif
