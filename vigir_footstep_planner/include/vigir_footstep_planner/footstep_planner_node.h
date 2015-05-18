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

#ifndef FOOTSTEP_PLANNER_NODE_H__
#define FOOTSTEP_PLANNER_NODE_H__

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_msgs/visualization.h>

#include <vigir_footstep_planning_lib/helper.h>
#include <vigir_footstep_planning_lib/visualization/footstep_planning_vis.h>
#include <vigir_footstep_planning_lib/plugins/robot_model_plugin.h>
#include <vigir_footstep_planning_lib/plugins/step_plan_msg_plugin.h>

#include <vigir_footstep_planner/robot_model/dynamics_reachability.h>
#include <vigir_footstep_planner/robot_model/reachability_polygon.h>

#include <vigir_footstep_planner/post_processor/step_dynamics_post_process.h>

#include <vigir_footstep_planner/world_model/foot_grid_map_model.h>
#include <vigir_footstep_planner/world_model/upper_body_grid_map_model.h>
#include <vigir_footstep_planner/world_model/terrain_model.h>

#include <vigir_footstep_planner/step_cost_estimators/const_step_cost_estimator.h>
#include <vigir_footstep_planner/step_cost_estimators/euclidean_step_cost_estimator.h>
#include <vigir_footstep_planner/step_cost_estimators/boundary_step_cost_estimator.h>
#include <vigir_footstep_planner/step_cost_estimators/dynamics_step_cost_estimator.h>
#include <vigir_footstep_planner/step_cost_estimators/ground_contact_step_cost_estimator.h>
#include <vigir_footstep_planner/step_cost_estimators/travel_time_step_cost_estimator.h>

#include <vigir_footstep_planner/heuristics/dynamics_heuristic.h>
#include <vigir_footstep_planner/heuristics/euclidean_heuristic.h>
#include <vigir_footstep_planner/heuristics/hot_map_heuristic.h>
#include <vigir_footstep_planner/heuristics/step_cost_heuristic.h>
#include <vigir_footstep_planner/heuristics/occupancy_grid_map_heuristic.h>
#include <vigir_footstep_planner/heuristics/travel_time_heuristic.h>

#include <vigir_footstep_planner/footstep_planner.h>


namespace vigir_footstep_planning
{
/**
 * @brief Wrapper class for FootstepPlanner, providing callbacks for
 * the node functionality.
 */
class FootstepPlannerNode
{
public:
  FootstepPlannerNode();
  virtual ~FootstepPlannerNode();

  virtual void initPlugins(ros::NodeHandle& nh);
  virtual void init(ros::NodeHandle& nh);

protected:
  // callbacks
  void planningResultCallback(const msgs::StepPlanRequestService::Response& resp);
  void planningResultActionCallback(const msgs::StepPlanRequestService::Response& resp, SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as);

  void planningFeedbackCallback(const msgs::PlanningFeedback& feedback);
  void planningFeedbackActionCallback(const msgs::PlanningFeedback& feedback, SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as);

  void planningPreemptionActionCallback(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as);

  // subscriber
  void setParams(const msgs::ParameterSetConstPtr& params);
  void setParams(const std_msgs::StringConstPtr& params_name);
  void stepPlanRequest(const msgs::StepPlanRequestConstPtr& plan_request);
  void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose);

  // service calls
  bool setParamsService(msgs::SetParameterSetService::Request& req, msgs::SetParameterSetService::Response& resp);
  bool stepPlanRequestService(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp);
  bool updateFootService(msgs::UpdateFootService::Request& req, msgs::UpdateFootService::Response& resp);
  bool updateFeetService(msgs::UpdateFeetService::Request& req, msgs::UpdateFeetService::Response& resp);
  bool updateStepPlanService(msgs::UpdateStepPlanService::Request& req, msgs::UpdateStepPlanService::Response& resp);

  // action server calls
  void setParameterSetAction(SimpleActionServer<msgs::SetParameterSetAction>::Ptr& as);
  void stepPlanRequestAction(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as);
  void stepPlanRequestPreempt(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as);
  void updateFootAction(SimpleActionServer<msgs::UpdateFootAction>::Ptr& as);
  void updateFeetAction(SimpleActionServer<msgs::UpdateFeetAction>::Ptr& as);
  void updateStepPlanAction(SimpleActionServer<msgs::UpdateStepPlanAction>::Ptr& as);

  // subscribers
  ros::Subscriber set_parameter_set_sub;
  ros::Subscriber set_active_parameter_set_sub;
  ros::Subscriber step_plan_request_sub;
  ros::Subscriber goal_pose_sub;

  // publisher
  ros::Publisher step_plan_pub;
  ros::Publisher step_plan_request_vis_pub;
  ros::Publisher step_plan_vis_pub;
  ros::Publisher error_status_pub;
  ros::Publisher temp_step_plan_pub;
  ros::Publisher feedback_pub;

  // service clients
  ros::ServiceClient generate_feet_pose_client;

  // service servers
  ros::ServiceServer set_parameter_set_srv;
  ros::ServiceServer step_plan_request_srv;
  ros::ServiceServer update_foot_srv;
  ros::ServiceServer update_feet_srv;
  ros::ServiceServer update_step_plan_srv;

  // action servers
  SimpleActionServer<msgs::StepPlanRequestAction>::Ptr step_plan_request_as;
  SimpleActionServer<msgs::UpdateFootAction>::Ptr update_foot_as;
  SimpleActionServer<msgs::UpdateFeetAction>::Ptr update_feet_as;
  SimpleActionServer<msgs::UpdateStepPlanAction>::Ptr update_step_plan_as;

  mutable boost::recursive_mutex step_plan_request_as_mutex;

  FootstepPlanner::Ptr footstep_planner;
  geometry_msgs::Vector3 foot_size;
};
}
#endif
