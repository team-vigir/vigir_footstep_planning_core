//=================================================================================================
// Copyright (c) 2017, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_PATTERN_GENERATOR_H__
#define VIGIR_PATTERN_GENERATOR_H__

#include <map>

#include <ros/ros.h>
#include <tf/tf.h>

#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include <vigir_footstep_planning_lib/helper.h>
#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#include <vigir_pattern_generator/joystick_handler.h>



namespace vigir_footstep_planning
{
class PatternGenerator
{
public:
  PatternGenerator(ros::NodeHandle& nh);
  virtual ~PatternGenerator();

  void reset();
  void setParams(const msgs::PatternGeneratorParameters& params_);
  void setEnabled(bool enabled);
  bool isEnabled() const;
  bool isSimulationMode() const;

  bool hasSteps() const;
  bool hasNewSteps() const;
  void getCompleteStepPlan(msgs::StepPlan& step_plan) const;
  void getNewestStepPlan(msgs::StepPlan& step_plan) const;
  void clearStepPlan();

  bool setNextStartStepIndex(int step_index);
  int getNextStartStepIndex() const;

  void update(const ros::TimerEvent& timer);
  void updateLastPerformedStepIndex(int last_performed_step_index_);
  void updateFirstChangeableStepIndex(int first_changeable_step_index_);

  // simple pattern generator
  msgs::ErrorStatus generatePattern(const msgs::StepPlanRequest& step_plan_request, msgs::StepPlan& step_plan);

  // typedefs
  typedef boost::shared_ptr<PatternGenerator> Ptr;
  typedef boost::shared_ptr<PatternGenerator> ConstPtr;

private:
  void updateFeetStartPose(uint8_t foot_index, const geometry_msgs::Pose& pose);
  void updateFeetStartPose(const msgs::Foot& foot);
  void updateFeetStartPose(const msgs::Feet& feet);
  void updateFeetStartPose(const msgs::Step& step);
  bool updateFeetStartPoseByStepMap(const std::map<unsigned int, msgs::Step>& map, unsigned int step_index);

  void updateFootstepMap(std::map<unsigned int, msgs::Step>& map, const std::vector<msgs::Step>& vec) const;
  void mapToVectorIndexed(const std::map<unsigned int, msgs::Step>& map, std::vector<msgs::Step>& vec, unsigned int start_index, unsigned int end_index) const;

  void generateSteps(unsigned int n, bool close_step = false);

  // handler for joystick input
  JoystickHandler::Ptr joystick_handler_;
  geometry_msgs::Twist cmd;

  // generator params
  std::string world_frame_id_;
  unsigned int number_of_steps_needed_;
  msgs::PatternGeneratorParameters params_;

  // controller feedback
  int last_performed_step_index_; // last executed step (executing step is current + 1)
  int first_changeable_step_index_;
  int next_step_index_needed_;

  // state of pattern generator
  msgs::Feet::Ptr start_feet_pose_;
  unsigned int foot_start_step_index_left_;
  unsigned int foot_start_step_index_right_;
  mutable bool has_new_steps_;

  // generated step plan
  msgs::StepPlan complete_step_plan_;
  msgs::StepPlan newest_step_plan_;
  std::map<unsigned int, msgs::Step> step_map_;

  // service clients
  ros::ServiceClient generate_feet_pose_client_;
  ros::ServiceClient step_plan_request_client_;
  ros::ServiceClient stitch_step_plan_client;

  // action clients
  boost::shared_ptr<actionlib::SimpleActionClient<msgs::ExecuteStepPlanAction>> execute_step_plan_ac_;
};
}

#endif
