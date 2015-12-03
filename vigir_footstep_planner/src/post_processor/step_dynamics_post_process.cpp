/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
#include <vigir_footstep_planner/post_processor/step_dynamics_post_process.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
StepDynamicsPostProcessPlugin::StepDynamicsPostProcessPlugin()
  : PostProcessPlugin("step_dynamics_post_processor")
{
}

void StepDynamicsPostProcessPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  params.getParam("swing_height", default_swing_height);
  params.getParam("sway_duration", default_sway_duration);
  params.getParam("step_duration", default_step_duration);
}

void StepDynamicsPostProcessPlugin::postProcessStepForward(const State& left_foot, const State& right_foot, State& swing_foot) const
{
  const State& swing_foot_before = swing_foot.getLeg() == LEFT ? left_foot : right_foot;
  const State& stand_foot = swing_foot.getLeg() == LEFT ? right_foot : left_foot;

  determineStepAttributes(swing_foot_before, stand_foot, swing_foot);
  determineTravelDistance(swing_foot_before, stand_foot, swing_foot);
  determineTimings(swing_foot_before, stand_foot, swing_foot);
  determineDynamics(swing_foot_before, stand_foot, swing_foot);
}

void StepDynamicsPostProcessPlugin::postProcessStepBackward(const State& /*left_foot*/, const State& /*right_foot*/, State& /*swing_foot*/) const
{
  ROS_WARN_ONCE("[StepDynamicsPostProcessPlugin] postProcessStepBackward not implemented yet!");
}

void StepDynamicsPostProcessPlugin::determineStepAttributes(const State& /*swing_foot_before*/, const State& /*stand_foot*/, State& swing_foot_after) const
{
  swing_foot_after.setSwingHeight(default_swing_height);
}

void StepDynamicsPostProcessPlugin::determineTravelDistance(const State& swing_foot_before, const State& stand_foot, State& swing_foot_after) const
{
  // determine sway distance
  tf::Transform dsway;
  getDeltaStep(stand_foot.getPose(), swing_foot_after.getPose(), dsway);
  swing_foot_after.setSwayDistance(sqrt(dsway.getOrigin().x()*dsway.getOrigin().x() + dsway.getOrigin().y()*dsway.getOrigin().y() + dsway.getOrigin().z()*dsway.getOrigin().z()));

  // determine swing distance
  tf::Transform dswing;
  getDeltaStep(swing_foot_before.getPose(), swing_foot_after.getPose(), dswing);
  swing_foot_after.setSwingDistance(sqrt(dswing.getOrigin().x()*dswing.getOrigin().x() + dswing.getOrigin().y()*dswing.getOrigin().y() + dswing.getOrigin().z()*dswing.getOrigin().z()));
}

void StepDynamicsPostProcessPlugin::determineTimings(const State& /*swing_foot_before*/, const State& /*stand_foot*/, State& swing_foot_after) const
{
  swing_foot_after.setSwayDuration(default_sway_duration);
  swing_foot_after.setStepDuration(default_step_duration);
}

void StepDynamicsPostProcessPlugin::determineDynamics(const State& swing_foot_before, const State& /*stand_foot*/, State& swing_foot_after) const
{
  if (swing_foot_after.getSwingDistance() > 0.0)
  {
    geometry_msgs::Vector3 body_vel;

    // determine swing direction
    //double swing_vel = swing_foot_after.getStepDuration() > 0.0 ? 0.5*swing_foot_after.getSwingDistance()/swing_foot_after.getStepDuration() : 0.0;
    //double norm_factor = swing_vel/swing_foot_after.getSwingDistance();
    double norm_factor = swing_foot_after.getStepDuration() > 0.0 ? 0.5/swing_foot_after.getStepDuration() : 0.0;

    body_vel.x = (swing_foot_after.getX() - swing_foot_before.getX()) * norm_factor;
    body_vel.y = (swing_foot_after.getY() - swing_foot_before.getY()) * norm_factor;
    body_vel.z = (swing_foot_after.getZ() - swing_foot_before.getZ()) * norm_factor;

    swing_foot_after.setBodyVelocity(body_vel);
  }
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::StepDynamicsPostProcessPlugin, vigir_footstep_planning::PostProcessPlugin)
