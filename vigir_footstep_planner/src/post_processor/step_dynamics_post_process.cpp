#include <vigir_footstep_planner/post_processor/step_dynamics_post_process.h>

namespace vigir_footstep_planning
{
StepDynamicsPostProcessPlugin::StepDynamicsPostProcessPlugin(const ParameterSet& params)
  : PostProcessPlugin("step_dynamics_post_processor", params)
{
}

StepDynamicsPostProcessPlugin::StepDynamicsPostProcessPlugin()
  : PostProcessPlugin("step_dynamics_post_processor")
{
}

void StepDynamicsPostProcessPlugin::loadParams(const ParameterSet& params)
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
