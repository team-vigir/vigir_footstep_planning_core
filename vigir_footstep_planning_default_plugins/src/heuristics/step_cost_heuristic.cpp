#include <vigir_footstep_planning_default_plugins/heuristics/step_cost_heuristic.h>



namespace vigir_footstep_planning
{
StepCostHeuristic::StepCostHeuristic()
  : HeuristicPlugin("step_cost_heuristic")
{
}

bool StepCostHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::loadParams(params))
    return false;

  params.getParam("const_step_cost_estimator/step_cost", step_cost_, 0.1);
  params.getParam("diff_angle_cost", diff_angle_cost_);
  params.getParam("max_step_dist/x", max_step_dist_x_inv_);
  max_step_dist_x_inv_ = 1.0/max_step_dist_x_inv_;
  params.getParam("max_step_dist/y", max_step_dist_y_inv_);
  max_step_dist_y_inv_ = 1.0/max_step_dist_y_inv_;
  return true;
}

double StepCostHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  // expected steps
  tf::Transform dstep;
  getDeltaStep(from.getPose(), to.getPose(), dstep);
  double expected_steps_x = std::abs(dstep.getOrigin().x()) * max_step_dist_x_inv_;
  double expected_steps_y = std::abs(dstep.getOrigin().y()) * max_step_dist_y_inv_;
  double expected_steps = std::ceil(std::max(expected_steps_x, expected_steps_y));

  double diff_angle = 0.0;
  if (diff_angle_cost_ > 0.0)
    diff_angle = std::abs(angles::shortest_angular_distance(to.getYaw(), from.getYaw()));

//  ROS_WARN("-------------------------------");
//  ROS_INFO("x: %f %f %f", step.getOrigin().x(), ivMaxStepDistX, std::abs(step.getOrigin().x()) / ivMaxStepDistX);
//  ROS_INFO("y: %f %f %f", step.getOrigin().y(), ivMaxStepDistY, std::abs(step.getOrigin().y()) / ivMaxStepDistY);
//  ROS_INFO("steps: %f, dist: %f, cost: %f", expected_steps, dist, (dist + expected_steps * ivStepCost + diff_angle * ivDiffAngleCost));

  return expected_steps * step_cost_ + diff_angle * diff_angle_cost_;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::StepCostHeuristic, vigir_footstep_planning::HeuristicPlugin)
