#include <pluginlib/class_list_macros.h>

#include <vigir_footstep_planner/step_cost_estimators/dynamics_step_cost_estimator.h>



namespace vigir_footstep_planning
{
DynamicsStepCostEstimator::DynamicsStepCostEstimator()
  : StepCostEstimatorPlugin("dynamics_step_cost_estimator")
{
}

void DynamicsStepCostEstimator::loadParams(const ParameterSet& params)
{
  params.getParam("dynamic_step_cost_estimator/lower_step_limit", lower_step_limit);
  params.getParam("dynamic_step_cost_estimator/upper_step_limit", upper_step_limit);
  params.getParam("dynamic_step_cost_estimator/max_near_distance", max_near_distance_sq);
  max_near_distance_sq = max_near_distance_sq*max_near_distance_sq;
}

bool DynamicsStepCostEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
{
  cost = 0.0;
  cost_multiplier = 1.0;
  risk = 0.0;
  risk_multiplier = 1.0;

  if (swing_foot == left_foot || swing_foot == right_foot)
    return true;

  // only in specific modes we must care about robot dynamics
//  if (planner_environment.getPlanningMode() == flor_footstep_planner_msgs::FootstepPlan::MODE_WALK)
//  {
    // check distance to start pose
    State robot_start;
    if (state_space->getStartState(robot_start))
    {
      double dist_sq = euclidean_distance_sq(robot_start.getX(), robot_start.getY(), swing_foot.getX(), swing_foot.getY());
      double factor = std::max(0.0, (max_near_distance_sq-dist_sq)/max_near_distance_sq);

      // check distance to goal pose
      State robot_goal;
      if (factor < 1.0 && state_space->getGoalState(robot_goal))
      {
        dist_sq = euclidean_distance_sq(robot_goal.getX(), robot_goal.getY(), swing_foot.getX(), swing_foot.getY());
        factor = std::max(factor, (max_near_distance_sq-dist_sq)/max_near_distance_sq);
      }

      factor = 1.0-factor;
      double max_step_dist = lower_step_limit + upper_step_limit*factor*factor;

      // determine step distance
      const State& swing_foot_before = swing_foot.getLeg() == LEFT ? left_foot : right_foot;
      double step_dist_sq = euclidean_distance_sq(swing_foot_before.getX(), swing_foot_before.getY(), swing_foot.getX(), swing_foot.getY());

      if (step_dist_sq > max_step_dist*max_step_dist)
        risk = 200.0;
    }

    //ROS_INFO("%f %f | %f %f | %f | %f", min_step_dist, max_step_dist, std::sqrt(step_dist_sq), std::sqrt(dist_sq), factor, risk);
//  }

  return true;
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::DynamicsStepCostEstimator, vigir_footstep_planning::StepCostEstimatorPlugin)
