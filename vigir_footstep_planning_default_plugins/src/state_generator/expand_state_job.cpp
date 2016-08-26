#include <vigir_footstep_planning_default_plugins/state_generator/expand_state_job.h>

#include <vigir_footstep_planning_plugins/plugin_aggregators/robot_model.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/world_model.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/step_cost_estimator.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/post_processor.h>



namespace vigir_footstep_planning
{
namespace threading
{
ExpandStateJob::ExpandStateJob(const Footstep& footstep, const PlanningState& state, double max_risk)
  : successful(false)
  , footstep_(footstep)
  , state_(state)
  , max_risk_(max_risk)
{
}

ExpandStateJob::~ExpandStateJob()
{
}

void ExpandStateJob::run()
{
  successful = false;

  /// TODO: backward search case
  if (state_.getPredState() == nullptr)
    return;

  next.reset(new PlanningState(footstep_.performMeOnThisState(state_)));
  State& next_state = next->getState();

  if (*(state_.getPredState()) == *next)
    return;

  // check reachability due to discretization
  const State& left_foot = state_.getLeg() == LEFT ? state_.getState() : state_.getPredState()->getState();
  const State& right_foot = state_.getLeg() == RIGHT ? state_.getState() : state_.getPredState()->getState();
  if (!RobotModel::instance().isReachable(left_foot, right_foot, next_state))
    return;

  // lookup costs
  double cost, risk;
  if (!StepCostEstimator::instance().getCost(left_foot, right_foot, next_state, cost, risk))
    return;

  if (risk >= max_risk_)
    return;

  cost += footstep_.getStepCost();

  next_state.setCost(cost);
  next_state.setRisk(risk);

  // update 3D pose based on world data
  WorldModel::instance().update3DData(next_state);

  // apply post processing steps
  PostProcessor::instance().postProcessForward(left_foot, right_foot, next_state);
  //PostProcessor::instance().postProcessBackward(left_foot, right_foot, next_state);

  // collision check
  if (!WorldModel::instance().isAccessible(next_state, state_.getState()))
    return;

  successful = true;
}
}
}
