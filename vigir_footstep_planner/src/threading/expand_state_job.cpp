#include <vigir_footstep_planner/threading/expand_state_job.h>

#include <vigir_footstep_planner/robot_model/robot_model.h>
#include <vigir_footstep_planner/world_model/world_model.h>

namespace vigir_footstep_planning
{
namespace threading
{
ExpandStateJob::ExpandStateJob(const Footstep& footstep, const PlanningState& state, StateSpace& state_space)
  : footstep(footstep)
  , state(state)
  , state_space(state_space)
  , cost(0)
  , risk(0)
  , new_state(NULL)
  , successful(false)
{
}

ExpandStateJob::~ExpandStateJob()
{
}

void ExpandStateJob::run()
{
  new_state = NULL;
  successful = false;

  /// TODO: backward search case
  if (state.getPredState() == nullptr)
    return;

  next.reset(new PlanningState(footstep.performMeOnThisState(state)));
  State& next_state = next->getState();

  if (*(state.getPredState()) == *next)
    return;

  // check reachability due to discretization
  const State& left_foot = state.getLeg() == LEFT ? state.getState() : state.getPredState()->getState();
  const State& right_foot = state.getLeg() == RIGHT ? state.getState() : state.getPredState()->getState();
  if (!RobotModel::isReachable(left_foot, right_foot, next_state))
    return;

  // lookup costs
  double cost_d, risk_d;
  if (!state_space.getStepCost(state.getState(), state.getPredState()->getState(), next->getState(), cost_d, risk_d))
    return;

  cost_d += footstep.getStepCost();

  next_state.setCost(cost_d);
  next_state.setRisk(risk_d);
  cost = static_cast<int>(cvMmScale * cost_d);
  risk = static_cast<int>(cvMmScale * risk_d);

  // collision check
  if (!WorldModel::isAccessible(next->getState(), state.getState()))
    return;

  //new_state = state_space.createHashEntryIfNotExists(*next);
  successful = true;
}
}
}
