#include <vigir_footstep_planner/state_space/state_space.h>

#include <vigir_footstep_planning_lib/math.h>

#include <vigir_footstep_planning_plugins/plugin_aggregators/robot_model.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/post_processor.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/step_cost_estimator.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/heuristic.h>



namespace vigir_footstep_planning
{
StateSpace::StateSpace(const EnvironmentParameters& params, std::vector<int*>& state_ID2_index_mapping)
  : params(params)
  , frame_id("/world")
  , ivIdPlanningStart(-1)
  , ivIdPlanningGoal(-1)
  , ivIdStartFootLeft(-1)
  , ivIdStartFootRight(-1)
  , ivIdGoalFootLeft(-1)
  , ivIdGoalFootRight(-1)
  , state_ID2_index_mapping(state_ID2_index_mapping)
  , ivpStateHash2State(new std::vector<PlanningState*>[params.hash_table_size])
  , ivHeuristicExpired(false)
  , ivRandomNodeDist(params.random_node_distance / params.cell_size)
{
}

StateSpace::~StateSpace()
{
  if (ivpStateHash2State)
  {
    delete[] ivpStateHash2State;
    ivpStateHash2State = NULL;
  }
}

void StateSpace::reset()
{
  for(unsigned int i = 0; i < ivStateId2State.size(); ++i)
  {
    if (ivStateId2State[i])
    {
      delete ivStateId2State[i];
    }
  }
  ivStateId2State.clear();

  if (ivpStateHash2State)
  {
    for(int i = 0; i < params.hash_table_size; ++i)
      ivpStateHash2State[i].clear();
  }

  state_ID2_index_mapping.clear();

  ivRandomStates.clear();

  ivIdPlanningStart = -1;
  ivIdPlanningGoal = -1;

  ivIdGoalFootLeft = -1;
  ivIdGoalFootRight = -1;
  ivIdStartFootLeft = -1;
  ivIdStartFootRight = -1;

  ivHeuristicExpired = true;
}

void StateSpace::setFrameId(const std::string& frame_id)
{
  this->frame_id = frame_id;
}

std::pair<int, int> StateSpace::updateGoal(const State& foot_left, const State& foot_right)
{
  // keep the old IDs
  int goal_foot_id_left = ivIdGoalFootLeft;
  int goal_foot_id_right = ivIdGoalFootRight;

  // update the states for both feet (if necessary)
  goal_foot_left = getHashEntry(foot_left);
  if (goal_foot_left == NULL)
    goal_foot_left = createNewHashEntry(foot_left);
  goal_foot_right = getHashEntry(foot_right);
  if (goal_foot_right == NULL)
    goal_foot_right = createNewHashEntry(foot_right);
  ivIdGoalFootLeft = goal_foot_left->getId();
  ivIdGoalFootRight = goal_foot_right->getId();
  // check if everything has been set correctly
  assert(ivIdGoalFootLeft != -1);
  assert(ivIdGoalFootRight != -1);

  // if using the forward search a change of the goal states involves an
  // update of the heuristic
  if (params.forward_search)
  {
    // check if the goal states have been changed
    if (goal_foot_id_left != ivIdGoalFootLeft || goal_foot_id_right != ivIdGoalFootRight)
    {
      ivHeuristicExpired = true;
      //setStateArea(*goal_foot_left, *goal_foot_right);
    }
  }

  return std::pair<int, int>(ivIdGoalFootLeft, ivIdGoalFootRight);
}

std::pair<int, int> StateSpace::updateStart(const State& foot_left, const State& foot_right)
{
  // keep the old IDs
  int start_foot_id_left = ivIdStartFootLeft;
  int start_foot_id_right = ivIdStartFootRight;

  // update the states for both feet (if necessary)
  start_foot_left = getHashEntry(foot_left);
  if (start_foot_left == NULL)
    start_foot_left = createNewHashEntry(foot_left);
  start_foot_right = getHashEntry(foot_right);
  if (start_foot_right == NULL)
    start_foot_right = createNewHashEntry(foot_right);
  ivIdStartFootLeft = start_foot_left->getId();
  ivIdStartFootRight = start_foot_right->getId();
  // check if everything has been set correctly
  assert(ivIdStartFootLeft != -1);
  assert(ivIdStartFootRight != -1);

  // if using the backward search a change of the start states involves an
  // update of the heuristic
  if (!params.forward_search)
  {
    // check if the start states have been changed
    if (start_foot_id_left != ivIdStartFootLeft || start_foot_id_right != ivIdStartFootRight)
    {
      ivHeuristicExpired = true;
      //setStateArea(*start_foot_left, *start_foot_right);
    }
  }

  return std::pair<int, int>(ivIdStartFootLeft, ivIdStartFootRight);
}

void StateSpace::setPlannerStartAndGoal(unsigned int start_foot_selection)
{
  if (start_foot_selection == msgs::StepPlanRequest::AUTO)
  {
    State robot_start;
    getStartState(robot_start);

    State robot_goal;
    getGoalState(robot_goal);

    tf::Transform direction = robot_start.getPose().inverse() * robot_goal.getPose();
    if (direction.getOrigin().getY() > 0.0)
      start_foot_selection = msgs::StepPlanRequest::RIGHT;
    else
      start_foot_selection = msgs::StepPlanRequest::LEFT;
  }

  if (start_foot_selection == msgs::StepPlanRequest::LEFT) // move first right foot
  {
    ivIdPlanningStart = ivIdStartFootLeft;
    start_foot_right->setSuccState(start_foot_left);
    start_foot_left->setPredState(start_foot_right);

    ivIdPlanningGoal = ivIdGoalFootRight;
    goal_foot_right->setSuccState(goal_foot_left);
    goal_foot_left->setPredState(goal_foot_right);
  }
  else if (start_foot_selection == msgs::StepPlanRequest::RIGHT) // move first left foot
  {
    ivIdPlanningStart = ivIdStartFootRight;
    start_foot_left->setSuccState(start_foot_right);
    start_foot_right->setPredState(start_foot_left);

    ivIdPlanningGoal = ivIdGoalFootLeft;
    goal_foot_left->setSuccState(goal_foot_right);
    goal_foot_right->setPredState(goal_foot_left);
  }
  else
    ROS_ERROR("[setPlannerStartAndGoal] Unknown selection mode: %u", start_foot_selection);
}

bool StateSpace::getState(unsigned int id, State &s) const
{
  if (id >= ivStateId2State.size())
    return false;

  s = ivStateId2State[id]->getState();
  return true;
}

bool StateSpace::getStartState(State &left, State &right) const
{
  if (!getState(ivIdStartFootLeft, left))
    return false;
  if (!getState(ivIdStartFootRight, right))
    return false;
  return true;
}

bool StateSpace::getStartState(State &robot) const
{
  State left;
  State right;

  if (!getState(ivIdStartFootLeft, left))
    return false;
  if (!getState(ivIdStartFootRight, right))
    return false;

  robot.setX(0.5*(left.getX()+right.getX()));
  robot.setY(0.5*(left.getY()+right.getY()));
  robot.setZ(0.5*(left.getZ()+right.getZ()));
  robot.setYaw(0.5*(left.getYaw()+right.getYaw()));
  return true;
}

bool StateSpace::getGoalState(State &left, State &right) const
{
  if (!getState(ivIdGoalFootLeft, left))
    return false;
  if (!getState(ivIdGoalFootRight, right))
    return false;
  return true;
}

bool StateSpace::getGoalState(State &robot) const
{
  State left;
  State right;

  if (!getState(ivIdGoalFootLeft, left))
    return false;
  if (!getState(ivIdGoalFootRight, right))
    return false;

  robot.setX(0.5*(left.getX()+right.getX()));
  robot.setY(0.5*(left.getY()+right.getY()));
  robot.setZ(0.5*(left.getZ()+right.getZ()));
  robot.setYaw(0.5*(left.getYaw()+right.getYaw()));
  return true;
}

PlanningState *StateSpace::createNewHashEntry(const State& s)
{
  PlanningState tmp(s, params.cell_size, params.angle_bin_size, params.hash_table_size);
  return createNewHashEntry(tmp);
}

PlanningState* StateSpace::createNewHashEntry(const PlanningState& s)
{
  unsigned int state_hash = s.getHashTag();
  PlanningState* new_state = new PlanningState(s);

  boost::unique_lock<boost::shared_mutex> lock(hash_table_shared_mutex);

  size_t state_id = ivStateId2State.size();
  assert(state_id < (size_t)std::numeric_limits<int>::max());

  // insert the ID of the new state into the corresponding map
  new_state->setId(state_id);
  ivStateId2State.push_back(new_state);

  // insert the new state into the hash map at the corresponding position
  ivpStateHash2State[state_hash].push_back(new_state);

  int* entry = new int[NUMOFINDICES_STATEID2IND];
  state_ID2_index_mapping.push_back(entry);

  for(int i = 0; i < NUMOFINDICES_STATEID2IND; ++i)
    state_ID2_index_mapping[state_id][i] = -1;

  assert(state_ID2_index_mapping.size() - 1 == state_id);

  return new_state;
}

PlanningState* StateSpace::getHashEntry(const State& s)
{
  PlanningState tmp(s, params.cell_size, params.angle_bin_size, params.hash_table_size);
  return getHashEntry(tmp);
}

PlanningState* StateSpace::getHashEntry(const PlanningState& s)
{
  unsigned int state_hash = s.getHashTag();
  std::vector<PlanningState*>::const_iterator state_iter;
  boost::shared_lock<boost::shared_mutex> lock(hash_table_shared_mutex);
  for (state_iter = ivpStateHash2State[state_hash].begin(); state_iter != ivpStateHash2State[state_hash].end(); ++state_iter)
  {
    if (*(*state_iter) == s)
      return *state_iter;
  }

  return NULL;
}

PlanningState* StateSpace::createHashEntryIfNotExists(const PlanningState& s)
{
  PlanningState* hash_entry = getHashEntry(s);
  if (hash_entry == NULL)
    hash_entry = createNewHashEntry(s);

  return hash_entry;
}

bool StateSpace::closeToStart(const PlanningState& from) const
{
  if (from.getLeg() == ivStateId2State[ivIdPlanningStart]->getLeg())
    return false;

  assert(from.getSuccState() != nullptr);

  // check if first goal pose can be reached
  State left_foot = (from.getLeg() == LEFT) ? from.getState() : from.getSuccState()->getState();
  State right_foot = (from.getLeg() == RIGHT) ? from.getState() : from.getSuccState()->getState();
  State start_foot = ivStateId2State[ivIdPlanningStart]->getState();

  PostProcessor::instance().postProcessBackward(left_foot, right_foot, start_foot);
  if (!RobotModel::instance().isReachable(left_foot, right_foot, start_foot))
    return false;

  // check if second (final) goal can be reached
  if (start_foot.getLeg() == LEFT)
  {
    left_foot = start_foot;
    start_foot = start_foot_right->getState();
  }
  else
  {
    right_foot = start_foot;
    start_foot = start_foot_left->getState();
  }

  PostProcessor::instance().postProcessBackward(left_foot, right_foot, start_foot);
  start_foot.setBodyVelocity(geometry_msgs::Vector3()); // set velocity to zero
  if (!RobotModel::instance().isReachable(left_foot, right_foot, start_foot))
    return false;

  return true;
}

bool StateSpace::closeToGoal(const PlanningState& from) const
{
  if (from.getLeg() == ivStateId2State[ivIdPlanningGoal]->getLeg())
    return false;

  assert(from.getPredState() != nullptr);

  // check if first goal pose can be reached
  State left_foot = (from.getLeg() == LEFT) ? from.getState() : from.getPredState()->getState();
  State right_foot = (from.getLeg() == RIGHT) ? from.getState() : from.getPredState()->getState();
  State goal_foot = ivStateId2State[ivIdPlanningGoal]->getState();

  PostProcessor::instance().postProcessForward(left_foot, right_foot, goal_foot);
  if (!RobotModel::instance().isReachable(left_foot, right_foot, goal_foot))
    return false;

  // check if second (final) goal can be reached
  if (goal_foot.getLeg() == LEFT)
  {
    left_foot = goal_foot;
    goal_foot = goal_foot_right->getState();
  }
  else
  {
    right_foot = goal_foot;
    goal_foot = goal_foot_left->getState();
  }

  PostProcessor::instance().postProcessForward(left_foot, right_foot, goal_foot);
  goal_foot.setBodyVelocity(geometry_msgs::Vector3()); // set velocity to zero
  if (!RobotModel::instance().isReachable(left_foot, right_foot, goal_foot))
    return false;

  return true;
}

bool StateSpace::getStepCost(const State& stand_foot, const State& swing_foot_before, const State& swing_foot_after, double& cost, double& risk) const
{
  cost = 0.0;
  risk = 0.0;

  if (stand_foot.getLeg() == swing_foot_before.getLeg())
  {
    ROS_ERROR("Can't compute step cost: No standing foot is same leg as swing foot!");
    return false;
  }

  if (swing_foot_before.getLeg() != swing_foot_after.getLeg())
  {
    ROS_ERROR("Can't compute step cost: Swing foot states have not same leg!");
    return false;
  }

  const State& left_foot  = stand_foot.getLeg() == LEFT  ? stand_foot : swing_foot_before;
  const State& right_foot = stand_foot.getLeg() == RIGHT ? stand_foot : swing_foot_before;

  if (!StepCostEstimator::instance().getCost(left_foot, right_foot, swing_foot_after, cost, risk))
    return false;

  if (risk >= params.max_risk)
    return false;

  return true;
}

bool StateSpace::getStepCost(const State& stand_foot, const State& swing_foot_before, const State& swing_foot_after, int& cost, int& risk) const
{
  double cost_d, risk_d;
  if (!getStepCost(stand_foot, swing_foot_before, swing_foot_after, cost_d, risk_d))
    return false;

  cost = static_cast<int>(cvMmScale * cost_d + 0.5);
  risk = static_cast<int>(cvMmScale * risk_d + 0.5);

  return true;
}

bool StateSpace::getStepCost(const State& stand_foot, const State& swing_foot_before, const State& swing_foot_after, int& cost) const
{
  int risk = 0;
  return getStepCost(stand_foot, swing_foot_before, swing_foot_after, cost, risk);
}
}
