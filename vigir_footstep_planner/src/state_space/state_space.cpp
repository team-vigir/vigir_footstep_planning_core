#include <vigir_footstep_planner/state_space/state_space.h>

namespace vigir_footstep_planning
{
StateSpace::StateSpace(const EnvironmentParameters& params, std::vector<int*>& state_ID2_index_mapping)
  : params(params)
  , ivIdPlanningStart(-1)
  , ivIdPlanningGoal(-1)
  , ivIdStartFootLeft(-1)
  , ivIdStartFootRight(-1)
  , ivIdGoalFootLeft(-1)
  , ivIdGoalFootRight(-1)
  , state_ID2_index_mapping(state_ID2_index_mapping)
  , ivpStateHash2State(new std::vector<PlanningState*>[params.hash_table_size])
  , ivRandomNodeDist(params.random_node_distance / params.cell_size)
  , ivHeuristicExpired(false)
  , frame_id("/world")
{
  // determine whether a (x,y) translation can be performed by the robot by
  // checking if it is within a certain area of performable steps
  for (int y = params.ivMaxInvStepRangeY; y <= params.ivMaxStepRangeY; y++)
  {
    for (int x = params.ivMaxInvStepRangeX; x <= params.ivMaxStepRangeX; x++)
    {
      bool in_step_range = pointWithinPolygon(x, y, params.step_range);

      if (!in_step_range)
        continue;

      // generate area of samplings for gpr/map-based planning
      for (int theta = params.ivMaxInvStepRangeTheta; theta <= params.ivMaxStepRangeTheta; theta++)
      {
        Footstep f(cont_val(x, params.cell_size), cont_val(y, params.cell_size), angle_cell_2_state(theta, params.angle_bin_size), 0.0, params.cell_size, params.num_angle_bins, params.hash_table_size);
        ivContFootstepSet.push_back(f);
      }
    }
  }

  // tests
  /*ros::NodeHandle nh;
  ivTestPub = nh.advertise<visualization_msgs::MarkerArray>("test", 1, true);

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "/world";
  marker.ns = "test";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;

  marker.id = 0;

  PlanningState stand (  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, ivparams.swing_height, ivparams.step_duration, RIGHT, ivparams.cell_size, ivparams.angle_bin_size, ivparams.hash_table_size);
  PlanningState before(-0.20, 0.25, 0.0, 0.0, 0.0, 0.0, ivparams.swing_height, ivparams.step_duration,  LEFT, ivparams.cell_size, ivparams.angle_bin_size, ivparams.hash_table_size);

  for(std::vector<Footstep>::const_iterator iter = ivGPRFootstepSet.begin(); iter != ivGPRFootstepSet.end(); iter++)
  {
    const PlanningState after = iter->performMeOnThisState(stand);


    //ROS_INFO("%i %i %i", after.getX(), after.getY(), after.getYaw());
    //const State &afterState = after.getState();
    //ROS_INFO("%f %f %f", afterState.getX(), afterState.getY(), afterState.getYaw());


    //ROS_INFO("%f %f %f", after.getX(), after.getY(), after.getYaw());

    int risk_cost;
    int cost = stepCost(stand, before, after, risk_cost);

    if (risk_cost < 500)
    {
      marker.id++;

      tf::poseTFToMsg(after.getState().getPose(), marker.pose);
      marker.pose.position.z = after.getState().getYaw();

      marker.color.r = double(risk_cost)/cvMmScale;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker);
    }
  }
  ivTestPub.publish(marker_array);*/

  /*ROS_INFO("x: [%i %i], y: [%i %i], theta: [%i %i]",
            ivMaxInvStepRangeX, ivMaxStepRangeX,
            ivMaxInvStepRangeY, ivMaxStepRangeY,
            ivMaxInvStepRangeTheta, ivMaxStepRangeTheta);
  std::string msg;
  for (int x = ivMaxStepRangeX; x >= ivMaxInvStepRangeX; --x)
  {
    //msg += boost::lexical_cast<std::string>(x) + ": ";
    for (int y = ivMaxStepRangeY; y >= ivMaxInvStepRangeY; --y)
    {
      bool in_step_range = ivpStepRange[(y - ivMaxInvStepRangeY) * num_x + (x - ivMaxInvStepRangeX)];
      msg += in_step_range ? "+ " : "- ";
    }

    ROS_INFO("%s", msg.c_str());
    msg.clear();
  }
  ROS_INFO("Size: %lu of %u", ivContFootstepSet.size(), num_x * (ivMaxStepRangeY - ivMaxInvStepRangeY + 1) * (ivMaxStepRangeTheta - ivMaxInvStepRangeTheta + 1));
  ROS_INFO("%f %f %i", params.max_inverse_step_range_theta, params.max_step_range_theta, params.num_angle_bins);*/
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

bool StateSpace::reachable(const State& stand_foot, const State& swing_foot_after) const
{
  return RobotModel::isReachable(stand_foot, swing_foot_after);
}

bool StateSpace::closeToStart(const PlanningState& from) const
{
  assert(from.getSuccState() != nullptr);
  return reachable(ivStateId2State[ivIdPlanningStart]->getState(), from.getState());

  // check if first goal pose can be reached
  const State& left = (from.getLeg() == LEFT) ? from.getState() : from.getSuccState()->getState();
  const State& right = (from.getLeg() == RIGHT) ? from.getState() : from.getSuccState()->getState();
  State start = ivStateId2State[ivIdPlanningStart]->getState();

  PostProcessor::postProcessBackward(left, right, start);
  if (!reachable(from.getState(), start))
    return false;

  // check if second (final) goal can be reached
  State final_start;
  if (start.getLeg() == LEFT)
  {
    final_start = start_foot_right->getState();
    PostProcessor::postProcessBackward(start, right, final_start);
  }
  else
  {
    final_start = start_foot_left->getState();
    PostProcessor::postProcessBackward(left, start, final_start);
  }

  final_start.setBodyVelocity(geometry_msgs::Vector3()); // set velocity to zero
  if (!reachable(start, final_start))
    return false;

  return true;
}

bool StateSpace::closeToGoal(const PlanningState& from) const
{
  if (from.getLeg() == ivStateId2State[ivIdPlanningGoal]->getLeg())
    return false;

  assert(from.getPredState() != nullptr);

  // check if first goal pose can be reached
  const State& left = (from.getLeg() == LEFT) ? from.getState() : from.getPredState()->getState();
  const State& right = (from.getLeg() == RIGHT) ? from.getState() : from.getPredState()->getState();
  State goal = ivStateId2State[ivIdPlanningGoal]->getState();

  PostProcessor::postProcessForward(left, right, goal);
  if (!reachable(from.getState(), goal))
    return false;

  // check if second (final) goal can be reached
  State final_goal;
  if (goal.getLeg() == LEFT)
  {
    final_goal = goal_foot_right->getState();
    PostProcessor::postProcessForward(goal, right, final_goal);
  }
  else
  {
    final_goal = goal_foot_left->getState();
    PostProcessor::postProcessForward(left, goal, final_goal);
  }

  final_goal.setBodyVelocity(geometry_msgs::Vector3()); // set velocity to zero
  if (!reachable(goal, final_goal))
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

  if (!StepCostEstimator::getCost(left_foot, right_foot, swing_foot_after, cost, risk))
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
