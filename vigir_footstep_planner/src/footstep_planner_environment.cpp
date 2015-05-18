#include <vigir_footstep_planner/footstep_planner_environment.h>

namespace vigir_footstep_planning
{
FootstepPlannerEnvironment::FootstepPlannerEnvironment(const EnvironmentParameters& params, const FootPoseTransformer& foot_pose_transformer, boost::function<void (msgs::PlanningFeedback)>& feedback_cb)
  : DiscreteSpaceInformation()
  , foot_pose_transformer(foot_pose_transformer)
  , feedback_cb(feedback_cb)
  , params(params)
  , ivNumExpandedStates(0)
  , last_feedback_flush(ros::Time::now())
  , frame_id("/world")
{
  state_space.reset(new StateSpace(params, StateID2IndexMapping));

  expand_states_manager.reset(new threading::ThreadingManager<threading::ExpandStateJob>(params.threads, params.jobs_per_thread));
}

FootstepPlannerEnvironment::~FootstepPlannerEnvironment()
{
}

void FootstepPlannerEnvironment::setFrameId(const std::string& frame_id)
{
  this->frame_id = frame_id;
}

void FootstepPlannerEnvironment::updateHeuristicValues()
{
  // check if start and goal have been set
  assert(state_space->ivIdGoalFootLeft != -1 && state_space->ivIdGoalFootRight != -1);
  assert(state_space->ivIdStartFootLeft != -1 && state_space->ivIdStartFootRight != -1);

  if (!state_space->ivHeuristicExpired)
    return;

  ROS_INFO("Updating the heuristic values.");

//  if (state_space->ivHeuristicPtr->getHeuristicType() == Heuristic::PATH_STEP_COST_HEURISTIC)
//  {
//    boost::shared_ptr<PathCostHeuristic> h =
//        boost::dynamic_pointer_cast<PathCostHeuristic>(
//          state_space->ivHeuristicPtr);
//    MDPConfig MDPCfg;
//    InitializeMDPCfg(&MDPCfg);
//    const PlanningState* start = state_space->ivStateId2State[MDPCfg.startstateid];
//    const PlanningState* goal = state_space->ivStateId2State[MDPCfg.goalstateid];

//    // NOTE: start/goal state are set to left leg
//    bool success;
//    if (params.forward_search)
//      success = h->calculateDistances(*start, *goal);
//    else
//      success = h->calculateDistances(*goal, *start);
//    if (!success)
//    {
//      ROS_ERROR("Failed to calculate path cost heuristic.");
//      exit(1);
//    }
//  }

  ROS_DEBUG("Finished updating the heuristic values.");
  state_space->ivHeuristicExpired = false;
}

void FootstepPlannerEnvironment::reset()
{
  state_space->reset();

  ivExpandedStates.clear();
  ivNumExpandedStates = 0;

  visited_steps.clear();
  last_feedback_flush = ros::Time::now();
}

void FootstepPlannerEnvironment::stateExpanded(const PlanningState& s)
{
  ivExpandedStates.insert(std::pair<int,int>(s.getX(), s.getY()));
  ++ivNumExpandedStates;

  if (!feedback_cb.empty())
  {
    if (((ros::Time::now() - last_feedback_flush).toSec()) > 1.0/params.feedback_rate)
    {
      // send recent visited states
      msgs::PlanningFeedback feedback;
      feedback.header.stamp = ros::Time::now();
      feedback.header.frame_id = frame_id;
      feedback.visited_steps = visited_steps;

      // build current plan
      const PlanningState* state = &s;
      msgs::Step step;
      step.header = feedback.header;
      step.foot.header = feedback.header;

      feedback.current_step_plan.header = feedback.header;

      while (state)
      {
        state->getState().getStep(step);
        feedback.current_step_plan.steps.push_back(step);

        if (params.forward_search)
        {
          if (!state->getPredState())
            break;
          state = state->getPredState();
        }
        else
        {
          if (!state->getSuccState())
            break;
          state = state->getSuccState();
        }
      }

      // transform step plan
      foot_pose_transformer.transformToRobotFrame(feedback.current_step_plan);

      // publish feedback
      feedback_cb(feedback);

      visited_steps.clear();
      last_feedback_flush = ros::Time::now();
    }
  }
}

void FootstepPlannerEnvironment::stateVisited(const PlanningState& s)
{
  if (!feedback_cb.empty())
  {
    msgs::Step step;
    s.getState().getStep(step);
    step.header.stamp = ros::Time::now();
    step.header.frame_id = frame_id;
    step.foot.header = step.header;
    visited_steps.push_back(step); /// TODO: resize is expensive!
  }
}

int FootstepPlannerEnvironment::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  assert(FromStateID >= 0 && (unsigned int) FromStateID < state_space->ivStateId2State.size());
  assert(ToStateID >= 0 && (unsigned int) ToStateID < state_space->ivStateId2State.size());

  if ((FromStateID == state_space->ivIdGoalFootLeft && ToStateID == state_space->ivIdGoalFootRight)
      || (FromStateID == state_space->ivIdGoalFootRight && ToStateID == state_space->ivIdGoalFootLeft)){
    return 0;
  }

  return GetFromToHeuristic(*(state_space->ivStateId2State[FromStateID]), *(state_space->ivStateId2State[ToStateID]),
                            *(state_space->ivStateId2State[state_space->ivIdPlanningStart]), *(state_space->ivStateId2State[state_space->ivIdPlanningGoal]));
}

int FootstepPlannerEnvironment::GetFromToHeuristic(const PlanningState& from, const PlanningState& to, const PlanningState& start, const PlanningState& goal)
{
  return cvMmScale * params.heuristic_scale * Heuristic::getHeuristicValue(from.getState(), to.getState(), start.getState(), goal.getState());
}

int FootstepPlannerEnvironment::GetGoalHeuristic(int stateID)
{
  const PlanningState* current = state_space->ivStateId2State[stateID];
  if (current->getLeg() == LEFT)
    return GetFromToHeuristic(stateID, state_space->ivIdGoalFootLeft);
  else
    return GetFromToHeuristic(stateID, state_space->ivIdGoalFootRight);
  //return GetFromToHeuristic(stateID, state_space->ivIdPlanningGoal);
}

void FootstepPlannerEnvironment::GetPreds(int TargetStateID, std::vector<int> *PredIDV, std::vector<int> *CostV)
{
  boost::this_thread::interruption_point();

  PredIDV->clear();
  CostV->clear();

  assert(TargetStateID >= 0 && (unsigned int) TargetStateID < state_space->ivStateId2State.size());

  // make start states always absorbing
  if (TargetStateID == state_space->ivIdStartFootLeft || TargetStateID == state_space->ivIdStartFootRight)
    return;

  const PlanningState* current = state_space->ivStateId2State[TargetStateID];

  if (!current->getSuccState())
  {
    ROS_WARN("Step cost should be evaluated but current state has no successor!");
    return;
  }

  // make sure goal state transitions are consistent with
  // GetSuccs(some_state, goal_state) where goal_state is reachable by an
  // arbitrary step from some_state
  if (params.forward_search)
  {
    if (state_space->ivStateArea.empty())
      ROS_WARN("This should not happen! Reactivate setStateArea");
    if (TargetStateID == state_space->ivIdGoalFootLeft || TargetStateID == state_space->ivIdGoalFootRight)
    {
      const PlanningState* s;
      int cost;
      std::vector<int>::const_iterator state_id_iter;
      for(state_id_iter = state_space->ivStateArea.begin(); state_id_iter != state_space->ivStateArea.end(); ++state_id_iter)
      {
        s = state_space->ivStateId2State[*state_id_iter];
        if (s->getLeg() == current->getLeg())
          continue;
        if (*(current->getSuccState()) == *s)
          continue;

        if (!state_space->getStepCost(current->getState(), s->getState(), current->getSuccState()->getState(), cost))
          continue;

        PredIDV->push_back(s->getId());
        CostV->push_back(cost);
        stateVisited(*s);
      }
      return;
    }
  }

  stateExpanded(*current);

  // check if start is reachable
  if (state_space->closeToStart(*current))
  {
    const PlanningState* start = state_space->ivStateId2State[state_space->ivIdPlanningStart];
    if (*(current->getSuccState()) == *start)
      return;

    if (WorldModel::isAccessible(start->getState(), current->getState()))
    {
      int cost;
      if (state_space->getStepCost(current->getState(), start->getState(), current->getSuccState()->getState(), cost))
      {
        PredIDV->push_back(start->getId());
        CostV->push_back(cost);
        stateVisited(*start);
        return;
      }
    }
  }

  PredIDV->reserve(state_space->ivContFootstepSet.size());
  CostV->reserve(state_space->ivContFootstepSet.size());

  for(std::vector<Footstep>::const_iterator footstep_set_iter = state_space->ivContFootstepSet.begin(); footstep_set_iter != state_space->ivContFootstepSet.end(); footstep_set_iter++)
  {
    const PlanningState predecessor = footstep_set_iter->reverseMeOnThisState(*current);

    if (*(current->getSuccState()) == predecessor)
      continue;

    // lookup costs
    int cost;
    if (!state_space->getStepCost(current->getState(), predecessor.getState(), current->getSuccState()->getState(), cost))
      continue;

    cost += static_cast<int>(cvMmScale * footstep_set_iter->getStepCost());

    // collision check
    if (!WorldModel::isAccessible(predecessor.getState(), current->getState()))
      continue;

    const PlanningState* predecessor_hash = state_space->createHashEntryIfNotExists(predecessor);
    PredIDV->push_back(predecessor_hash->getId());
    CostV->push_back(cost);
    stateVisited(*predecessor_hash);
  }
}

int FootstepPlannerEnvironment::GetStartHeuristic(int stateID)
{
  const PlanningState* current = state_space->ivStateId2State[stateID];
  if (current->getLeg() == LEFT)
    return GetFromToHeuristic(stateID, state_space->ivIdStartFootLeft);
  else
    return GetFromToHeuristic(stateID, state_space->ivIdStartFootRight);
  //return GetFromToHeuristic(stateID, state_space->ivIdPlanningStart);
}


void FootstepPlannerEnvironment::GetSuccs(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV)
{
  boost::this_thread::interruption_point();

  SuccIDV->clear();
  CostV->clear();

  assert(SourceStateID >= 0 && unsigned(SourceStateID) < state_space->ivStateId2State.size());

  // make goal states always absorbing
  if (SourceStateID == state_space->ivIdGoalFootLeft || SourceStateID == state_space->ivIdGoalFootRight)
    return;

  const PlanningState* current = state_space->ivStateId2State[SourceStateID];

  if (!current->getPredState())
  {
    ROS_WARN("Step cost should be evaluated but current state has no predecessor!");
    return;
  }

  // make sure start state transitions are consistent with
  // GetPreds(some_state, start_state) where some_state is reachable by an
  // arbitrary step from start_state
  if (!params.forward_search)
  {
    if (state_space->ivStateArea.empty())
      ROS_WARN("This should not happen! Reactivate setStateArea");
    if (SourceStateID == state_space->ivIdStartFootLeft || SourceStateID == state_space->ivIdStartFootRight)
    {
      const PlanningState* s;
      int cost;
      std::vector<int>::const_iterator state_id_iter;
      for(state_id_iter = state_space->ivStateArea.begin(); state_id_iter != state_space->ivStateArea.end(); ++state_id_iter)
      {
        s = state_space->ivStateId2State[*state_id_iter];
        if (s->getLeg() == current->getLeg())
          continue;
        if (*(current->getPredState()) == *s)
          continue;

        if (!state_space->getStepCost(current->getState(), current->getPredState()->getState(), s->getState(), cost))
          continue;

        SuccIDV->push_back(s->getId());
        CostV->push_back(cost);
        stateVisited(*s);
      }
      return;
    }
  }

  stateExpanded(*current);

  // check if goal is reachable
  if (state_space->closeToGoal(*current))
  {
    const PlanningState* goal = state_space->ivStateId2State[state_space->ivIdPlanningGoal];
    if (*(current->getPredState()) == *goal)
      return;

    if (WorldModel::isAccessible(goal->getState(), current->getState()))
    {
      int cost;
      if (state_space->getStepCost(current->getState(), current->getPredState()->getState(), goal->getState(), cost))
      {
        SuccIDV->push_back(goal->getId());
        CostV->push_back(cost);
        stateVisited(*goal);
        return;
      }
    }
  }

  // explorate all state
  SuccIDV->reserve(state_space->ivContFootstepSet.size());
  CostV->reserve(state_space->ivContFootstepSet.size());

  std::list<threading::ExpandStateJob::Ptr> jobs;
  for(std::vector<Footstep>::const_iterator footstep_set_iter = state_space->ivContFootstepSet.begin(); footstep_set_iter != state_space->ivContFootstepSet.end(); footstep_set_iter++)
  {
    jobs.push_back(threading::ExpandStateJob::Ptr(new threading::ExpandStateJob(*footstep_set_iter, *current, *state_space)));
//    const PlanningState successor = footstep_set_iter->performMeOnThisState(*current, params.use_terrain_model ? state_space->ivTerrainModel : TerrainModel::ConstPtr());

//    if (*(current->getPredState()) == successor)
//      continue;

//    // lookup costs
//    int cost;
//    if (state_space->getStepCost(current->getState(), current->getPredState()->getState(), successor.getState(), cost))
//      continue;

//    cost += static_cast<int>(cvMmScale * footstep_set_iter->getStepCost());

//    // collision check
//    if (!WorldModel::isAccessible(predecessor.getState(), current->getState()))
//      continue;

//    const PlanningState* successor_hash = state_space->createHashEntryIfNotExists(successor);
//    SuccIDV->push_back(successor_hash->getId());
//    CostV->push_back(cost);
//    stateVisited(*successor_hash, cost, risk_cost);
  }

  expand_states_manager->addJobs(jobs);
  expand_states_manager->waitUntilJobsFinished();

  for (std::list<threading::ExpandStateJob::Ptr>::iterator itr = jobs.begin(); itr != jobs.end(); itr++)
  {
    threading::ExpandStateJob::Ptr& job = *itr;
    if (!job->successful)
      continue;

    const PlanningState* successor_hash = state_space->createHashEntryIfNotExists(*(job->next));

    SuccIDV->push_back(successor_hash->getId());
    CostV->push_back(job->cost);
    stateVisited(*successor_hash);
  }
}

void FootstepPlannerEnvironment::GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV)
{
  boost::this_thread::interruption_point();

  assert(SourceStateID >= 0 && unsigned(SourceStateID) < state_space->ivStateId2State.size());
  //goal state should be absorbing
  if (SourceStateID == state_space->ivIdGoalFootLeft || SourceStateID == state_space->ivIdGoalFootRight )
    return;


  const PlanningState* currentState = state_space->ivStateId2State[SourceStateID];
  // TODO: state_space->closeToGoal?
  //
  //    	if (state_space->closeToGoal(currentState->getState()))
  //    		return;

  //get the successors
  //GetRandomNeighs(currentState, SuccIDV, CLowV, params.num_random_nodes, state_space->ivRandomNodeDist, true);
}

void FootstepPlannerEnvironment::GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV)
{
  boost::this_thread::interruption_point();

  assert(TargetStateID >= 0 && unsigned(TargetStateID) < state_space->ivStateId2State.size());

  // start state should be absorbing
  if (TargetStateID == state_space->ivIdStartFootLeft || TargetStateID == state_space->ivIdStartFootRight)
    return;

  const PlanningState* currentState = state_space->ivStateId2State[TargetStateID];

  // TODO: ???
  //    	if(state_space->closeToStart(currentState->getState()))
  //    		return;

  //get the predecessors
  //GetRandomNeighs(currentState, PredIDV, CLowV, params.num_random_nodes, state_space->ivRandomNodeDist, false);
}

bool FootstepPlannerEnvironment::AreEquivalent(int StateID1, int StateID2)
{
  assert(StateID1 >= 0 && StateID2 >= 0 && unsigned(StateID1) < state_space->ivStateId2State.size() && unsigned(StateID2) < state_space->ivStateId2State.size());

  if (StateID1 == StateID2)
    return true;

  const PlanningState* s1 = state_space->ivStateId2State[StateID1];
  const PlanningState* s2 = state_space->ivStateId2State[StateID2];

  return *s1 == *s2;
}

bool FootstepPlannerEnvironment::InitializeEnv(const char *sEnvFile)
{
  //  ROS_ERROR("FootstepPlanerEnvironment::InitializeEnv: Hit unimplemented "
  //            "function. Check this!");
  return true;
}

bool FootstepPlannerEnvironment::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  // NOTE: The internal start and goal ids are set here to the left foot
  // (this affects the calculation of the heuristic values)
  MDPCfg->startstateid = state_space->ivIdPlanningStart;
  MDPCfg->goalstateid = state_space->ivIdPlanningGoal;

  assert(state_space->ivIdPlanningStart != -1);
  assert(state_space->ivIdPlanningGoal != -1);

  return true;
}

void FootstepPlannerEnvironment::PrintEnv_Config(FILE *fOut)
{
  // NOTE: implement this if the planner needs to print out configurations
  ROS_ERROR("FootstepPlanerEnvironment::PrintEnv_Config: Hit "
            "unimplemented function. Check this!");
}

void FootstepPlannerEnvironment::PrintState(int stateID, bool bVerbose, FILE *fOut)
{
  if(fOut == NULL)
  {
    fOut = stdout;
  }

  if(stateID == state_space->ivIdGoalFootLeft && bVerbose)
  {
    SBPL_FPRINTF(fOut, "the state is a goal state\n");
  }

  const PlanningState* s = state_space->ivStateId2State[stateID];

  if(bVerbose)
  {
    SBPL_FPRINTF(fOut, "X=%i Y=%i THETA=%i FOOT=%i\n",
                 s->getX(), s->getY(), s->getYaw(), s->getLeg());
  }
  else
  {
    SBPL_FPRINTF(fOut, "%i %i %i %i\n",
                 s->getX(), s->getY(), s->getYaw(), s->getLeg());
  }
}

void FootstepPlannerEnvironment::SetAllActionsandAllOutcomes(CMDPSTATE *state)
{
  // NOTE: not implemented so far
  // Description: Some searches may also use SetAllActionsandAllOutcomes
  // or SetAllPreds functions if they keep the pointers to successors
  // (predecessors) but most searches do not require this, so it is not
  // necessary to support this

  ROS_ERROR("FootstepPlannerEnvironment::SetAllActionsandAllOutcomes: Hit"
            " unimplemented function. Check this!");
}

void FootstepPlannerEnvironment::SetAllPreds(CMDPSTATE *state)
{
  // NOTE: not implemented so far
  // Description: Some searches may also use SetAllActionsandAllOutcomes
  // or SetAllPreds functions if they keep the pointers to successors
  // (predecessors) but most searches do not require this, so it is not
  // necessary to support this

  ROS_ERROR("FootstepPlannerEnvironment::SetAllPreds: Hit unimplemented "
            "function. Check this!");
}

int FootstepPlannerEnvironment::SizeofCreatedEnv()
{
  return state_space->ivStateId2State.size();
}

bool FootstepPlannerEnvironment::less::operator()(const PlanningState* a, const PlanningState* b) const
{
  if (a->getX() < b->getX())
    return true;
  else if (a->getY() < b->getY())
    return true;
  else if (a->getYaw() < b->getYaw())
    return true;
  else
    return false;
}
}
