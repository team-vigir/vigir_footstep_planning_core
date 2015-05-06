#include <vigir_footstep_planner/footstep_planner.h>

using vigir_gridmap_2d::GridMap2D;
using vigir_gridmap_2d::GridMap2DPtr;

namespace vigir_footstep_planning
{
FootstepPlanner::FootstepPlanner(ros::NodeHandle &nh)
  : foot_pose_transformer(FootPoseTransformer(nh))
  , start_foot_selection(msgs::StepPlanRequest::AUTO)
  , start_pose_set_up(false)
  , goal_pose_set_up(false)
  , max_number_steps(0.0)
  , max_path_length_ratio(0.0)
  , ivPathCost(0)
  , ivCheckedFootContactSupport(new pcl::PointCloud<pcl::PointXYZI>())
  , step_plan_uid(0)
{
  nh.getParam("world_frame_id", frame_id);

  // publish topics
  ivCheckedFootContactSupportPub = nh.advertise<sensor_msgs::PointCloud2>("foot_contact_support", 1);

  // initialize the planner environment
  if (!ParameterManager::empty())
    setParams(ParameterManager::getActive());
  else
    ROS_ERROR("[FootstepPlanner] Can't initialize environment due to missing parameters!");
}

FootstepPlanner::~FootstepPlanner()
{
}

void FootstepPlanner::setPlanner()
{
  if (env_params->ivPlannerType == "ARAPlanner" ||
      env_params->ivPlannerType == "ADPlanner"  ||
      env_params->ivPlannerType == "RSTARPlanner" )
  {
    ROS_INFO_STREAM("Planning with " << env_params->ivPlannerType);
  }
  else
  {
    ROS_ERROR_STREAM("Planner "<< env_params->ivPlannerType <<" not available / untested.");
    exit(1);
  }
  if (env_params->forward_search)
  {
    ROS_INFO_STREAM("Search direction: forward planning");
  }
  else
  {
    ROS_INFO_STREAM("Search direction: backward planning");
  }

  if (env_params->ivPlannerType == "ARAPlanner")
  {
    ivPlannerPtr.reset(
          new ARAPlanner(ivPlannerEnvironmentPtr.get(),
                         env_params->forward_search));
  }
  else if (env_params->ivPlannerType == "ADPlanner")
  {
    ivPlannerPtr.reset(
          new ADPlanner(ivPlannerEnvironmentPtr.get(),
                        env_params->forward_search));
  }
  else if (env_params->ivPlannerType == "RSTARPlanner")
  {
    RSTARPlanner* p =
        new RSTARPlanner(ivPlannerEnvironmentPtr.get(),
                         env_params->forward_search);
    // new options, require patched SBPL
    //          p->set_local_expand_thres(500);
    //          p->set_eps_step(1.0);
    ivPlannerPtr.reset(p);
  }
  //        else if (env_params->ivPlannerType == "ANAPlanner")
  //        	ivPlannerPtr.reset(new anaPlanner(ivPlannerEnvironmentPtr.get(),
  //        	                                  ivForwardSearch));
}

bool FootstepPlanner::isPlanning() const
{
  return planning_thread.joinable();
}

bool FootstepPlanner::plan(ReplanParams& params)
{
  bool path_existed = (bool)ivPath.size();
  int ret = 0;
  MDPConfig mdp_config;
  std::vector<int> solution_state_ids;

  // commit start/goal poses to the environment
  /// @TODO: updateGoal adds goal states to list so planner may use it independend from costs...
  ivPlannerEnvironmentPtr->getStateSpace()->updateStart(ivStartFootLeft, ivStartFootRight);
  ivPlannerEnvironmentPtr->getStateSpace()->updateGoal(ivGoalFootLeft, ivGoalFootRight);
  ivPlannerEnvironmentPtr->getStateSpace()->setPlannerStartAndGoal(start_foot_selection);
  ivPlannerEnvironmentPtr->updateHeuristicValues();
  ivPlannerEnvironmentPtr->InitializeEnv(NULL);
  ivPlannerEnvironmentPtr->InitializeMDPCfg(&mdp_config);

  // inform AD planner about changed (start) states for replanning
  if (path_existed &&
      !env_params->forward_search &&
      env_params->ivPlannerType == "ADPlanner")
  {
    std::vector<int> changed_edges;
    changed_edges.push_back(mdp_config.startstateid);
    // update the AD planner
    boost::shared_ptr<ADPlanner> ad_planner =
        boost::dynamic_pointer_cast<ADPlanner>(ivPlannerPtr);
    ad_planner->update_preds_of_changededges(&changed_edges);
  }

  // set up SBPL
  if (ivPlannerPtr->set_start(mdp_config.startstateid) == 0)
  {
    ROS_ERROR("Failed to set start state.");
    return false;
  }
  if (ivPlannerPtr->set_goal(mdp_config.goalstateid) == 0)
  {
    ROS_ERROR("Failed to set goal state\n");
    return false;
  }

  int path_cost;
  ros::WallTime startTime = ros::WallTime::now();
  try
  {
    ROS_INFO("Start planning (max time: %f, initial eps: %f, decrease eps: %f\n", params.max_time, params.initial_eps, params.dec_eps);
    ret = ivPlannerPtr->replan(&solution_state_ids, params, &path_cost);
  }
  catch (const SBPL_Exception* e)
  {
    ROS_ERROR("SBPL planning failed (%s)", e->what());
    return false;
  }
  ivPathCost = double(path_cost) / cvMmScale;

  bool path_is_new = pathIsNew(solution_state_ids);
  if (ret && solution_state_ids.size() > 0 && path_is_new)
  {
    ROS_INFO("Solution of size %zu found after %f s",
             solution_state_ids.size(),
             (ros::WallTime::now()-startTime).toSec());

    if (extractPath(solution_state_ids))
    {
      ROS_INFO("Expanded states: %i total / %i new",
               ivPlannerEnvironmentPtr->getNumExpandedStates(),
               ivPlannerPtr->get_n_expands());
      ROS_INFO("Final eps: %f", ivPlannerPtr->get_final_epsilon());
      ROS_INFO("Path cost: %f (%i)\n", ivPathCost, path_cost);

      ivPlanningStatesIds = solution_state_ids;

      return true;
    }
    else
    {
      ROS_ERROR("extracting path failed\n\n");
      return false;
    }
  }
  else if (!path_is_new)
  {
    ROS_ERROR("Solution found by SBPL is the same as the old solution. Replanning failed.");
    return false;
  }
  else
  {
    ROS_ERROR("No solution found");
    return false;
  }
}

bool FootstepPlanner::extractPath(const std::vector<int>& state_ids)
{
  ivPath.clear();

  State s;
  State start_left;
  std::vector<int>::const_iterator state_ids_iter = state_ids.begin();

  // first state is always the robot's left foot
  if (!ivPlannerEnvironmentPtr->getStateSpace()->getState(*state_ids_iter, start_left))
  {
    ivPath.clear();
    return false;
  }
  ++state_ids_iter;
  if (!ivPlannerEnvironmentPtr->getStateSpace()->getState(*state_ids_iter, s))
  {
    ivPath.clear();
    return false;
  }
  ++state_ids_iter;

  // check if the robot's left foot can be ommited as first state in the path,
  // i.e. the robot's right foot is appended first to the path
  if (s.getLeg() == LEFT)
    ivPath.push_back(ivStartFootRight);
  else
    ivPath.push_back(start_left);
  ivPath.push_back(s);

  for(; state_ids_iter < state_ids.end(); ++state_ids_iter)
  {
    if (!ivPlannerEnvironmentPtr->getStateSpace()->getState(*state_ids_iter, s))
    {
      ivPath.clear();
      return false;
    }
    ivPath.push_back(s);
  }

  // add last neutral step
  if (ivPath.back().getLeg() == RIGHT)
    ivPath.push_back(ivGoalFootLeft);
  else // last_leg == LEFT
    ivPath.push_back(ivGoalFootRight);

  return true;
}

bool FootstepPlanner::setParams(const ParameterSet& params)
{
  boost::recursive_mutex::scoped_lock lock(planner_mutex);

  // reinitialize the planner environment parameters
  env_params.reset(new EnvironmentParameters(params));

  PluginManager::loadParams(params);

  // reinitialize robot model
  RobotModel::loadPlugins();

  // reinitialize world model
  WorldModel::loadPlugins();

  // reinitialize step cost estimators
  StepCostEstimator::loadPlugins();

  // reinitialize heuristics
  Heuristic::loadPlugins();

  resetTotally();

  return true;
}

msgs::ErrorStatus FootstepPlanner::updateFoot(msgs::Foot& foot, uint8_t mode, bool transform) const
{
  msgs::ErrorStatus status;

  // transform to sole frame
  if (transform)
    foot_pose_transformer.transformToPlannerFrame(foot);

  if (mode & msgs::UpdateMode::UPDATE_MODE_MOVE_TO_VALID)
  {
    State s(foot, 0.0, 0.0);
    if (findNearestValidState(s))
      s.getFoot(foot);
    else
      status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "updateFoot: Couldn't determine valid position!");
  }
  else if (mode & msgs::UpdateMode::UPDATE_MODE_3D)
  {
    if (!WorldModel::isTerrainModelAvailable() || !WorldModel::getTerrainModel()->update3DData(foot.pose))
      status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlanner", "updateFoot: Couldn't update 3D data.", false);
  }
  else if (mode & msgs::UpdateMode::UPDATE_MODE_Z)
  {
    if (!WorldModel::isTerrainModelAvailable() || !WorldModel::getTerrainModel()->getHeight(foot.pose.position.x, foot.pose.position.y, foot.pose.position.z))
      status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlanner", "updateFoot: Couldn't update z.", false);
  }

  // transform back to robot frame
  if (transform)
    foot_pose_transformer.transformToRobotFrame(foot);

  foot.header.stamp = ros::Time::now();

  return status;
}

msgs::ErrorStatus FootstepPlanner::updateFeet(msgs::Feet& feet, uint8_t mode, bool transform) const
{
  msgs::ErrorStatus status;

  if (mode & msgs::UpdateMode::UPDATE_MODE_MOVE_TO_VALID)
  {
    State left(feet.left, 0.0, 0.0);
    State right(feet.right, 0.0, 0.0);
    if (findNearestValidState(left) && findNearestValidState(right) && RobotModel::isReachable(left, right))
    {
      left.getFoot(feet.left);
      right.getFoot(feet.right);
    }
    else
      status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "updateFeet: Couldn't determine valid position!");

    mode -= msgs::UpdateMode::UPDATE_MODE_MOVE_TO_VALID;
  }

  status += updateFoot(feet.left, mode, transform);
  status += updateFoot(feet.right, mode, transform);

  return status;
}

msgs::ErrorStatus FootstepPlanner::updateStepPlan(msgs::StepPlan& step_plan, uint8_t mode, const std::string& /*param_set_name*/, bool transform) const
{
  if (step_plan.steps.empty())
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "[FootstepPlanner]", "updateStepPlan: Can't process empty step plan!");
  if (step_plan.start.header.frame_id.empty())
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "[FootstepPlanner]", "updateStepPlan: Can't process step plan due to missing start feet poses!");

  // lock entire planning system
  boost::recursive_mutex::scoped_lock lock(planner_mutex);

  // transform to sole frame
  if (transform)
    foot_pose_transformer.transformToPlannerFrame(step_plan);

  msgs::ErrorStatus status;

  if (mode & msgs::UpdateMode::UPDATE_MODE_REPLAN)
  {
    ROS_WARN("UPDATE_MODE_REPLAN isn't implemented yet!");
  }
  else
  {
    std::vector<msgs::Step>::iterator itr = step_plan.steps.begin();

    State left_state(step_plan.start.left, 0.0, 0.0);
    State right_state(step_plan.start.right, 0.0, 0.0);
    State prev_state(*itr);

    itr++;
    for (; itr != step_plan.steps.end(); itr++)
    {
      msgs::Step& cur_step = *itr;
      State cur_state = State(cur_step);

      // update feet
      status += updateFoot(cur_step.foot, mode, false);

      // check reachability
      if (mode & msgs::UpdateMode::UPDATE_MODE_CHECK_VALIDITY)
      {
        cur_step.valid = ivPlannerEnvironmentPtr->getStateSpace()->reachable(prev_state, cur_state);
      }

      // check collision
      if (mode & msgs::UpdateMode::UPDATE_MODE_CHECK_COLLISION)
      {
        cur_step.colliding = !WorldModel::isAccessible(cur_state, prev_state);
      }

      // recompute cost
      if (mode & msgs::UpdateMode::UPDATE_MODE_COST)
      {
        double c, r;
        if (StepCostEstimator::getCost(left_state, right_state, cur_state, c, r))
        {
          cur_step.cost = c;
          cur_step.risk = r;
        }
        else
          status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlanner", "updateStepPlan: Couldn't determine cost for step " + boost::lexical_cast<std::string>(cur_step.step_index) + "!");
      }

      // prepare next iteration
      if (cur_state.getLeg() == LEFT)
        left_state = cur_state;
      else
        right_state = cur_state;

      prev_state = cur_state;
    }
  }

  // transform back to robot frame
  if (transform)
    foot_pose_transformer.transformToRobotFrame(step_plan);

  step_plan.header.stamp = ros::Time::now();

  return msgs::ErrorStatus();
}

void FootstepPlanner::reset()
{
  boost::recursive_mutex::scoped_lock lock(planner_mutex);

  // reset the previously calculated paths
  ivPath.clear();
  ivPlanningStatesIds.clear();
  ivCheckedFootContactSupport->clear();
  // reset the planner
  // INFO: force_planning_from_scratch was not working properly the last time
  // checked; therefore instead of using this function the planner is manually
  // reset
  //ivPlannerPtr->force_planning_from_scratch();
  ivPlannerEnvironmentPtr->reset();
  setPlanner();
}


void FootstepPlanner::resetTotally()
{
  boost::recursive_mutex::scoped_lock lock(planner_mutex);

  // reset the previously calculated paths
  ivPath.clear();
  ivPlanningStatesIds.clear();
  ivCheckedFootContactSupport->clear();
  // reinitialize the planner environment
  ivPlannerEnvironmentPtr.reset(new FootstepPlannerEnvironment(*env_params, foot_pose_transformer, feedback_cb));
  setPlanner();
}

msgs::ErrorStatus FootstepPlanner::planSteps(msgs::StepPlanRequestService::Request& req)
{
  // set start foot poses
  if (!setStart(req.plan_request, true)) /// @TODO: Hack to disable collision check for start pose
    return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_START, "FootstepPlanner", "planSteps: Couldn't set start pose! Please check if poses are set!");

  // set goal foot poses
  if (!setGoal(req.plan_request))
    return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_GOAL, "FootstepPlanner", "planSteps: Couldn't set goal pose! Please check if poses are set!");

  reset();

  ReplanParams params(req.plan_request.max_planning_time);
  params.initial_eps = env_params->initial_eps;
  params.final_eps = 1.0;
  params.dec_eps = env_params->decrease_eps;
  params.return_first_solution = env_params->ivSearchUntilFirstSolution;
  params.repair_time = -1;

  // start the planning and return success
  if (!plan(params))
    return ErrorStatusError(msgs::ErrorStatus::ERR_NO_SOLUTION, "FootstepPlanner", "planSteps: No solution found!");

  return msgs::ErrorStatus();
}

msgs::ErrorStatus FootstepPlanner::planPattern(msgs::StepPlanRequestService::Request& req)
{
  double cell_size = 0.0001;
  int num_angle_bins = 2048;
  double angle_bin_size = (2.0*M_PI / num_angle_bins);

  // set start foot poses
  if (!setStart(req.plan_request, true))
    return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_START, "FootstepPlanner", "planPattern: Couldn't set start pose! Please check if poses are set!");

  boost::shared_ptr<Footstep> footstep;
  boost::shared_ptr<Footstep> footstep_left;
  boost::shared_ptr<Footstep> footstep_right;

  State current_state;
  unsigned int num_steps = req.plan_request.pattern_parameters.steps;
  bool close_step = req.plan_request.pattern_parameters.close_step && num_steps > 0;
  bool single_step_mode = false;

  bool change_z = true;
  double step_up_height = std::abs(req.plan_request.pattern_parameters.dz);
  double extra_seperation_factor = req.plan_request.pattern_parameters.extra_seperation ? 1.25 : 1.0;

  ROS_INFO("Start planning stepping (mode: %u, steps: %u)\n", req.plan_request.pattern_parameters.mode, num_steps);

  msgs::ErrorStatus status;

  switch (req.plan_request.pattern_parameters.mode)
  {
    case msgs::PatternParameters::STEP_UP:
    case msgs::PatternParameters::STEP_DOWN:
    case msgs::PatternParameters::STEP_OVER:
      change_z = false; // as dz is used for step up/down motion
    case msgs::PatternParameters::FORWARD:
    {
      footstep = boost::shared_ptr<Footstep>(
                   new Footstep(req.plan_request.pattern_parameters.step_distance_forward, env_params->foot_seperation, 0.0,
                                env_params->swing_height,
                                env_params->step_duration,
                                0.0,
                                cell_size,
                                num_angle_bins,
                                env_params->hash_table_size));
      current_state = req.plan_request.start_foot_selection == msgs::StepPlanRequest::RIGHT ? ivStartFootLeft : ivStartFootRight;
      break;
    }
    case msgs::PatternParameters::BACKWARD:
    {
      footstep = boost::shared_ptr<Footstep>(
                   new Footstep(-req.plan_request.pattern_parameters.step_distance_forward, env_params->foot_seperation, 0.0,
                                env_params->swing_height,
                                env_params->step_duration,
                                0.0,
                                cell_size,
                                num_angle_bins,
                                env_params->hash_table_size));
      current_state = req.plan_request.start_foot_selection == msgs::StepPlanRequest::RIGHT ? ivStartFootLeft : ivStartFootRight;
      break;
    }
    case msgs::PatternParameters::STRAFE_LEFT:
    {
      footstep = boost::shared_ptr<Footstep>(
                   new Footstep(0.0, env_params->foot_seperation+req.plan_request.pattern_parameters.step_distance_sideward, 0.0,
                                env_params->swing_height,
                                env_params->step_duration,
                                0.0,
                                cell_size,
                                num_angle_bins,
                                env_params->hash_table_size));
      current_state = ivStartFootRight;
      single_step_mode = true;
      break;
    }
    case msgs::PatternParameters::STRAFE_RIGHT:
    {
      footstep = boost::shared_ptr<Footstep>(
                   new Footstep(0.0, env_params->foot_seperation+req.plan_request.pattern_parameters.step_distance_sideward, 0.0,
                                env_params->swing_height,
                                env_params->step_duration,
                                0.0,
                                cell_size,
                                num_angle_bins,
                                env_params->hash_table_size));
      current_state = ivStartFootLeft;
      single_step_mode = true;
      break;
    }
    case msgs::PatternParameters::ROTATE_LEFT:
    {
      footstep = boost::shared_ptr<Footstep>(
                   new Footstep(-sin(req.plan_request.pattern_parameters.turn_angle)*(env_params->foot_seperation*extra_seperation_factor)/2,
                                cos(req.plan_request.pattern_parameters.turn_angle)*(env_params->foot_seperation*extra_seperation_factor)/2+(env_params->foot_seperation*extra_seperation_factor)/2,
                                req.plan_request.pattern_parameters.turn_angle,
                                env_params->swing_height,
                                env_params->step_duration,
                                0.0,
                                cell_size,
                                num_angle_bins,
                                env_params->hash_table_size));
      current_state = ivStartFootRight;
      single_step_mode = true;
      break;
    }
    case msgs::PatternParameters::ROTATE_RIGHT:
    {
      footstep = boost::shared_ptr<Footstep>(
                   new Footstep(-sin(req.plan_request.pattern_parameters.turn_angle)*(env_params->foot_seperation*extra_seperation_factor)/2,
                                cos(req.plan_request.pattern_parameters.turn_angle)*(env_params->foot_seperation*extra_seperation_factor)/2+(env_params->foot_seperation*extra_seperation_factor)/2,
                                req.plan_request.pattern_parameters.turn_angle,
                                env_params->swing_height,
                                env_params->step_duration,
                                0.0,
                                cell_size,
                                num_angle_bins,
                                env_params->hash_table_size));
      current_state = ivStartFootLeft;
      single_step_mode = true;
      break;
    }
    case msgs::PatternParameters::SAMPLING:
    {
      footstep_left = boost::shared_ptr<Footstep>(
                        new Footstep(req.plan_request.pattern_parameters.step_distance_forward, env_params->foot_seperation+req.plan_request.pattern_parameters.step_distance_sideward, req.plan_request.pattern_parameters.turn_angle,
                                     env_params->swing_height,
                                     env_params->step_duration,
                                     0.0,
                                     cell_size,
                                     num_angle_bins,
                                     env_params->hash_table_size));
      footstep_right = boost::shared_ptr<Footstep>(
                         new Footstep(req.plan_request.pattern_parameters.step_distance_forward, env_params->foot_seperation-req.plan_request.pattern_parameters.step_distance_sideward, -req.plan_request.pattern_parameters.turn_angle,
                                      env_params->swing_height,
                                      env_params->step_duration,
                                      0.0,
                                      cell_size,
                                      num_angle_bins,
                                      env_params->hash_table_size));
      current_state = req.plan_request.start_foot_selection == msgs::StepPlanRequest::RIGHT ? ivStartFootLeft : ivStartFootRight;
      single_step_mode = std::abs(req.plan_request.pattern_parameters.step_distance_sideward) > 0.0;
      break;
    }
    case msgs::PatternParameters::FEET_REALIGN_ON_CENTER:
    {
      State feet_center(0.5*(ivStartFootLeft.getX()+ivStartFootRight.getX()),
                        0.5*(ivStartFootLeft.getY()+ivStartFootRight.getY()),
                        0.5*(ivStartFootLeft.getZ()+ivStartFootRight.getZ()),
                        0.5*(ivStartFootLeft.getRoll()+ivStartFootRight.getRoll()),
                        0.5*(ivStartFootLeft.getPitch()+ivStartFootRight.getPitch()),
                        0.5*(ivStartFootLeft.getYaw()+ivStartFootRight.getYaw()),
                        env_params->swing_height, env_params->step_duration, NOLEG);

      // first add start foot
      current_state = req.plan_request.start_foot_selection == msgs::StepPlanRequest::RIGHT ? ivStartFootLeft : ivStartFootRight;
      status += finalizeAndAddStepToPlan(current_state, req.plan_request.pattern_parameters, false);
      if (hasError(status))
        return status;

      // add pattern directly
      if (req.plan_request.start_foot_selection == msgs::StepPlanRequest::RIGHT)
      {
        State temp = getFootPose(feet_center, RIGHT);
        finalizeAndAddStepToPlan(temp, req.plan_request.pattern_parameters, false);

        temp = getFootPose(feet_center, LEFT);
        finalizeAndAddStepToPlan(temp, req.plan_request.pattern_parameters, false);
      }
      else
      {
        State temp = getFootPose(feet_center, LEFT);
        finalizeAndAddStepToPlan(temp, req.plan_request.pattern_parameters, false);

        temp = getFootPose(feet_center, RIGHT);
        finalizeAndAddStepToPlan(temp, req.plan_request.pattern_parameters, false);
      }

      return status;
    }
    case msgs::PatternParameters::FEET_REALIGN_ON_LEFT:
    {
      current_state = ivStartFootLeft;
      num_steps = 0;
      close_step = true;
      break;
    }
    case msgs::PatternParameters::FEET_REALIGN_ON_RIGHT:
    {
      current_state = ivStartFootRight;
      num_steps = 0;
      close_step = true;
      break;
    }
    case msgs::PatternParameters::WIDE_STANCE:
    {
      State feet_center(0.5*(ivStartFootLeft.getX()+ivStartFootRight.getX()),
                        0.5*(ivStartFootLeft.getY()+ivStartFootRight.getY()),
                        0.5*(ivStartFootLeft.getZ()+ivStartFootRight.getZ()),
                        0.5*(ivStartFootLeft.getRoll()+ivStartFootRight.getRoll()),
                        0.5*(ivStartFootLeft.getPitch()+ivStartFootRight.getPitch()),
                        0.5*(ivStartFootLeft.getYaw()+ivStartFootRight.getYaw()),
                        env_params->swing_height, env_params->step_duration, NOLEG);

      // first add start foot
      current_state = req.plan_request.start_foot_selection == msgs::StepPlanRequest::RIGHT ? ivStartFootLeft : ivStartFootRight;
      status += finalizeAndAddStepToPlan(current_state, req.plan_request.pattern_parameters, false);
      if (hasError(status))
        return status;

      // add pattern directly
      if (req.plan_request.start_foot_selection == msgs::StepPlanRequest::RIGHT)
      {
        State temp = getFootPose(feet_center, RIGHT, req.plan_request.pattern_parameters.step_distance_forward, req.plan_request.pattern_parameters.step_distance_sideward, req.plan_request.pattern_parameters.turn_angle);
        finalizeAndAddStepToPlan(temp, req.plan_request.pattern_parameters, false);

        temp = getFootPose(feet_center, LEFT, req.plan_request.pattern_parameters.step_distance_forward, req.plan_request.pattern_parameters.step_distance_sideward, req.plan_request.pattern_parameters.turn_angle);
        finalizeAndAddStepToPlan(temp, req.plan_request.pattern_parameters, false);
      }
      else
      {
        State temp = getFootPose(feet_center, LEFT, req.plan_request.pattern_parameters.step_distance_forward, req.plan_request.pattern_parameters.step_distance_sideward, req.plan_request.pattern_parameters.turn_angle);
        finalizeAndAddStepToPlan(temp, req.plan_request.pattern_parameters, false);

        temp = getFootPose(feet_center, RIGHT, req.plan_request.pattern_parameters.step_distance_forward, req.plan_request.pattern_parameters.step_distance_sideward, req.plan_request.pattern_parameters.turn_angle);
        finalizeAndAddStepToPlan(temp, req.plan_request.pattern_parameters, false);
      }

      return status;
    }
    default:
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "planPattern: Unknown walk mode (" + boost::lexical_cast<std::string>(req.plan_request.pattern_parameters.mode) + ") set!");
  }

  // generate simple path via pattern generator
  /// @TODO: check for collision

  // add start state
  status += finalizeAndAddStepToPlan(current_state, req.plan_request.pattern_parameters, false);
  if (hasError(status))
    return status;

  // add step up motion
  if (req.plan_request.pattern_parameters.mode == msgs::PatternParameters::STEP_UP || req.plan_request.pattern_parameters.mode == msgs::PatternParameters::STEP_OVER)
  {
    Footstep step_up(1.5*env_params->foot_size.x, env_params->foot_seperation, 0.0,
                     env_params->swing_height,
                     env_params->step_duration,
                     0.0,
                     cell_size,
                     num_angle_bins,
                     env_params->hash_table_size);

    // generate next step
    PlanningState next = step_up.performMeOnThisState(PlanningState(current_state, cell_size, angle_bin_size, env_params->hash_table_size));

    current_state = next.getState();
    current_state.setZ(current_state.getZ() + step_up_height);

    // add next step to plan
    status += finalizeAndAddStepToPlan(current_state, req.plan_request.pattern_parameters, false);
    if (hasError(status))
      return status;
  }

  // add pattern
  for (unsigned int i = 0; i < num_steps; i++)
  {
    // sampling uses alternating step sequences
    if (req.plan_request.pattern_parameters.mode == msgs::PatternParameters::SAMPLING)
    {
      if (current_state.getLeg() == LEFT)
        footstep = footstep_right;
      else if (current_state.getLeg() == RIGHT)
        footstep = footstep_left;
    }

    // generate next step
    PlanningState next = footstep->performMeOnThisState(PlanningState(current_state, cell_size, angle_bin_size, env_params->hash_table_size));
    current_state = next.getState();

    // add next step to plan
    status += finalizeAndAddStepToPlan(current_state, req.plan_request.pattern_parameters, change_z);
    if (hasError(status))
      return status;

    // in single step mode, the second foot should be placed parallel after each step
    if (single_step_mode && (!req.plan_request.pattern_parameters.close_step || i < (num_steps-1)))
    {
      current_state = getParallelFootPose(current_state, env_params->foot_seperation*(extra_seperation_factor-1.0));
      status += finalizeAndAddStepToPlan(current_state, req.plan_request.pattern_parameters, change_z);
      if (hasError(status))
        return status;
      i++;
    }
  }

  // add step down motion
  if (req.plan_request.pattern_parameters.mode == msgs::PatternParameters::STEP_DOWN || req.plan_request.pattern_parameters.mode == msgs::PatternParameters::STEP_OVER)
  {
    Footstep step_down(1.2*env_params->foot_size.x, env_params->foot_seperation, 0.0,
                     env_params->swing_height,
                     env_params->step_duration,
                     0.0,
                     cell_size,
                     num_angle_bins,
                     env_params->hash_table_size);

    // generate next step
    PlanningState next = step_down.performMeOnThisState(PlanningState(current_state, cell_size, angle_bin_size, env_params->hash_table_size));

    current_state = next.getState();
    current_state.setZ(current_state.getZ() - step_up_height);

    // add next step to plan
    status += finalizeAndAddStepToPlan(current_state, req.plan_request.pattern_parameters, false);
    if (hasError(status))
      return status;
  }

  // add final step so feet are parallel
  if (close_step)
  {
    current_state = getParallelFootPose(current_state);
    status += finalizeAndAddStepToPlan(current_state, req.plan_request.pattern_parameters, false);
    if (hasError(status))
      return status;
  }

  return status;
}

msgs::ErrorStatus FootstepPlanner::finalizeAndAddStepToPlan(State& s, const msgs::PatternParameters& params, bool change_z)
{
  if (change_z)
    s.setZ(s.getZ() + params.dz);

  if (params.override)
  {
    State temp = s;
    temp.setRoll(params.roll);
    temp.setPitch(params.pitch);
    ivPath.push_back(temp);
    return msgs::ErrorStatus();
  }
  else
  {
    ivPath.push_back(s);
    return msgs::ErrorStatus();
  }

  return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "addStepping: Something went wrong...");
}

bool FootstepPlanner::finalizeStepPlan(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp, bool post_process)
{
  resp.step_plan.header.frame_id = frame_id;
  resp.step_plan.header.stamp = req.plan_request.header.stamp;
  resp.step_plan.header.seq = step_plan_uid++;

  // add footstep plan
  msgs::Step step;

  // init msg
  StepPlanMsgPlugin::Ptr plugin;
  if (PluginManager::getPlugin(plugin))
  {
    plugin->initMsg(resp.step_plan);
    plugin->initMsg(step);
  }

  step.header = resp.step_plan.header;
  step.foot.header = resp.step_plan.header;
  step.valid = true;
  step.colliding = false;

  State left_foot = ivStartFootLeft;
  State right_foot = ivStartFootRight;

  int step_index = req.plan_request.start_step_index;
  resp.step_plan.steps.reserve(getPathSize());
  for (state_iter_t path_iter = getPathBegin(); path_iter != getPathEnd(); ++path_iter)
  {
    const State& swing_foot = *path_iter;
    const State& stand_foot = swing_foot.getLeg() == LEFT ? right_foot : left_foot;

    // convert footstep
    swing_foot.getStep(step);
    if (swing_foot.getLeg() == LEFT)
      step.foot.foot_index = msgs::Foot::LEFT;
    else if (swing_foot.getLeg() == RIGHT)
      step.foot.foot_index = msgs::Foot::RIGHT;
    else
    {
      ROS_ERROR("Footstep pose at (%f, %f, %f, %f) is set to NOLEG!",
                swing_foot.getX(), swing_foot.getY(), swing_foot.getZ(), swing_foot.getYaw());
      continue;
    }
    step.step_index = step_index++;

    if (std::abs(stand_foot.getZ() - swing_foot.getZ()) > 0.18)
      resp.status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlanner", "finalizeStepPlan: Plan contains large lift heights");

    // finally add step to plan
    resp.step_plan.steps.push_back(step);

    // next step
    if (swing_foot.getLeg() == LEFT)
      left_foot = swing_foot;
    else if (swing_foot.getLeg() == RIGHT)
      right_foot = swing_foot;
  }

  // perform post processing on entire plan
  if (post_process && !resp.step_plan.steps.empty())
  {
    // get post processing plugins
    std::vector<PostProcessPlugin::Ptr> post_processors;
    PluginManager::getPluginsByType(post_processors);

    if (post_processors.empty())
      ROS_INFO("Finalize Step Plan: No post processing plugins loaded. Skipping post processing!");
    else
      ROS_INFO("Finalize Step Plan: Found following post processing plugins:");

    for (std::vector<PostProcessPlugin::Ptr>::const_iterator itr = post_processors.begin(); itr != post_processors.end(); itr++)
    {
      const PostProcessPlugin::Ptr& pp_plugin = *itr;
      if (pp_plugin)
      {
        ROS_INFO("    %s (%s)", pp_plugin->getName().c_str(), pp_plugin->getTypeId().c_str());
        pp_plugin->postProcess(resp.step_plan);
      }
    }
  }

  // add start and goal configuration
  resp.step_plan.start.header = resp.step_plan.header;

  ivStartFootLeft.getFoot(resp.step_plan.start.left);
  resp.step_plan.start.left.foot_index = msgs::Foot::LEFT;
  resp.step_plan.start.left.header = resp.step_plan.header;
  foot_pose_transformer.transformToRobotFrame(resp.step_plan.start.left);

  ivStartFootRight.getFoot(resp.step_plan.start.right);
  resp.step_plan.start.right.foot_index = msgs::Foot::RIGHT;
  resp.step_plan.start.right.header = resp.step_plan.header;
  foot_pose_transformer.transformToRobotFrame(resp.step_plan.start.right);

  resp.step_plan.goal.header = resp.step_plan.header;

  ivGoalFootLeft.getFoot(resp.step_plan.goal.left);
  resp.step_plan.goal.left.foot_index = msgs::Foot::LEFT;
  resp.step_plan.goal.left.header = resp.step_plan.header;
  foot_pose_transformer.transformToRobotFrame(resp.step_plan.goal.left);

  ivGoalFootRight.getFoot(resp.step_plan.goal.right);
  resp.step_plan.goal.right.foot_index = msgs::Foot::RIGHT;
  resp.step_plan.goal.right.header = resp.step_plan.header;
  foot_pose_transformer.transformToRobotFrame(resp.step_plan.goal.right);

  // plan validation and computation of final cost
  if (req.plan_request.planning_mode == msgs::StepPlanRequest::PLANNING_MODE_PATTERN)
    updateStepPlan(resp.step_plan, msgs::UpdateMode::UPDATE_MODE_CHECK_VALIDITY | msgs::UpdateMode::UPDATE_MODE_CHECK_COLLISION |  msgs::UpdateMode::UPDATE_MODE_COST, std::string(), false);
  else
    updateStepPlan(resp.step_plan, msgs::UpdateMode::UPDATE_MODE_COST, std::string(), false);

  // transform step plan
  foot_pose_transformer.transformToRobotFrame(resp.step_plan);

  resp.final_eps = ivPlannerPtr->get_final_epsilon();
  resp.planning_time = ivPlannerPtr->get_final_eps_planning_time();

  if (resp.status.error == msgs::ErrorStatus::NO_ERROR && resp.final_eps > 1.4)
    resp.status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlanner", "stepPlanRequestService: Suboptimal plan (eps: " + boost::lexical_cast<std::string>(resp.final_eps) + ")!");

  // some debug outputs and visualization stuff
  for (std::vector<msgs::Step>::const_iterator itr = resp.step_plan.steps.begin(); itr != resp.step_plan.steps.end(); itr++)
  {
    step = *itr;

    geometry_msgs::Vector3 n;
    quaternionToNormal(step.foot.pose.orientation, n);
    ROS_INFO("[%i] x/y/z: %f/%f/%f", step.step_index, step.foot.pose.position.x, step.foot.pose.position.y, step.foot.pose.position.z);
    ROS_INFO("[%i] n: %f/%f/%f", step.step_index, n.x, n.y, n.z);
    ROS_INFO("[%i] step duration: %f, swing height: %f", step.step_index, step.step_duration, step.swing_height);
    ROS_INFO("[%i] valid: %s, colliding: %s", step.step_index, step.valid ? "y" : "n", step.colliding ? "y" : "n");
    ROS_INFO("[%i] cost: %f risk: %f", step.step_index, step.cost, step.risk);

    if (WorldModel::isTerrainModelAvailable())
    {
      double support = 0.0;
      WorldModel::getTerrainModel()->getFootContactSupport(step.foot.pose, support, ivCheckedFootContactSupport);
      ROS_INFO("[%i] Ground contact support: %f", step.step_index, support);
    }

    ROS_INFO("-------------------------------------");
  }

  if (ivCheckedFootContactSupportPub.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*ivCheckedFootContactSupport, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();
    ivCheckedFootContactSupportPub.publish(msg);
  }

  return true;
}

msgs::ErrorStatus FootstepPlanner::stepPlanRequest(msgs::StepPlanRequestService::Request& req, ResultCB result_cb, FeedbackCB feedback_cb, PreemptCB preempt_cb)
{
  // preempt any planning
  if (isPlanning())
    preemptPlanning();

  this->result_cb = result_cb;
  this->feedback_cb = feedback_cb;
  this->preempt_cb = preempt_cb;

  // check input
  // strip '/'
  std::string request_frame_id = strip_const(req.plan_request.header.frame_id, '/');
  std::string start_frame_id = strip_const(req.plan_request.start.header.frame_id, '/');
  std::string goal_frame_id = strip_const(req.plan_request.goal.header.frame_id, '/');

  if (request_frame_id != start_frame_id)
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "stepPlanRequest: Frame IDs of plan request ('" + request_frame_id + ") and start pose ('" + start_frame_id + "') do not match!");

  if (req.plan_request.planning_mode != msgs::StepPlanRequest::PLANNING_MODE_PATTERN && request_frame_id != goal_frame_id)
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "stepPlanRequest: Frame IDs of plan request ('" + request_frame_id + "') and goal pose ('" + goal_frame_id + "') do not match!");

  // load parameter set, if given
  if (!req.plan_request.parameter_set_name.data.empty())
  {
    if (!ParameterManager::setActive(req.plan_request.parameter_set_name.data))
      return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_PARAMETERS, "FootstepPlanner", "stepPlanRequest: Can't find parameter set named '" + req.plan_request.parameter_set_name.data + "'!");

    setParams(ParameterManager::getActive());
  }
  else
  {
    // just reset planner
    reset();
  }

  // transform feet poses
  foot_pose_transformer.transformToPlannerFrame(req.plan_request.start);
  foot_pose_transformer.transformToPlannerFrame(req.plan_request.goal);

  // set request specific parameters
  start_foot_selection = req.plan_request.start_foot_selection;
  max_number_steps = req.plan_request.max_number_steps;
  max_path_length_ratio = req.plan_request.max_path_length_ratio;
  frame_id = req.plan_request.header.frame_id;
  ivPlannerEnvironmentPtr->setFrameId(frame_id);

  // start planning
  startPlanning(req);

  return msgs::ErrorStatus();
}

bool FootstepPlanner::stepPlanRequestService(msgs::StepPlanRequestService::Request& req, msgs::StepPlanRequestService::Response& resp)
{
  // generate step plan based on request
  resp.status += stepPlanRequest(req);

  if (hasError(resp.status))
    return true; // return always true so the message is returned

  // wait for thread to terminate
  if (planning_thread.joinable())
    planning_thread.join();

  // finalize plan and generate response
  finalizeStepPlan(req, resp, true);

  return true; // return always true so the message is returned
}

void FootstepPlanner::startPlanning(msgs::StepPlanRequestService::Request& req)
{
  // start planning in seperate thread
  planning_thread = boost::thread(&FootstepPlanner::doPlanning, this, req);
}

void FootstepPlanner::doPlanning(msgs::StepPlanRequestService::Request& req)
{
  // lock entire planning system
  boost::recursive_mutex::scoped_lock lock(planner_mutex);

  msgs::StepPlanRequestService::Response resp;

  // set world model mode
  if (req.plan_request.planning_mode == msgs::StepPlanRequest::PLANNING_MODE_3D)
    WorldModel::useTerrainModel(true);
  else if (req.plan_request.planning_mode == msgs::StepPlanRequest::PLANNING_MODE_PATTERN)
    WorldModel::useTerrainModel(req.plan_request.pattern_parameters.use_terrain_model);
  else
    WorldModel::useTerrainModel(false);

  // dispatch planning mode and plan
  switch (req.plan_request.planning_mode)
  {
    case msgs::StepPlanRequest::PLANNING_MODE_2D:
    case msgs::StepPlanRequest::PLANNING_MODE_3D:
      resp.status = planSteps(req);
      break;
    case msgs::StepPlanRequest::PLANNING_MODE_PATTERN:
      resp.status = planPattern(req);
      break;
    default:
      resp.status = ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlanner", "stepPlanRequest: A not supported planning mode '" + boost::lexical_cast<std::string>(req.plan_request.planning_mode) + "' was given!");
      break;
  }

  // result callbacks
  if (!result_cb.empty())
  {
    // finalize plan and generate response
    if (!hasError(resp.status))
      finalizeStepPlan(req, resp, true);
    result_cb(resp);
  }
}

void FootstepPlanner::preemptPlanning()
{
  if (!isPlanning())
    return;

  if (planning_thread.joinable())
  {
    planning_thread.interrupt();
    planning_thread.join();
  }

  if (!preempt_cb.empty())
    preempt_cb();
}

bool FootstepPlanner::findNearestValidState(State& s) const
{
  if (WorldModel::isAccessible(s))
    return true;

  State current_state = s;
  State best_state = s;

  double pos_diff = FLT_MAX;
  double yaw_diff = FLT_MAX;
  bool solution_found = false;

  // get transformation foot -> world
  tf::Transform t;
  t.setOrigin(s.getPose().getOrigin());
  t.setBasis(s.getPose().getBasis());

  tf::Vector3 orig_pos;
  tf::Vector3 trans_pos;
  orig_pos.setZ(0.0);

  for (double yaw = -0.2; yaw <= 0.4; yaw+=env_params->angle_bin_size)
  {
    current_state.setYaw(s.getYaw() + (s.getLeg() == LEFT ? yaw : -yaw));
    for (double y = -0.05; y <= 0.2; y+=env_params->cell_size)
    {
      orig_pos.setY(s.getLeg() == LEFT ? y : -y);
      for (double x = -0.15; x <= 0.15; x+=env_params->cell_size)
      {
        // determine point in world frame and get height at this point
        orig_pos.setX(x);
        trans_pos = t * orig_pos;

        current_state.setX(trans_pos.getX());
        current_state.setY(trans_pos.getY());

        if (!WorldModel::update3DData(current_state))
          continue;

        if (!WorldModel::isAccessible(current_state))
          continue;

        double dist = std::sqrt(x*x + y*y);
        if (pos_diff >= dist && yaw_diff >= std::abs(yaw))
        {
          best_state = current_state;
          pos_diff = dist;
          yaw_diff = std::abs(yaw);
          solution_found = true;
        }
      }
    }
  }

  if (solution_found)
    s = best_state;

  return solution_found;
}

bool FootstepPlanner::checkRobotCollision(const State& left_foot, const State& right_foot, bool& left, bool& right) const
{
  left = !WorldModel::isAccessible(left_foot);
  right = !WorldModel::isAccessible(right_foot);

  if (!left && !right && !WorldModel::isAccessible(left_foot, right_foot))
  {
    left = true;
    right = true;
  }

  return left || right;
}

bool FootstepPlanner::setStart(const State& left_foot, const State& right_foot, bool ignore_collision)
{
  // check for errors
  if (std::isnan(left_foot.getRoll())) return false;
  if (std::isnan(left_foot.getPitch())) return false;
  if (std::isnan(left_foot.getYaw())) return false;
  if (std::isnan(right_foot.getRoll())) return false;
  if (std::isnan(right_foot.getPitch())) return false;
  if (std::isnan(right_foot.getYaw())) return false;

  bool left_collision = false;
  bool right_collision = false;

  if (!ignore_collision && checkRobotCollision(left_foot, right_foot, left_collision, right_collision))
  {
    start_pose_set_up = false;
    return false;
  }
  else
    start_pose_set_up = true;

  ivStartFootLeft = left_foot;
  ivStartFootRight = right_foot;

  ROS_INFO("Start foot poses set to (left: %f %f %f %f) and (right: %f %f %f %f)",
           ivStartFootLeft.getX(), ivStartFootLeft.getY(), ivStartFootLeft.getZ(), ivStartFootLeft.getYaw(),
           ivStartFootRight.getX(), ivStartFootRight.getY(), ivStartFootRight.getZ(), ivStartFootRight.getYaw());

  return true;
}

bool FootstepPlanner::setStart(const msgs::StepPlanRequest& req, bool ignore_collision)
{
  State left_foot(req.start.left, env_params->swing_height, env_params->step_duration);
  State right_foot(req.start.right, env_params->swing_height, env_params->step_duration);

  if (req.planning_mode == msgs::StepPlanRequest::PLANNING_MODE_2D)
  {
    double z = 0.5*(req.start.left.pose.position.z + req.start.right.pose.position.z);
    left_foot.setZ(z);
    right_foot.setZ(z);
  }

  return setStart(left_foot, right_foot, ignore_collision);
}

bool FootstepPlanner::setGoal(const State& left_foot, const State& right_foot, bool ignore_collision)
{
  // check for errors
  if (std::isnan(left_foot.getRoll())) return false;
  if (std::isnan(left_foot.getPitch())) return false;
  if (std::isnan(left_foot.getYaw())) return false;
  if (std::isnan(right_foot.getRoll())) return false;
  if (std::isnan(right_foot.getPitch())) return false;
  if (std::isnan(right_foot.getYaw())) return false;

  bool left_collision = false;
  bool right_collision = false;

  if (!ignore_collision && checkRobotCollision(left_foot, right_foot, left_collision, right_collision))
  {
    goal_pose_set_up = false;
    return false;
  }
  else
    goal_pose_set_up = true;

  ivGoalFootLeft = left_foot;
  ivGoalFootRight = right_foot;

  ROS_INFO("Goal foot poses set to (left: %f %f %f %f) and (right: %f %f %f %f)",
           ivGoalFootLeft.getX(), ivGoalFootLeft.getY(), ivGoalFootLeft.getZ(), ivGoalFootLeft.getYaw(),
           ivGoalFootRight.getX(), ivGoalFootRight.getY(), ivGoalFootRight.getZ(), ivGoalFootRight.getYaw());

  return true;
}

bool FootstepPlanner::setGoal(const msgs::StepPlanRequest& req, bool ignore_collision)
{
  State left_foot(req.goal.left, env_params->swing_height, env_params->step_duration);
  State right_foot(req.goal.right, env_params->swing_height, env_params->step_duration);

  if (req.planning_mode == msgs::StepPlanRequest::PLANNING_MODE_2D)
  {
    double z = 0.5*(req.start.left.pose.position.z + req.start.right.pose.position.z);
    geometry_msgs::Vector3 n;

    left_foot.setZ(z);
    quaternionToNormal(req.start.left.pose.orientation, n);
    left_foot.setNormal(n);

    right_foot.setZ(z);
    quaternionToNormal(req.start.right.pose.orientation, n);
    right_foot.setNormal(n);
  }

  return setGoal(left_foot, right_foot, ignore_collision);
}

State FootstepPlanner::getFootPose(const State& robot, Leg leg, double dx, double dy, double dyaw)
{
  double sign = -1.0;
  if (leg == LEFT)
    sign = 1.0;

  double cos_theta = cos(robot.getYaw());
  double sin_theta = sin(robot.getYaw());
  double shift_x = cos_theta * sign * dx - sin_theta * (0.5 * env_params->foot_seperation + dy);
  double shift_y = sin_theta * sign * dx + cos_theta * (0.5 * env_params->foot_seperation + dy);

  State foot(robot.getX() + sign * shift_x,
             robot.getY() + sign * shift_y,
             robot.getZ(),
             robot.getRoll(),
             robot.getPitch(),
             robot.getYaw() + sign * dyaw,
             robot.getSwingHeight(),
             robot.getStepDuration(),
             leg);

  WorldModel::update3DData(foot);

  return foot;
}

State FootstepPlanner::getFootPose(const State& robot, Leg leg)
{
  return getFootPose(robot, leg, 0.0, 0.0, 0.0);
}

State FootstepPlanner::getParallelFootPose(const State& foot, double additional_seperation)
{
  double shift_x = -sin(foot.getYaw()) * (env_params->foot_seperation + additional_seperation);
  double shift_y =  cos(foot.getYaw()) * (env_params->foot_seperation + additional_seperation);

  double sign = -1.0;
  if (foot.getLeg() == RIGHT)
    sign = 1.0;

  State foot_parallel(foot.getX() + sign * shift_x,
                      foot.getY() + sign * shift_y,
                      foot.getZ(),
                      foot.getRoll(),
                      foot.getPitch(),
                      foot.getYaw(),
                      foot.getSwingHeight(),
                      foot.getStepDuration(),
                      foot.getLeg() == RIGHT ? LEFT : RIGHT);

  WorldModel::update3DData(foot_parallel);

  return foot_parallel;
}

bool FootstepPlanner::pathIsNew(const std::vector<int>& new_path)
{
  if (new_path.size() != ivPlanningStatesIds.size())
    return true;

  bool unequal = true;
  for (unsigned i = 0; i < new_path.size(); ++i)
    unequal = new_path[i] != ivPlanningStatesIds[i] && unequal;

  return unequal;
}
}
