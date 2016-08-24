#include <vigir_footstep_planner/footstep_planner_node.h>

#include <vigir_footstep_planning_lib/helper.h>
#include <vigir_footstep_planning_lib/visualization/footstep_planning_vis.h>

#include <vigir_footstep_planning_plugins/plugins/state_generator_plugin.h>
#include <vigir_footstep_planning_plugins/plugins/robot_model_plugin.h>
#include <vigir_footstep_planning_plugins/plugins/step_plan_msg_plugin.h>
#include <vigir_footstep_planning_plugins/plugins/collision_check_plugin.h>
#include <vigir_footstep_planning_plugins/plugins/terrain_model_plugin.h>



namespace vigir_footstep_planning
{
FootstepPlannerNode::FootstepPlannerNode(ros::NodeHandle& nh)
{
  initPlugins(nh);
  init(nh);
}

void FootstepPlannerNode::initPlugins(ros::NodeHandle& nh)
{
  vigir_pluginlib::PluginManager::addPluginClassLoader<StateGeneratorPlugin>("vigir_footstep_planning_plugins", "vigir_footstep_planning::StateGeneratorPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<StepPlanMsgPlugin>("vigir_footstep_planning_plugins", "vigir_footstep_planning::StepPlanMsgPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<ReachabilityPlugin>("vigir_footstep_planning_plugins", "vigir_footstep_planning::ReachabilityPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<StepCostEstimatorPlugin>("vigir_footstep_planning_plugins", "vigir_footstep_planning::StepCostEstimatorPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<HeuristicPlugin>("vigir_footstep_planning_plugins", "vigir_footstep_planning::HeuristicPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<PostProcessPlugin>("vigir_footstep_planning_plugins", "vigir_footstep_planning::PostProcessPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<CollisionCheckPlugin>("vigir_footstep_planning_plugins", "vigir_footstep_planning::CollisionCheckPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<TerrainModelPlugin>("vigir_footstep_planning_plugins", "vigir_footstep_planning::TerrainModelPlugin");

  /** No need to load plugin set here as it will be done in the constructor of FootstepPlanner */
  //vigir_pluginlib::PluginManager::addPlugin(new RobotModelPlugin(nh));
}

void FootstepPlannerNode::init(ros::NodeHandle& nh)
{
  getFootSize(nh, foot_size);

  // init planner
  footstep_planner.reset(new FootstepPlanner(nh));

  // subscribe topics
  set_active_parameter_set_sub = nh.subscribe<std_msgs::String>("set_active_parameter_set", 1, &FootstepPlannerNode::setParams, this);
  step_plan_request_sub = nh.subscribe("step_plan_request", 1, &FootstepPlannerNode::stepPlanRequest, this);
  goal_pose_sub = nh.subscribe("/goal_pose", 1, &FootstepPlannerNode::goalPoseCallback, this);

  // publish topics
  step_plan_pub = nh.advertise<msgs::StepPlan>("step_plan", 1);
  step_plan_request_vis_pub = nh.advertise<msgs::StepPlanRequest>("vis/step_plan_request", 1);
  step_plan_vis_pub = nh.advertise<msgs::StepPlan>("vis/step_plan", 1);
  error_status_pub = nh.advertise<msgs::ErrorStatus>("error_status", 1);
  temp_step_plan_pub = nh.advertise<msgs::StepPlan>("temp_step_plan", 1);
  feedback_pub = nh.advertise<msgs::PlanningFeedback>("planning_feedback", 1);

  // start service clients
  generate_feet_pose_client = nh.serviceClient<msgs::GenerateFeetPoseService>("generate_feet_pose");

  // start own services
  step_plan_request_srv = nh.advertiseService("step_plan_request", &FootstepPlannerNode::stepPlanRequestService, this);
  update_foot_srv = nh.advertiseService("update_foot", &FootstepPlannerNode::updateFootService, this);
  update_feet_srv = nh.advertiseService("update_feet", &FootstepPlannerNode::updateFeetService, this);
  update_step_plan_srv = nh.advertiseService("update_step_plan", &FootstepPlannerNode::updateStepPlanService, this);

  // init action servers
  step_plan_request_as = SimpleActionServer<msgs::StepPlanRequestAction>::create(nh, "step_plan_request", true, boost::bind(&FootstepPlannerNode::stepPlanRequestAction, this, boost::ref(step_plan_request_as))
                                                                                                              , boost::bind(&FootstepPlannerNode::stepPlanRequestPreempt, this, boost::ref(step_plan_request_as)));
  update_foot_as = SimpleActionServer<msgs::UpdateFootAction>::create(nh, "update_foot", true, boost::bind(&FootstepPlannerNode::updateFootAction, this, boost::ref(update_foot_as)));
  update_feet_as = SimpleActionServer<msgs::UpdateFeetAction>::create(nh, "update_feet", true, boost::bind(&FootstepPlannerNode::updateFeetAction, this, boost::ref(update_feet_as)));
  update_step_plan_as = SimpleActionServer<msgs::UpdateStepPlanAction>::create(nh, "update_step_plan", true, boost::bind(&FootstepPlannerNode::updateStepPlanAction, this, boost::ref(update_step_plan_as)));
}

FootstepPlannerNode::~FootstepPlannerNode()
{
}

// --- Callbacks ---

void FootstepPlannerNode::planningResultCallback(const msgs::StepPlanRequestService::Response& resp)
{
  // check result
  if (hasError(resp.status))
  {
    ROS_ERROR("[FootstepPlannerNode] Error while planning steps:\n%s", toString(resp.status).c_str());
    return;
  }
  else if (hasWarning(resp.status))
    ROS_WARN("[FootstepPlannerNode] Warning occured:\n%s", toString(resp.status).c_str());

  // publish and visualize plan
  step_plan_pub.publish(resp.step_plan);
  temp_step_plan_pub.publish(resp.step_plan);
  step_plan_vis_pub.publish(resp.step_plan);
  error_status_pub.publish(resp.status);
}

void FootstepPlannerNode::planningResultActionCallback(const msgs::StepPlanRequestService::Response& resp, SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex);

  // finish action server
  msgs::StepPlanRequestResult result;

  result.step_plan = resp.step_plan;
  result.status = resp.status;
  result.final_eps = resp.final_eps;
  result.planning_time = resp.planning_time;

  as->finish(result);

  // check result
  if (hasError(resp.status))
  {
    ROS_ERROR("[FootstepPlannerNode] Error while planning steps:\n%s", toString(resp.status).c_str());
    return;
  }
  else if (hasWarning(resp.status))
    ROS_WARN("[FootstepPlannerNode] Warning occured:\n%s", toString(resp.status).c_str());

  // publish and visualize plan
  temp_step_plan_pub.publish(resp.step_plan);
  step_plan_vis_pub.publish(resp.step_plan);
  error_status_pub.publish(resp.status);
}

void FootstepPlannerNode::planningFeedbackCallback(const msgs::PlanningFeedback& feedback)
{
  feedback_pub.publish(feedback);
}

void FootstepPlannerNode::planningFeedbackActionCallback(const msgs::PlanningFeedback& feedback, SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  planningFeedbackCallback(feedback);

  msgs::StepPlanRequestFeedback fb;
  fb.feedback = feedback;
  as->publishFeedback(fb);
}

void FootstepPlannerNode::planningPreemptionActionCallback(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex);

  if (as->isActive())
    as->setPreempted();
}

// --- Subscriber calls ---

void FootstepPlannerNode::setParams(const std_msgs::StringConstPtr& params_name)
{
  vigir_generic_params::ParameterSet params;

  if (!ParameterManager::getParameterSet(params_name->data, params))
    ROS_ERROR("[FootstepPlannerNode] setParams: Unknown parameter set '%s'!", params_name->data.c_str());
  else if (!footstep_planner->setParams(params))
    ROS_ERROR("[FootstepPlannerNode] setParams: Couldn't set parameter set '%s'!", params_name->data.c_str());
  else
    ParameterManager::setActive(params_name->data);
}

void FootstepPlannerNode::stepPlanRequest(const msgs::StepPlanRequestConstPtr &plan_request)
{
  msgs::ErrorStatus status;
  msgs::StepPlanRequestService::Request step_plan_request;
  step_plan_request.plan_request = *plan_request;

  // generate start feet pose if needed
  if (step_plan_request.plan_request.start.header.frame_id.empty())
  {
    status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlannerNode", "stepPlanRequest: No valid frame_id was given as start pose. Try to use current robot pose as start.");
    status += determineStartFeetPose(step_plan_request.plan_request.start, generate_feet_pose_client, step_plan_request.plan_request.header);

    if (hasError(status))
    {
      ROS_WARN("[FootstepPlannerNode] Can't obtain start feet pose:\n%s", toString(status).c_str());
      return;
    }
    else if (hasWarning(status))
      ROS_WARN("[FootstepPlannerNode] Warning occured while obtaining start feet pose:\n%s", toString(status).c_str());
  }

  step_plan_request_vis_pub.publish(step_plan_request.plan_request);

  // start planning
  status = footstep_planner->stepPlanRequest(step_plan_request, boost::bind(&FootstepPlannerNode::planningResultCallback, this, _1)
                                                              , boost::bind(&FootstepPlannerNode::planningFeedbackCallback, this, _1));

  if (!isOk(status))
    ROS_INFO("[FootstepPlannerNode] stepPlanRequest:\n%s", toString(status).c_str());
}

void FootstepPlannerNode::goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_pose)
{
  // get start feet pose
  msgs::Feet start_feet_pose;
  msgs::ErrorStatus status = determineStartFeetPose(start_feet_pose, generate_feet_pose_client, goal_pose->header);

  //start_feet_pose.left.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -0.1, -0.05);
  //start_feet_pose.right.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -0.1, 0.05);

  if (hasError(status))
  {
    ROS_WARN("[FootstepPlannerNode] Can't obtain start feet pose:\n%s", toString(status).c_str());
    return;
  }
  else if (hasWarning(status))
    ROS_WARN("[FootstepPlannerNode] Warning occured while obtaining start feet pose:\n%s", toString(status).c_str());

  // get goal feet pose
  msgs::Feet goal_feet_pose;
  goal_feet_pose.left.pose.position.z = goal_feet_pose.right.pose.position.z = start_feet_pose.left.pose.position.z;
  status = determineGoalFeetPose(goal_feet_pose, generate_feet_pose_client, *goal_pose);

  if (hasError(status))
  {
    ROS_WARN("[FootstepPlannerNode] Can't obtain goal feet pose:\n%s", toString(status).c_str());
    return;
  }
  else if (hasWarning(status))
    ROS_WARN("[FootstepPlannerNode] Warning occured while obtaining goal feet pose:\n%s", toString(status).c_str());

  footstep_planner->updateFeet(goal_feet_pose, msgs::UpdateMode::UPDATE_MODE_MOVE_TO_VALID);

  // request step plan
  msgs::StepPlanRequestService::Request step_plan_request;
  step_plan_request.plan_request.header = goal_pose->header;
  step_plan_request.plan_request.start = start_feet_pose;
  step_plan_request.plan_request.goal = goal_feet_pose;
  step_plan_request.plan_request.start_step_index = 0;
  step_plan_request.plan_request.start_foot_selection = msgs::StepPlanRequest::AUTO;
  step_plan_request.plan_request.planning_mode = WorldModel::instance().isTerrainModelAvailable() ? static_cast<uint8_t>(msgs::StepPlanRequest::PLANNING_MODE_3D)
                                                                                       : static_cast<uint8_t>(msgs::StepPlanRequest::PLANNING_MODE_2D);
  step_plan_request.plan_request.max_planning_time = 0.0;
  step_plan_request.plan_request.max_number_steps = 0.0;
  step_plan_request.plan_request.max_path_length_ratio = 0.0;
  step_plan_request.plan_request.parameter_set_name.data = std::string();

  // visualize request
  step_plan_request_vis_pub.publish(step_plan_request.plan_request);

  // start planning
  status = footstep_planner->stepPlanRequest(step_plan_request, boost::bind(&FootstepPlannerNode::planningResultCallback, this, _1)
                                                              , boost::bind(&FootstepPlannerNode::planningFeedbackCallback, this, _1));

  if (!isOk(status))
    ROS_INFO("[FootstepPlannerNode] goalPoseCallback:\n%s", toString(status).c_str());
}

// --- service calls ---

bool FootstepPlannerNode::stepPlanRequestService(msgs::StepPlanRequestService::Request &req, msgs::StepPlanRequestService::Response &resp)
{
  // generate start feet pose if needed
  if (req.plan_request.start.header.frame_id.empty())
  {
    resp.status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlannerNode", "stepPlanRequestService: No valid frame_id was given as start pose. Try to use current robot pose as start.");
    resp.status += determineStartFeetPose(req.plan_request.start, generate_feet_pose_client, req.plan_request.header);
  }

  step_plan_request_vis_pub.publish(req.plan_request);

  // start planning
  if (!footstep_planner->stepPlanRequestService(req, resp))
    resp.status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "FootstepPlannerNode", "stepPlanRequestService: Can't call footstep planner service!");

  temp_step_plan_pub.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plan)));
  step_plan_vis_pub.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plan)));
  error_status_pub.publish(msgs::ErrorStatusConstPtr(new msgs::ErrorStatus(resp.status)));

  return true; // return always true so the message is returned
}

bool FootstepPlannerNode::updateFootService(msgs::UpdateFootService::Request &req, msgs::UpdateFootService::Response &resp)
{
  resp.foot = req.foot;
  resp.status = footstep_planner->updateFoot(resp.foot, req.update_mode.mode);
  return true; // return always true so the message is returned
}

bool FootstepPlannerNode::updateFeetService(msgs::UpdateFeetService::Request &req, msgs::UpdateFeetService::Response &resp)
{
  resp.feet = req.feet;
  resp.status = footstep_planner->updateFeet(resp.feet, req.update_mode.mode);
  return true; // return always true so the message is returned
}

bool FootstepPlannerNode::updateStepPlanService(msgs::UpdateStepPlanService::Request &req, msgs::UpdateStepPlanService::Response &resp)
{
  resp.step_plan = req.step_plan;
  resp.status = footstep_planner->updateStepPlan(resp.step_plan, req.update_mode.mode, req.parameter_set_name.data);
  step_plan_vis_pub.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(resp.step_plan)));
  return true; // return always true so the message is returned
}

//--- action server calls ---

void FootstepPlannerNode::stepPlanRequestAction(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  // preempt any previous goal if active due to given callback
  footstep_planner->preemptPlanning();

  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex);

  // accept new goal
  const msgs::StepPlanRequestGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::ErrorStatus status;
  msgs::StepPlanRequestService::Request step_plan_request;
  step_plan_request.plan_request = goal->plan_request;

  // generate start feet pose if needed
  if (step_plan_request.plan_request.start.header.frame_id.empty())
  {
    status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "FootstepPlannerNode", "stepPlanRequestAction: No valid frame_id was given as start pose. Try to use current robot pose as start.");
    status += determineStartFeetPose(step_plan_request.plan_request.start, generate_feet_pose_client, step_plan_request.plan_request.header);
  }

  step_plan_request_vis_pub.publish(step_plan_request.plan_request);

  // start planning
  status = footstep_planner->stepPlanRequest(step_plan_request, boost::bind(&FootstepPlannerNode::planningResultActionCallback, this, _1, boost::ref(as))
                                                              , boost::bind(&FootstepPlannerNode::planningFeedbackActionCallback, this, _1, boost::ref(as))
                                                              , boost::bind(&FootstepPlannerNode::planningPreemptionActionCallback, this, boost::ref(as)));

  if (!isOk(status))
    ROS_INFO("[FootstepPlannerNode] stepPlanRequest:\n%s", toString(status).c_str());
}

void FootstepPlannerNode::stepPlanRequestPreempt(SimpleActionServer<msgs::StepPlanRequestAction>::Ptr& as)
{
  boost::recursive_mutex::scoped_lock lock(step_plan_request_as_mutex);

  if (as->isActive())
  {
    footstep_planner->preemptPlanning();
    as->setPreempted();
  }
}

void FootstepPlannerNode::updateFootAction(SimpleActionServer<msgs::UpdateFootAction>::Ptr& as)
{
  const msgs::UpdateFootGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::UpdateFootResult result;
  result.foot = goal->foot;
  result.status = footstep_planner->updateFoot(result.foot, goal->update_mode.mode);

  as->finish(result);
}

void FootstepPlannerNode::updateFeetAction(SimpleActionServer<msgs::UpdateFeetAction>::Ptr& as)
{
  const msgs::UpdateFeetGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::UpdateFeetResult result;
  result.feet = goal->feet;
  result.status = footstep_planner->updateFeet(result.feet, goal->update_mode.mode);

  as->finish(result);
}

void FootstepPlannerNode::updateStepPlanAction(SimpleActionServer<msgs::UpdateStepPlanAction>::Ptr& as)
{
  const msgs::UpdateStepPlanGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::UpdateStepPlanResult result;

  result.step_plan = goal->step_plan;
  result.status = footstep_planner->updateStepPlan(result.step_plan, goal->update_mode.mode, goal->parameter_set_name.data);
  step_plan_vis_pub.publish(msgs::StepPlanConstPtr(new msgs::StepPlan(result.step_plan)));

  as->finish(result);
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vigir_footstep_planner");

  ros::NodeHandle nh;

  // ensure that node's services are set up in proper namespace
  if (nh.getNamespace().size() <= 1)
    nh = ros::NodeHandle("~");

  // init parameter and plugin manager
  vigir_generic_params::ParameterManager::initialize(nh);
  vigir_pluginlib::PluginManager::initialize(nh);

  // init footstep planner
  vigir_footstep_planning::FootstepPlannerNode footstep_planner_node(nh);

  ros::spin();

  return 0;
}
