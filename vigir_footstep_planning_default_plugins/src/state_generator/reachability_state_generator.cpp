#include <vigir_footstep_planning_default_plugins/state_generator/reachability_state_generator.h>

#include <vigir_footstep_planning_lib/math.h>

#include <vigir_footstep_planning_plugins/plugin_aggregators/robot_model.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/world_model.h>



namespace vigir_footstep_planning
{
ReachabilityStateGenerator::ReachabilityStateGenerator()
  : StateGeneratorPlugin("reachability_state_generator")
{
}

bool ReachabilityStateGenerator::loadParams(const vigir_generic_params::ParameterSet& global_params)
{
  if (!StateGeneratorPlugin::loadParams(global_params))
    return false;

  bool result = true;

  int threads;
  unsigned int jobs_per_thread;
  result &= global_params.getParam("threads", threads);
  result &= global_params.getParam("jobs_per_thread", jobs_per_thread);


  int hash_table_size;
  double cell_size;
  int num_angle_bins;
  result &= global_params.getParam("max_hash_size", hash_table_size);
  result &= global_params.getParam("collision_check/cell_size", cell_size);
  result &= global_params.getParam("collision_check/num_angle_bins", num_angle_bins);
  double angle_bin_size = 2.0*M_PI / static_cast<double>(num_angle_bins);


  global_params.getParam("max_risk", max_risk_, 1.0);



  int ivMaxStepRangeX, ivMaxStepRangeY, ivMaxStepRangeTheta;
  int ivMaxInvStepRangeX, ivMaxInvStepRangeY, ivMaxInvStepRangeTheta;

  /// Defines the area of performable (discrete) steps.
  std::vector<std::pair<int, int> > step_range;

  // step range
  XmlRpc::XmlRpcValue step_range_x;
  XmlRpc::XmlRpcValue step_range_y;
  if (global_params.getParam("step_range/x", step_range_x) && global_params.getParam("step_range/y", step_range_y))
  {
    // create step range
    step_range.clear();
    step_range.reserve(step_range_x.size());
    double max_x = (double)step_range_x[0];
    double max_y = (double)step_range_y[0];
    double max_inv_x = (double)step_range_x[0];
    double max_inv_y = (double)step_range_y[0];
    for (int i=0; i < step_range_x.size(); ++i)
    {
      double x = (double)step_range_x[i];
      double y = (double)step_range_y[i];

      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
      max_inv_x = std::min(max_inv_x, x);
      max_inv_y = std::min(max_inv_y, y);

      step_range.push_back(std::pair<int, int>(disc_val(x, cell_size), disc_val(y, cell_size)));
    }

    double max_step_range_theta;
    double max_inverse_step_range_theta;
    result &= global_params.getParam("foot/max/step/theta", max_step_range_theta);
    result &= global_params.getParam("foot/max/inverse/step/theta", max_inverse_step_range_theta);

    ivMaxStepRangeX = disc_val(max_x, cell_size);
    ivMaxStepRangeY = disc_val(max_y, cell_size);
    ivMaxStepRangeTheta = angle_state_2_cell(max_step_range_theta, angle_bin_size);
    ivMaxInvStepRangeX = disc_val(max_inv_x, cell_size);
    ivMaxInvStepRangeY = disc_val(max_inv_y, cell_size);
    ivMaxInvStepRangeTheta = angle_state_2_cell(max_inverse_step_range_theta, angle_bin_size);
  }


  // we can't proceed if parameters are missing
  if (!result)
    return false;

  // setup state expansion manager
  expand_states_manager.reset(new threading::ThreadingManager<threading::ExpandStateJob>(threads, jobs_per_thread));


  // determine whether a (x,y) translation can be performed by the robot by
  // checking if it is within a certain area of performable steps
  for (int y = ivMaxInvStepRangeY; y <= ivMaxStepRangeY; y++)
  {
    for (int x = ivMaxInvStepRangeX; x <= ivMaxStepRangeX; x++)
    {
      bool in_step_range = pointWithinPolygon(x, y, step_range);

      if (!in_step_range)
        continue;

      // generate area of samplings for gpr/map-based planning
      for (int theta = ivMaxInvStepRangeTheta; theta <= ivMaxStepRangeTheta; theta++)
      {
        Footstep f(cont_val(x, cell_size), cont_val(y, cell_size), angle_cell_2_state(theta, angle_bin_size), 0.0, cell_size, num_angle_bins, hash_table_size);
        ivContFootstepSet.push_back(f);
      }
    }
  }

  return result;
}

std::list<PlanningState::Ptr> ReachabilityStateGenerator::generatePredecessor(const PlanningState& state) const
{
  std::list<PlanningState::Ptr> result;

  ROS_ERROR("[ReachabilityStateGenerator] generatePredecessor not implemented yet!");

  return result;
}

std::list<PlanningState::Ptr> ReachabilityStateGenerator::generateSuccessor(const PlanningState& state) const
{
  std::list<PlanningState::Ptr> result;

  // explorate all state
  std::list<threading::ExpandStateJob::Ptr> jobs;
  for (const Footstep& footstep : ivContFootstepSet)
    jobs.push_back(threading::ExpandStateJob::Ptr(new threading::ExpandStateJob(footstep, state, max_risk_)));

  expand_states_manager->addJobs(jobs);
  expand_states_manager->waitUntilJobsFinished();

  for (std::list<threading::ExpandStateJob::Ptr>::iterator itr = jobs.begin(); itr != jobs.end(); itr++)
  {
    threading::ExpandStateJob::Ptr& job = *itr;
    if (job->successful)
      result.push_back(job->next);
  }

  return result;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::ReachabilityStateGenerator, vigir_footstep_planning::StateGeneratorPlugin)
