#include <vigir_footstep_planning_default_plugins/state_generator/reachability_state_generator.h>



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

  return true;
}

std::list<PlanningState::Ptr> ReachabilityStateGenerator::generatePredecessor(const PlanningState& state) const
{
  std::list<PlanningState::Ptr> result;
  return result;
}

std::list<PlanningState::Ptr> ReachabilityStateGenerator::generateSuccessor(const PlanningState& state) const
{
  std::list<PlanningState::Ptr> result;
  return result;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::ReachabilityStateGenerator, vigir_footstep_planning::StateGeneratorPlugin)
