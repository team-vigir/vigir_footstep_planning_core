#include <vigir_footstep_planner/post_processor/post_processor.h>

namespace vigir_footstep_planning
{
PostProcessor::Ptr PostProcessor::singelton = PostProcessor::Ptr();

PostProcessor::PostProcessor()
{
}

PostProcessor::Ptr& PostProcessor::Instance()
{
  if (!singelton)
    singelton.reset(new PostProcessor());
  return singelton;
}

void PostProcessor::loadPlugins()
{
  // get collision check plugins
  PluginManager::getPluginsByType(Instance()->post_process_plugins);

  ROS_INFO("[PostProcessor] Plugins loaded:");
  if (Instance()->post_process_plugins.empty())
    ROS_INFO("    No plugins found!");

  for (std::vector<PostProcessPlugin::Ptr>::const_iterator itr = Instance()->post_process_plugins.begin(); itr != Instance()->post_process_plugins.end(); itr++)
  {
    const PostProcessPlugin::Ptr& post_processor = *itr;
    if (post_processor)
      ROS_INFO("    %s (%s)", post_processor->getName().c_str(), post_processor->getTypeId().c_str());
  }
}

void PostProcessor::loadParams(const ParameterSet& params)
{
  for (std::vector<PostProcessPlugin::Ptr>::iterator itr = Instance()->post_process_plugins.begin(); itr != Instance()->post_process_plugins.end(); itr++)
  {
    PostProcessPlugin::Ptr& post_processor = *itr;
    if (post_processor)
      post_processor->loadParams(params);
  }
}

void PostProcessor::postProcessForward(const State& left_foot, const State& right_foot, State& swing_foot)
{
  for (std::vector<PostProcessPlugin::Ptr>::iterator itr = Instance()->post_process_plugins.begin(); itr != Instance()->post_process_plugins.end(); itr++)
  {
    PostProcessPlugin::Ptr& post_processor = *itr;
    if (post_processor)
      post_processor->postProcessStepForward(left_foot, right_foot, swing_foot);
  }
}

void PostProcessor::postProcessBackward(const State& left_foot, const State& right_foot, State& swing_foot)
{
  for (std::vector<PostProcessPlugin::Ptr>::iterator itr = Instance()->post_process_plugins.begin(); itr != Instance()->post_process_plugins.end(); itr++)
  {
    PostProcessPlugin::Ptr& post_processor = *itr;
    if (post_processor)
      post_processor->postProcessStepBackward(left_foot, right_foot, swing_foot);
  }
}

void PostProcessor::postProcess(msgs::StepPlan step_plan)
{
  for (std::vector<PostProcessPlugin::Ptr>::iterator itr = Instance()->post_process_plugins.begin(); itr != Instance()->post_process_plugins.end(); itr++)
  {
    PostProcessPlugin::Ptr& post_processor = *itr;
    if (post_processor)
      post_processor->postProcess(step_plan);
  }
}
}
