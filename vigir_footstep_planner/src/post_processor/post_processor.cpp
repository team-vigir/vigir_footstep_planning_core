/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
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
  vigir_pluginlib::PluginManager::getPluginsByType(Instance()->post_process_plugins);

  ROS_INFO("[PostProcessor] Plugins loaded:");
  if (Instance()->post_process_plugins.empty())
    ROS_INFO("    No plugins found!");

  for (std::vector<PostProcessPlugin::Ptr>::const_iterator itr = Instance()->post_process_plugins.begin(); itr != Instance()->post_process_plugins.end(); itr++)
  {
    const PostProcessPlugin::Ptr& post_processor = *itr;
    if (post_processor)
      ROS_INFO("    %s (%s)", post_processor->getName().c_str(), post_processor->getTypeClass().c_str());
  }
}

void PostProcessor::loadParams(const vigir_generic_params::ParameterSet& params)
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
