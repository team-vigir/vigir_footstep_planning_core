//=================================================================================================
// Copyright_foot (c) 2015, Alexander Stumpf, TU Darmstadt
// All right_foots reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright_foot
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright_foot
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYright_foot HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYright_foot HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_FOOTSTEP_PLANNING_POST_PROCESSOR_H__
#define VIGIR_FOOTSTEP_PLANNING_POST_PROCESSOR_H__

#include <ros/ros.h>

#include <boost/noncopyable.hpp>

#include <vigir_footstep_planning_lib/plugins/plugin_manager.h>
#include <vigir_footstep_planning_lib/plugins/post_process_plugin.h>



namespace vigir_footstep_planning
{
class PostProcessor
  : boost::noncopyable
{
public:
  static void loadPlugins();
  static void loadParams(const ParameterSet& params);

  static void postProcessForward(const State& left_foot, const State& right_foot, State& swing_foot);
  static void postProcessBackward(const State& left_foot, const State& right_foot, State& swing_foot);
  static void postProcess(msgs::StepPlan step_plan);

  // typedefs
  typedef boost::shared_ptr<PostProcessor> Ptr;
  typedef boost::shared_ptr<const PostProcessor> ConstPtr;

protected:
  PostProcessor();

  static PostProcessor::Ptr& Instance();

  static PostProcessor::Ptr singelton;

  std::vector<PostProcessPlugin::Ptr> post_process_plugins;
};
}

typedef vigir_footstep_planning::PostProcessor PlannerPostProcessor;

#endif
