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
#include <vigir_footstep_planner/heuristics/dynamics_heuristic.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
DynamicsHeuristic::DynamicsHeuristic()
  : HeuristicPlugin("dynamics_heuristic")
{
}

void DynamicsHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  HeuristicPlugin::loadParams(params);

  params.getParam("dynamics/body/max_acc", max_body_acc, 0.0);
}

double DynamicsHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  if (max_body_acc <= 0.0)
    return 0.0;

  // check if we can still retard in time
  const geometry_msgs::Vector3& v = from.getBodyVelocity();
  double d_min = (v.x*v.x + v.y*v.y + v.z*v.z)/(2.0*max_body_acc);

  // if planner has issues finding a solution, increase the scaling of d_min or decrease cell size
  if (d_min*1.2 > euclidean_distance(from.getX(), from.getY(), from.getZ(), to.getX(), to.getY(), to.getZ()))
    return max_heuristic_value;

  return 0.0;
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::DynamicsHeuristic, vigir_footstep_planning::HeuristicPlugin)
