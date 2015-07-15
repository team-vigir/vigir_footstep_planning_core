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
#include <vigir_footstep_planner/robot_model/dynamics_reachability.h>

namespace vigir_footstep_planning
{
DynamicsReachability::DynamicsReachability(const ParameterSet& params)
  : ReachabilityPlugin("dynamics_reachability", params)
{
}

DynamicsReachability::DynamicsReachability()
  : ReachabilityPlugin("dynamics_reachability")
{
}

void DynamicsReachability::loadParams(const ParameterSet& params)
{
  params.getParam("dynamics/body/max_vel", max_body_vel_sq, 0.0);
  max_body_vel_sq *= max_body_vel_sq;
  params.getParam("dynamics/body/max_acc", max_body_acc_sq, 0.0);
  max_body_acc_sq *= max_body_acc_sq;
}

bool DynamicsReachability::isReachable(const State& current, const State& next) const
{
  if (max_body_vel_sq > 0.0)
  {
    const geometry_msgs::Vector3& v = next.getBodyVelocity();
    if ((v.x*v.x + v.y*v.y + v.z*v.z) > max_body_vel_sq)
      return false;
  }

  if (max_body_acc_sq > 0.0 && next.getStepDuration() > 0.0)
  {
    const geometry_msgs::Vector3& v0 = current.getBodyVelocity();
    const geometry_msgs::Vector3& v1 = next.getBodyVelocity();

    geometry_msgs::Vector3 acc;
    acc.x = (v1.x-v0.x)/next.getStepDuration();
    acc.y = (v1.y-v0.y)/next.getStepDuration();
    acc.z = (v1.z-v0.z)/next.getStepDuration();

    if ((acc.x*acc.x + acc.y*acc.y + acc.z*acc.z) > max_body_acc_sq)
      return false;
  }

  return true;
}
}
