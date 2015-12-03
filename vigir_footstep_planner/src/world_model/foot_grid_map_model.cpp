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
#include <vigir_footstep_planner/world_model/foot_grid_map_model.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
FootGridMapModel::FootGridMapModel(const std::string& name, const vigir_generic_params::ParameterSet& params)
  : GridMapModel(name, params)
{
}

FootGridMapModel::FootGridMapModel(const std::string& name)
  : GridMapModel(name)
{
}

FootGridMapModel::FootGridMapModel()
  : GridMapModel("foot_grid_map_model")
{
}

bool FootGridMapModel::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapModel::initialize(nh, params))
    return false;

  // get foot dimensions
  getFootSize(nh, foot_size);

  return true;
}

bool FootGridMapModel::isAccessible(const State& s) const
{
  boost::shared_lock<boost::shared_mutex> lock(grid_map_shared_mutex);

  if (!occupancy_grid_map)
  {
    ROS_ERROR_THROTTLE(10, "[FootGridMapModel] No ground level grid map available yet.");
    return true;
  }

  double x = s.getX();
  double y = s.getY();

  double theta = s.getYaw();

  // collision check for the foot center
  return !collision_check(x, y, cos(theta), sin(theta), foot_size.x, foot_size.y);
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::FootGridMapModel, vigir_footstep_planning::CollisionCheckPlugin)
