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
#include <vigir_footstep_planner/world_model/upper_body_grid_map_model.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
UpperBodyGridMapModel::UpperBodyGridMapModel(const std::string& name, const vigir_generic_params::ParameterSet& params)
  : GridMapModel(name, params)
{
}

UpperBodyGridMapModel::UpperBodyGridMapModel(const std::string& name)
  : GridMapModel(name)
{
}

UpperBodyGridMapModel::UpperBodyGridMapModel()
  : GridMapModel("upper_body_grid_map_model")
{
}

bool UpperBodyGridMapModel::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapModel::initialize(nh, params))
    return false;

  // get upper body dimensions
  getUpperBodySize(nh, upper_body_size);
  getUpperBodyOriginShift(nh, upper_body_origin_shift);

  return true;
}

bool UpperBodyGridMapModel::isAccessible(const State& /*s*/) const
{
  // We can't make any checks with a single foot pose
  return true;
}

bool UpperBodyGridMapModel::isAccessible(const State& next, const State& current) const
{
  boost::shared_lock<boost::shared_mutex> lock(grid_map_shared_mutex);

  if (!occupancy_grid_map)
  {
    ROS_ERROR_THROTTLE(10, "[UpperBodyGridMapModel] No body level grid map available yet.");
    return true;
  }

  if (next.getLeg() == current.getLeg())
  {
    ROS_ERROR_THROTTLE(10, "[UpperBodyGridMapModel] Swing foot can't equal support foot.");
    return false;
  }

  const State& left = next.getLeg() == LEFT ? next : current;
  const State& right = next.getLeg() == RIGHT ? next : current;

  // approximate upper body dimensions
  float x = right.getX() + 0.5 * (left.getX() - right.getX());
  float y = right.getY() + 0.5 * (left.getY() - right.getY());

  float theta = right.getYaw() + 0.5 * (left.getYaw() - right.getYaw());

  // determine shift of polygon based on foot orientation
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);
  float shift_x = cos_theta * upper_body_origin_shift.x - sin_theta * upper_body_origin_shift.y;
  float shift_y = sin_theta * upper_body_origin_shift.x + cos_theta * upper_body_origin_shift.y;

  // shift pose
  x += shift_x;
  y += shift_y;

  return !collision_check(x, y, cos_theta, sin_theta, upper_body_size.x, upper_body_size.y);
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::UpperBodyGridMapModel, vigir_footstep_planning::CollisionCheckPlugin)
