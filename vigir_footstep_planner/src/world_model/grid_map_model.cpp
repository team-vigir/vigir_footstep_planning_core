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
#include <vigir_footstep_planner/world_model/grid_map_model.h>

namespace vigir_footstep_planning
{
GridMapModel::GridMapModel(const std::string& name, const ParameterSet& params, unsigned int collision_check_flag, ros::NodeHandle& nh, const std::string& topic)
  : CollisionCheckGridMapPlugin(name, params, collision_check_flag, nh, topic)
{
}

GridMapModel::GridMapModel(const std::string& name, unsigned int collision_check_flag, ros::NodeHandle& nh, const std::string& topic)
  : CollisionCheckGridMapPlugin(name, collision_check_flag, nh, topic)
{
}

void GridMapModel::loadParams(const ParameterSet& params)
{
  CollisionCheckGridMapPlugin::loadParams(params);
  params.getParam("collision_check/collision_check_accuracy", (int&)collision_check_accuracy);
}

void GridMapModel::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map)
{
  CollisionCheckGridMapPlugin::mapCallback(occupancy_grid_map);

  boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex);
  distance_map.setMap(occupancy_grid_map);
}

bool GridMapModel::collision_check(double x, double y, double cos_theta, double sin_theta, double height, double width) const
{
  double d = distance_map.distanceMapAt(x, y);

  bool out_of_bounds = d < 0.0; // if out of bounds => assume no collision

  if (out_of_bounds && collision_check_accuracy != 2)
    return false;

  d = std::max(d-distance_map.getResolution(), 0.0);

  double r_o = width*width + height*height;

  if (!out_of_bounds)
  {
    if (d*d >= 0.25 * r_o)
      return false;
    else if (collision_check_accuracy == 0)
      return false;
  }

  double h_half = 0.5 * height;
  double w_half = 0.5 * width;
  double r_i = std::min(w_half, h_half);

  if (!out_of_bounds)
  {
    if (d <= r_i)
      return true;
    else if (collision_check_accuracy == 1)
      return true;
  }
  else if (r_i < distance_map.getResolution())
  {
    return false;
  }

  double h_new;
  double w_new;
  double delta_x;
  double delta_y;
  if (width < height)
  {
    double h_clear = out_of_bounds ? 0.0 : sqrt(d*d - w_half*w_half);
    h_new = h_half - h_clear;
    w_new = width;
    delta_x = h_clear + 0.5 * h_new;
    delta_y = 0.0;
  }
  else // footWidth >= footHeight
  {
    double w_clear = out_of_bounds ? 0.0 : sqrt(d*d - h_half*h_half);
    h_new = height;
    w_new = w_half - w_clear;
    delta_x = 0.0;
    delta_y = w_clear + 0.5 * w_new;
  }
  double x_shift = cos_theta*delta_x - sin_theta*delta_y;
  double y_shift = sin_theta*delta_x + cos_theta*delta_y;

  return (collision_check(x+x_shift, y+y_shift, cos_theta, sin_theta, h_new, w_new) ||
          collision_check(x-x_shift, y-y_shift, cos_theta, sin_theta, h_new, w_new));
}
}
