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
#include <vigir_footstep_planner/heuristics/occupancy_grid_map_heuristic.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
OccupancyGridMapHeuristic::OccupancyGridMapHeuristic()
: HeuristicPlugin("occupancy_grid_map_heuristic")
{
}

void OccupancyGridMapHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  HeuristicPlugin::loadParams(params);

  params.getParam("occupancy_grid_map_heuristic/grid_map_topic", grid_map_topic);
}

bool OccupancyGridMapHeuristic::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::initialize(nh, params))
    return false;

  // subscribe topics
  occupancy_grid_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(grid_map_topic, 1, &OccupancyGridMapHeuristic::mapCallback, this);

  return true;
}

void OccupancyGridMapHeuristic::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map)
{
  boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex);
  distance_map.setMap(occupancy_grid_map);
}

double OccupancyGridMapHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  boost::shared_lock<boost::shared_mutex> lock(grid_map_shared_mutex);

  double d = distance_map.distanceMapAt(from.getX(), from.getY());
  if (d < 0.0)
    return 0.0;

  if (d < 0.01)
    return max_heuristic_value;

  return 5.0*std::max(0.0, 0.5-d);
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::OccupancyGridMapHeuristic, vigir_footstep_planning::HeuristicPlugin)
