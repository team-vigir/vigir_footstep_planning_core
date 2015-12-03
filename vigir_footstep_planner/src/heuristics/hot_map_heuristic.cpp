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
#include <vigir_footstep_planner/heuristics/hot_map_heuristic.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
HotMapHeuristic::HotMapHeuristic()
: HeuristicPlugin("hot_map_heuristic")
{
}

void HotMapHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  HeuristicPlugin::loadParams(params);

  params.getParam("collision_check/cell_size", cell_size);
  int num_angle_bins;
  params.getParam("collision_check/num_angle_bins", num_angle_bins);
  angle_bin_size = 2.0*M_PI / static_cast<double>(num_angle_bins);
}

bool HotMapHeuristic::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::initialize(nh, params))
    return false;

  // publish topics
  hot_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("hot_map", 1);
  publish_timer = nh.createTimer(ros::Duration(1.0), &HotMapHeuristic::publishHotMap, this);

  return true;
}

void HotMapHeuristic::reset()
{
  boost::unique_lock<boost::shared_mutex> lock(hot_map_shared_mutex);
  hot_map.clear();
  total_hit_counter = 0;
}

double HotMapHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  unsigned int hit = 0;
  {
    boost::unique_lock<boost::shared_mutex> lock(hot_map_shared_mutex);
    hit = hot_map[StateKey(from, cell_size, angle_bin_size)]++;
    total_hit_counter++;
  }

  return 1000.0 * static_cast<double>(hit)/static_cast<double>(total_hit_counter);
}

void HotMapHeuristic::publishHotMap(const ros::TimerEvent& /*publish_timer*/) const
{
  if (hot_map_pub.getNumSubscribers() > 0)
  {
    boost::shared_lock<boost::shared_mutex> lock(hot_map_shared_mutex);

    nav_msgs::OccupancyGrid grid_map;

    grid_map.header.stamp = ros::Time::now();
    grid_map.header.frame_id = "/world";

    /// TODO: Implement grid map conversion

    hot_map_pub.publish(grid_map);
  }
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::HotMapHeuristic, vigir_footstep_planning::HeuristicPlugin)
