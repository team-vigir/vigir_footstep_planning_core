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
#include <vigir_walk_monitor/walk_analyzer.h>

namespace vigir_footstep_planning
{
WalkAnalyzer::WalkAnalyzer(ros::NodeHandle& nh)
{
  nh.getParam("logging_dir", logging_dir);
  //data_header = "index step_x step_y step_z droll dpitch dyaw distance_2D distance_3D duration swing_height";
  data_header = "index | step_x step_y step_yaw | dx dy dyaw | swing_distance duration";
  reset();
}

WalkAnalyzer::~WalkAnalyzer()
{
}

void WalkAnalyzer::reset()
{
  current_goal_id = actionlib_msgs::GoalID();
  current_step_plan = msgs::StepPlan();
  step_feedback_map.clear();
}

void WalkAnalyzer::executeStepPlanGoalCallback(const msgs::ExecuteStepPlanActionGoal& execute_goal)
{
  // check if new step plan is executed
  if (execute_goal.goal_id.id != current_goal_id.id || execute_goal.goal_id.stamp != current_goal_id.stamp)
  {
    if (!step_feedback_map.empty())
      writeResult(logging_dir, data_header);
    reset();
  }
  else
    return;

  if (!execute_goal.goal.step_plan.steps.empty())
  {
    current_goal_id = execute_goal.goal_id;
    current_step_plan = execute_goal.goal.step_plan;

    const msgs::Step& initial = current_step_plan.steps.front();

    msgs::Step start_left, start_right;
    start_left.foot = current_step_plan.start.left;
    start_right.foot = current_step_plan.start.right;

    // adding initial states
    step_feedback_map[initial.step_index-1] = initial.foot.foot_index == msgs::Foot::LEFT ? start_right : start_left;
    step_feedback_map[initial.step_index] = initial.foot.foot_index == msgs::Foot::LEFT ? start_left : start_right;

    ROS_INFO("[WalkAnalyzer] Got new step plan with %lu steps.", current_step_plan.steps.size());
  }
}

void WalkAnalyzer::executeStepPlanResultCallback(const msgs::ExecuteStepPlanActionResult& execute_result)
{
  if (!step_feedback_map.empty())
    writeResult(logging_dir, data_header);
  reset();
}

void WalkAnalyzer::stepFeedbackCallback(const msgs::StepPlanFeedback& feedback)
{
  if (current_step_plan.steps.empty() || feedback.steps.empty())
    return;

  for (std::vector<msgs::Step>::const_iterator itr = feedback.steps.begin(); itr != feedback.steps.end(); itr++)
  {
    if (itr->step_index <= current_step_plan.steps.back().step_index)
      step_feedback_map[itr->step_index] = *itr;
  }
}

bool WalkAnalyzer::analyze(const msgs::Step& previous, const msgs::Step& current, const msgs::Step& next, std::string& result) const
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6);
  std::ostringstream temp;
  temp << std::fixed << std::setprecision(6);

  // get travel distance
  geometry_msgs::Pose dswing;
  getDeltaStep(previous.foot, next.foot, dswing);

  tf::Quaternion swing_q;
  tf::quaternionMsgToTF(dswing.orientation, swing_q);

  double swing_r, swing_p, swing_y;
  tf::Matrix3x3(swing_q).getRPY(swing_r, swing_p, swing_y);

  // reconstruct delta step
  geometry_msgs::Pose dstep;
  getDeltaStep(current.foot, next.foot, dstep);

  tf::Quaternion step_q;
  tf::quaternionMsgToTF(dstep.orientation, step_q);

  double step_r, step_p, step_y;
  tf::Matrix3x3(step_q).getRPY(step_r, step_p, step_y);

  // generate message
  ss << next.step_index << " | "
     << dstep.position.x << " " << dstep.position.y << " "
     /*<< r << " " << p << " "*/ << step_y << " | "
     << dswing.position.x << " " << dswing.position.y << " " //<< dstep.position.z << " "
     /*<< r << " " << p << " "*/ << swing_y << " | "
     << sqrt(dswing.position.x*dswing.position.x + dswing.position.y*dswing.position.y) << " "
     //<< sqrt(dstep.position.x*dstep.position.x + dstep.position.y*dstep.position.y + dstep.position.z*dstep.position.z) << " "
     << next.step_duration;// << " " <<  next.swing_height;

  if (!temp.str().empty())
    temp << ", ";

  temp << "{" << sqrt(dswing.position.x*dswing.position.x + dswing.position.y*dswing.position.y) << ", " << next.step_duration << "}";

  //ss << "\n" << temp.str();

  result = ss.str();

  return true;
}

bool WalkAnalyzer::analyzeData(const std::map<int, msgs::Step>& data, std::string& result) const
{
  result.clear();

  if (data.empty())
    return false;

  if (data.size() < 3)
    return true;

  std::map<int, msgs::Step>::const_iterator itr = data.begin();
  msgs::Step previous = itr->second; itr++;
  msgs::Step current = itr->second; itr++;

  for (; itr != data.end(); itr++)
  {
    const msgs::Step& next = itr->second;

    if (current.step_index+1 != next.step_index)
    {
      ROS_ERROR("[WalkAnalyzer] analyzeData: Inconsistent data record! Stopping analysis.");
      return false;
    }

    std::string temp;
    if (!analyze(previous, current, next, temp))
      return false;

    if (!result.empty())
      result += "\n";
    result += temp;

    previous = current;
    current = next;
  }

  return true;
}

void WalkAnalyzer::writeResult(const std::string& logging_dir, const std::string& data_header)
{
  std::string result;
  if (!analyzeData(step_feedback_map, result))
  {
    ROS_ERROR("[WalkAnalyzer] Error while analyzing data!");
    return;
  }
  if (result.empty())
    return;

  std::string date_time = currentDateTime();

  // write to seperate file
  std::string filename = logging_dir + "/" + date_time + ".txt";
  std::ofstream file(filename);

  if (file.is_open())
  {
    file << data_header + "\n";
    file << result;
    file.close();
    ROS_INFO("[WalkAnalyzer] Written step plan result in '%s'", filename.c_str());
  }
  else
    ROS_ERROR("[WalkAnalyzer] Can't open file '%s'", filename.c_str());

  // append to total file
  filename = logging_dir + "/overall.txt";
  file.open(filename, std::ios_base::app);

  if (file.is_open())
  {
    file << result << "\n";
    file.close();
    ROS_INFO("[WalkAnalyzer] Written step plan result in '%s'", filename.c_str());
  }
  else
    ROS_ERROR("[WalkAnalyzer] Can't open file '%s'", filename.c_str());
}

std::string WalkAnalyzer::currentDateTime() const
{
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

  return buf;
}
}
