#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: rosbag_filter orig.bag target.bag"
fi

if [ ! -f "$1" ]; then
    echo "Error: File $1 does not exist!";
    exit 1
fi

rosbag filter $1 $2 \
"topic == '/flor/controller/step_command' or \
topic == '/flor/controller/step_feedback' or \
topic == '/robot_controllers/footstep_controller/execute_step_plan/feedback' or \
topic == '/robot_controllers/footstep_controller/execute_step_plan/goal' or \
topic == '/robot_controllers/footstep_controller/execute_step_plan/result' or \
topic == '/robot_controllers/footstep_controller/execute_step_plan/status' or \
topic == '/vigir/ocs/footstep_planning/step_plan_request/execute_step_plan/feedback' or \
topic == '/vigir/ocs/footstep_planning/step_plan_request/execute_step_plan/goal' or \
topic == '/vigir/ocs/footstep_planning/step_plan_request/execute_step_plan/result' or \
topic == '/vigir/ocs/footstep_planning/step_plan_request/execute_step_plan/status'"
