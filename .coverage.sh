#!/bin/bash

source "${ICI_SRC_PATH}/workspace.sh"
source "${ICI_SRC_PATH}/util.sh"

echo "Generating coverage for eigen_state_space_systems"

ws=~/target_ws
extend="/opt/ros/$ROS_DISTRO"
ici_exec_in_workspace "$extend" "$ws" catkin build eigen_state_space_systems -v --no-deps --catkin-make-args coverage_report

echo "Uploading coverage results to codecov.io"

# Remove duplicated information
rm "$ws/build/eigen_state_space_systems/coverage_report.info.cleaned"
rm "$ws/build/eigen_state_space_systems/coverage_report.info.removed"

# Actually upload coverage information
bash <(curl -s https://codecov.io/bash) -s "$ws/build/eigen_state_space_systems/"
