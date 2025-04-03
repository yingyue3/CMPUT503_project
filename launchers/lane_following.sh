#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun safety_detection lane_following_controller_template.py

# wait for app to end
dt-launchfile-join
