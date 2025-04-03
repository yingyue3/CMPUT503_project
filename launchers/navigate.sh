#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun safety_detection navigate_template.py

# wait for app to end
dt-launchfile-join
