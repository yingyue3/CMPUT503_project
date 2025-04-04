#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun llm_control motion.py

# wait for app to end
dt-launchfile-join
