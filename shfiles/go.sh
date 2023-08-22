#!/bin/bash

mate-terminal -- bash -c "roslaunch robot_navigation robot_navigation.launch; exec bash"

sleep 1s;

mate-terminal -- bash -c "roslaunch robot_navigation navigation_rviz.launch; exec bash"

sleep 1s;

mate-terminal -- bash -c "roslaunch robot_navigation way_point.launch; exec bash"

