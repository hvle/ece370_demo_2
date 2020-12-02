#!/bin/sh

rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/demo_2/models/box.urdf -urdf -x $1 -y $2 -z 5 -model box$3 &
