#!/bin/sh

rosservice call gazebo/delete_model dd_robot
rosrun gazebo_ros spawn_model -file model.sdf -sdf -x $1 -y $2 -z 1 -model dd_robot
