#!/bin/sh

rosrun gazebo_ros spawn_model -file box.urdf -urdf -x $1 -y $2 -z 5 -model box$3 &
