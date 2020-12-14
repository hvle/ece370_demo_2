#!/bin/sh

rosservice call gazebo/delete_model box
rosrun gazebo_ros spawn_model -file box.urdf -urdf -x 0 -y 0 -z 5 -model box
