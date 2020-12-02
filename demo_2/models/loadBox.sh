#!/bin/sh

rosservice call gazebo/delete_model box
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/demo_2/models/box.urdf -urdf -x 0 -y 0 -z 5 -model box
