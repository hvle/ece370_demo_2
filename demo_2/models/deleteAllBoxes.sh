#!/bin/sh

for i in {1..102}
do 
    echo $i
    rosservice call gazebo/delete_model box$i &
done