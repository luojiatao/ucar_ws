#!/bin/bash

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch gazebo_pkg race.launch; \
exec bash"

# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore

sleep 5s  

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch ucar_nav navi_fast.launch; \
exec bash"



echo “successfully started!”