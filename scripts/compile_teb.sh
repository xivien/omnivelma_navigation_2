#!/bin/bash 

cd ../../

git clone -b foxy-devel https://github.com/xivien/teb_local_planner.git
git clone -b ros2 https://github.com/rst-tu-dortmund/costmap_converter.git

cd ..

source /opt/ros/foxy/setup.bash 
colcon build --symlink-install