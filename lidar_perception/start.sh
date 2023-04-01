#bin/bash
catkin_make
source devel/setup.bash
roslaunch lidar_perception lidar_perception.launch
