export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/lane_following/models
source ~/catkin_ws/devel/setup.bash
roslaunch lane_following race_track.launch
