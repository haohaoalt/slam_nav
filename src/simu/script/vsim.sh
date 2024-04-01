source ~/course_ws/akm-robot/devel/setup.bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/course_ws/akm-robot/src/simu
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/course_ws/akm-robot/src/simu/models
sleep 1
roslaunch akm_simu vslam_world.launch
