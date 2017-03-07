source devel/setup.bash
export GAZEBO_MODEL_PATH=`pwd`/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=`pwd`/devel/lib:/opt/ros/indigo/lib:${GAZEBO_PLUGIN_PATH}
