source devel/setup.bash
export GAZEBO_MODEL_PATH=`pwd`/models:${GAZEBO_MODEL_PATH}
#export GAZEBO_PLUGIN_PATH=`pwd`/devel/lib:/opt/ros/kinetic/lib:${HOME}/hector_gazebo/devel/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=`pwd`/devel/lib:/opt/ros/kinetic/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_RESOURCE_PATH=`pwd`:${GAZEBO_RESOURCE_PATH}
