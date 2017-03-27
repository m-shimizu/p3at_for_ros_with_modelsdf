# p3at_for_ros_with_modelsdf
IF YOU DO NOT NEED TO VIEW YOUR ROBOT MODEL ON RVIZ, YOU CAN USE SDF FILES INSTEAD OF URDF FILES TO DESCRIBE YOUR ROBOT FOR GAZEBO.  
YOU CAN CONTROL YOUR ROBOT IN GAZEBO FROM ROS WITH MODEL.SDF.
YOU CAN GET 
A sample model.sdf of pioneer3at for using it in combination with ROS.
Yes, the model.sdf is including gazebo_ros libraries.

## REQUIREMENT OF THIS REPOSITORY
### Install Ubuntu 16.04 LTS
Please done installation of Ubuntu 16.04 LTS (64bit).

### Install ROS Kinetic and Gazebo7 from PPA
#### *[Ubuntu install of ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
#### *[Install Gazebo using Ubuntu packages](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install)  
Do followings:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
    sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116  
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'  
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -  
    sudo apt-get update  
    sudo apt-get install -y cmake g++ protobuf-compiler libgazebo7 libgazebo7-dev ros-kinetic-desktop ros-kinetic-gazebo7-ros-pkgs ros-kinetic-gazebo7-ros-control ros-kinetic-ros-controllers ros-kinetic-image-view2 ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-hector-mapping ros-kinetic-hector-geotiff ros-kinetic-joy ros-kinetic-teleop-twist-keyboard  
    sudo rosdep init  
    rosdep update  
    sudo apt-get install -y python−rosinstall  
    gazebo (and wait for finish of downloading fundamental models)  

## How to prepare to use this program  
1. Install an(some) additional requiered software(s).  

    $ sudo apt-get install ros-kinetic-hector-gazebo-thermal-camera  

2. Build this package.  

    $ cd  
    $ git clone https://github.com/m-shimizu/p3at_for_ros_with_modelsdf  
    $ cd p3at_for_ros_with_modelsdf/src  
    $ catkin_init_workspace  
    $ cd ..  
    $ catkin_make  
    
## How to use this program  
You need 2 terminals for spawning a robot and controlling the robot.

    Terminal 1(To spawn a robot):  

    $ cd p3at_for_ros_with_modelsdf  
    $ source setup.bash  
    $ roslaunch gazebo_ros_sdf empty.launch  
    (Then spawn a robot "pioneer3at_ros" from "INSERT TAB")
    
    Terminal 2(To control the robot):  

    $ rostopic list
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/pioneer3at_ros/cmd_vel

## How to get an experience with a thermal camera  
You need 2 terminals for spawning a robot and controlling the robot.

    Terminal 1(To spawn a robot):  

    $ cd p3at_for_ros_with_modelsdf  
    $ source setup.bash  
    $ roslaunch gazebo_ros_sdf empty.launch world:=where_is_hot.world  
    (pioneer2dx_ros will be spawned automatically)
    
    Terminal 2(Control the robot and watch the red boxes through the thermal camera):  

    $ rostopic list
    $ rosrun image_view2 image_view2 image:=/pioneer2dx_ros/thermal_camera/image_raw &    
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/pioneer2dx_ros/cmd_vel

UPDATED : 15/3/2017
