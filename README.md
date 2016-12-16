# p3at_for_ros_with_modelsdf
IF YOU DO NOT NEED TO VIEW YOUR ROBOT MODEL ON RVIZ, YOU CAN USE SDF FILES INSTEAD OF URDF FILES TO DESCRIBE YOUR ROBOT FOR GAZEBO.  
YOU CAN CONTROL YOUR ROBOT IN GAZEBO FROM ROS WITH MODEL.SDF.
YOU CAN GET 
A sample model.sdf of pioneer3at for using it in combination with ROS.
Yes, the model.sdf is including gazebo_ros libraries.

## How to prepare to use this program.  
    $ cd  
    $ git clone https://github.com/m-shimizu/p3at_for_ros_with_modelsdf  
    $ cd p3at_for_ros_with_modelsdf/src  
    $ catkin_init_workspace  
    $ cd ..  
    $ catkin_make  
    
## How to use this program.  
You need 2 terminals for spawning a robot and controlling the robot.

    Terminal 1(To spawn a robot):  

    $ cd p3at_for_ros_with_modelsdf  
    $ source setup.bash  
    $ roslaunch gazebo_ros_sdf empty.launch  
    (Then spawn a robot "pioneer3at_ros" from "INSERT TAB")
    
    Terminal 2(To control the robot):  

    $ rostopic list
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/pioneer3at_ros/cmd_vel

UPDATED : 16/12/2016
