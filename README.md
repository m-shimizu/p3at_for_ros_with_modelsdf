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
    $ source setup.bash  
    
## How to use this program.  
    $ cd p3at_for_ros_with_modelsdf
    $ source setup.bash  
    $ roslaunch .....  
    
    
