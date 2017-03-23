/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */


/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */

#include <algorithm>
#include <assert.h>

enum {
    SHOULDERYAW,
    SHOULDERPITCH,
    FINGER1,
    FINGER2,
    FINGER12,
    FINGER22,
    JOINT_SIZE
};

#include "gazebo_ros_my_arm.h"

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo
{


GazeboRosMyArm::GazeboRosMyArm() 
{
  for(int i = 0; i < JOINT_SIZE; i++)
  {
    Target_Angles_[i] = 0;
  }
}

// Destructor
GazeboRosMyArm::~GazeboRosMyArm() {}

// Load the controller
void GazeboRosMyArm::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

  this->parent = _parent;
  gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "MyArm" ) );
  // Make sure the ROS node for Gazebo has already been initialized
  gazebo_ros_->isInitialized();

  gazebo_ros_->getParameter<std::string> ( command_topic1_, "commandTopic1", "cmd_arm12" );
  gazebo_ros_->getParameter<std::string> ( command_topic2_, "commandTopic2", "cmd_hand12" );


/*
  gazebo_ros_->getParameter<std::string> ( arm_shoulder_frame_, "ArmShoulderFrame", "ArmShoulderFrame" );
  gazebo_ros_->getParameter<std::string> ( arm_elbow_frame_,  "ArmElbowFrame",  "ArmElbowFrame" );
  gazebo_ros_->getParameter<std::string> ( arm_wrist_frame_,  "ArmWristFrame",  "ArmWristFrame" );
  gazebo_ros_->getParameter<std::string> ( arm_finger_frame_,   "ArmFingerFrame",   "ArmFingerFrame" );
*/

  joints_.resize ( JOINT_SIZE );
  joints_[SHOULDERYAW]  = gazebo_ros_->getJoint ( parent, "shoulderYawJoint", "shoulderYaw_joint" );
  if(0==joints_[SHOULDERYAW])
    printf("Error getting joint: shoulderYawJoint\n");
  joints_[SHOULDERPITCH]  = gazebo_ros_->getJoint ( parent, "shoulderPitchJoint", "shoulderPitch_joint" );
  if(0==joints_[SHOULDERPITCH])
    printf("Error getting joint: shoulderPitchJoint\n");
  joints_[FINGER1]  = gazebo_ros_->getJoint ( parent, "finger1Joint", "finger1_joint" );
  if(0==joints_[FINGER1])
    printf("Error getting joint: finger1Joint\n");
  joints_[FINGER2]  = gazebo_ros_->getJoint ( parent, "finger2Joint", "finger2_joint" );
  if(0==joints_[FINGER2])
    printf("Error getting joint: finger2Joint\n");
  joints_[FINGER12] = gazebo_ros_->getJoint ( parent, "finger12Joint", "finger12_joint" );
  if(0==joints_[FINGER12])
    printf("Error getting joint: finger12Joint\n");
  joints_[FINGER22] = gazebo_ros_->getJoint ( parent, "finger22Joint", "finger22_joint" );
  if(0==joints_[FINGER22])
    printf("Error getting joint: finger22Joint\n");

  last_update_time_ = parent->GetWorld()->GetSimTime();

  alive_ = true;

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic1_.c_str());
  ros::SubscribeOptions so1 =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic1_, 1,
        boost::bind(&GazeboRosMyArm::cmdarm12_Callback, this, _1),
        ros::VoidPtr(), &queue_);
  cmd_arm12_subscriber_ = gazebo_ros_->node()->subscribe(so1); // DO NOT REMOVE "cmd_arm12_subscriber_ = "
  ROS_INFO("%s: Subscribe to %s!", gazebo_ros_->info(), command_topic1_.c_str());

  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic2_.c_str());
  ros::SubscribeOptions so2 =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic2_, 1,
        boost::bind(&GazeboRosMyArm::cmdhand12_Callback, this, _1),
        ros::VoidPtr(), &queue_);
  cmd_hand12_subscriber_ = gazebo_ros_->node()->subscribe(so2); // DO NOT REMOVE "cmd_hand12_subscriber_ = "
  ROS_INFO("%s: Subscribe to %s!", gazebo_ros_->info(), command_topic2_.c_str());

  // start custom queue for diff drive
  this->callback_queue_thread_ =
    boost::thread ( boost::bind ( &GazeboRosMyArm::QueueThread, this ) );

  // listen to the update event (broadcast every simulation iteration)
  this->update_connection_ =
    event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosMyArm::UpdateChild, this ) );
  printf("\n#####################################################\n");
  printf("#####################################################\n");
  printf("Type the following command in another terminal.\n");
  printf("rostopic pub -r 10 /my_arm_robot/cmd_arm12 geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'\n");
  printf("#####################################################\n");
  printf("#####################################################\n");

}

void GazeboRosMyArm::Reset()
{
  last_update_time_ = parent->GetWorld()->GetSimTime();
}

void GazeboRosMyArm::PID_Control(void)
{
  double Monitor_Angles_[JOINT_SIZE] , orders_[JOINT_SIZE];
  for(int i = 0 ; i < JOINT_SIZE; i++)
  {
    Monitor_Angles_[i] = this->joints_[i]->GetAngle(0).Radian();
  // printf("Monitor Angle[%d] : %f\n", i, this->joints_[i]->GetAngle(0).Degree());

  // Only Proportional Control in velocity
    orders_[i] = -1 * (Monitor_Angles_[i] - Target_Angles_[i]);

    this->parent->GetJointController()->SetVelocityPID(
        this->joints_[i]->GetScopedName(), common::PID(0.1, 0, 0));

    this->joints_[i]->SetVelocity(0, orders_[i]);
  }
}

#include <termios.h>
#include <fcntl.h>

// To know pushing any key
int  doslike_kbhit(void)
{
  struct termios  oldt, newt;
  int  ch;
  int  oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

/////////////////////////////////////////////////
// To gwt a charactor code of a pushed key
int  doslike_getch(void)
{
  static struct termios  oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

void  GazeboRosMyArm::check_key_command(void)
{
  if(doslike_kbhit())
  {
  int cmd = doslike_getch();
    switch(cmd)
    {
      case 'q': Target_Angles_[FINGER1] += 0.05;
          break;
      case 'a': Target_Angles_[FINGER1] -= 0.05;
          break;
      case 'w': Target_Angles_[SHOULDERYAW] += 0.05;
          break;
      case 's': Target_Angles_[SHOULDERYAW] -= 0.05;
          break;
      case 'e': Target_Angles_[SHOULDERPITCH] += 0.05;
          break;
      case 'd': Target_Angles_[SHOULDERPITCH] -= 0.05;
          break;
    }
  }
}

// Update the controller
void GazeboRosMyArm::UpdateChild()
{
  check_key_command();
  PID_Control();
}

// Finalize the controller
void GazeboRosMyArm::FiniChild()
{
  alive_ = false;
  queue_.clear();
  queue_.disable();
  gazebo_ros_->node()->shutdown();
  callback_queue_thread_.join();
}

void GazeboRosMyArm::cmdarm12_Callback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
  Target_Angles_[SHOULDERYAW]   = cmd_msg->linear.x;
  Target_Angles_[SHOULDERPITCH] = cmd_msg->linear.y;
  Target_Angles_[FINGER1]       = cmd_msg->linear.z;
  Target_Angles_[FINGER2]       = cmd_msg->angular.x;
  Target_Angles_[FINGER12]      = cmd_msg->angular.y;
  Target_Angles_[FINGER22]      = cmd_msg->angular.z;
/*
  printf("cmdarm12 : x,y,z,r,p,y=%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n"
            , cmd_msg->linear.x, cmd_msg->linear.y, cmd_msg->linear.z,
              cmd_msg->angular.x, cmd_msg->angular.y, cmd_msg->angular.z);
*/
}

void GazeboRosMyArm::cmdhand12_Callback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
  printf("cmdhand12 : x,y,z,r,p,y=%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n"
            , cmd_msg->linear.x, cmd_msg->linear.y, cmd_msg->linear.z,
              cmd_msg->angular.x, cmd_msg->angular.y, cmd_msg->angular.z);
}

void GazeboRosMyArm::QueueThread()
{
  static const double timeout = 0.01;
  while ( alive_ && gazebo_ros_->node()->ok() )
  {
    queue_.callAvailable ( ros::WallDuration ( timeout ) );
  }
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosMyArm )
}

