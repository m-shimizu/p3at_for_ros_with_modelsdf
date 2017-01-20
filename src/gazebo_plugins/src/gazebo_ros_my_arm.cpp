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
    SHOULDER,
    ELBOW,
    WRIST,
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
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "my_arm_robot" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

/*
    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_arm" );

    gazebo_ros_->getParameter<std::string> ( arm_shoulder_frame_, "ArmShoulderFrame", "ArmShoulderFrame" );
    gazebo_ros_->getParameter<std::string> ( arm_elbow_frame_,    "ArmElbowFrame",    "ArmElbowFrame" );
    gazebo_ros_->getParameter<std::string> ( arm_wrist_frame_,    "ArmWristFrame",    "ArmWristFrame" );
    gazebo_ros_->getParameter<std::string> ( arm_finger_frame_,   "ArmFingerFrame",   "ArmFingerFrame" );
*/

printf("AAAAAAA\n");
    joints_.resize ( JOINT_SIZE );
    joints_[SHOULDER] = gazebo_ros_->getJoint ( parent, "shoulderJoint", "shoulder_joint" );
if(0==joints_[SHOULDER]) printf("Error getting joint: shoulderJoint\n");
    joints_[ELBOW]    = gazebo_ros_->getJoint ( parent, "elbowJoint", "elbow_joint" );
if(0==joints_[ELBOW]) printf("Error getting joint: elbowJoint\n");
    joints_[WRIST]    = gazebo_ros_->getJoint ( parent, "wristJoint", "wrist_joint" );
if(0==joints_[WRIST]) printf("Error getting joint: wristJoint\n");
    joints_[FINGER1]  = gazebo_ros_->getJoint ( parent, "finger1Joint", "finger1_joint" );
if(0==joints_[FINGER1]) printf("Error getting joint: finger1Joint\n");
    joints_[FINGER2]  = gazebo_ros_->getJoint ( parent, "finger2Joint", "finger2_joint" );
if(0==joints_[FINGER2]) printf("Error getting joint: finger2Joint\n");
    joints_[FINGER12] = gazebo_ros_->getJoint ( parent, "finger12Joint", "finger12_joint" );
if(0==joints_[FINGER12]) printf("Error getting joint: finger12Joint\n");
    joints_[FINGER22] = gazebo_ros_->getJoint ( parent, "finger22Joint", "finger22_joint" );
if(0==joints_[FINGER22]) printf("Error getting joint: finger22Joint\n");

    last_update_time_ = parent->GetWorld()->GetSimTime();

    alive_ = true;

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

/*
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                boost::bind(&GazeboRosMyArm::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO("%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());
*/

printf("BBBBBBBB\n");

    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &GazeboRosMyArm::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosMyArm::UpdateChild, this ) );

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
    orders_[i] = -10 * (Monitor_Angles_[i] - Target_Angles_[i]);

    this->parent->GetJointController()->SetVelocityPID(
        this->joints_[i]->GetScopedName(), common::PID(0.1, 0, 0));

    this->joints_[i]->SetVelocity(0, orders_[i]);
  }
}

#include <termios.h>
#include <fcntl.h>

// To know pushing any key
int	doslike_kbhit(void)
{
	struct termios	oldt, newt;
	int	ch;
	int	oldf;
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
int	doslike_getch(void)
{
	static struct termios	oldt, newt;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	int c = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return c;
}

void	GazeboRosMyArm::check_key_command(void)
{
	if(doslike_kbhit())
	{
		int cmd = doslike_getch();
		switch(cmd)
		{
			case 'q': Target_Angles_[SHOULDER] += 0.05;
				  break;
			case 'a': Target_Angles_[SHOULDER] -= 0.05;
				  break;
			case 'w': Target_Angles_[ELBOW] += 0.05;
				  break;
			case 's': Target_Angles_[ELBOW] -= 0.05;
				  break;
			case 'e': Target_Angles_[WRIST] += 0.05;
				  break;
			case 'd': Target_Angles_[WRIST] -= 0.05;
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

/*
void GazeboRosMyArm::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
}
*/

void GazeboRosMyArm::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosMyArm )
}

