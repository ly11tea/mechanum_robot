/*
 * cmd_vel_publisher.cpp
 *
 *  Created on: Apr 17, 2014
 *      Author: sen
 */


/*
 * 
 * Copyright (c) 2012, University of Essex
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Ling Chen

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <math.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_T 't'
#define KEYCODE_Y 'y'
#define KEYCODE_U 'u'
#define KEYCODE_I 'i'
#define KEYCODE_G 'g'     
#define KEYCODE_H 'h'
#define KEYCODE_J 'j'

#define CONSTANT_V 0.25   //constant velocity for moving forward
#define CONSTANT_OMEGA 0.5 //constant gyro rate for turning
#define CONSTANT_AMPLITUDE 0.5 //the amplitude of the robot waiving
using namespace std;

#define PI 3.14159256

class RobotKeyControl
{
  private:
  double constantV_X;  // The constant linear velocity
  double constantV_Y;  // The constant linear velocity
  double Max_constant_V_X; // the maxmum constant velocity
  double Min_constant_V_X; // the maxmum constant velocity
  double Max_constant_V_Y; // the maxmum constant velocity
  double Min_constant_V_Y; // the maxmum constant velocity
  double delta_v; // the velocity increament

  double Herz; //the frequency of publishing cmd
  double frequency ; // the frequency of tail waiving
  double Amplitude ; // the amplitude of the tail waiving
  double Max_Amplitude; // the maximum amplitude of the robot waiving

  double counter ;  //counter
  double constantOmega; //the direct current part of angular rate
  double Max_constant_Omega; // the maximum omega
  double delta_omega ; // the increasing step of the omega.
  double Speed_Noise_Variance;
//  double heading ;  // heading of the vehicle
//  double headingToBeUed;
  bool dirty ;

  geometry_msgs::Twist cmd;

  ros::NodeHandle n_;
  ros::Publisher vel_pub_;
//  ros::Subscriber state_sub;

  public:
  void stateCallback(const nav_msgs::OdometryConstPtr &statemsg);
  void init()
  {
	    ros::NodeHandle n_local("~");
	    n_local.param("Max_constant_V_X",Max_constant_V_X, 0.5);
	    n_local.param("Min_constant_V_X",Min_constant_V_X, 0.05);
	    n_local.param("delta_v",delta_v, 0.05);
	    n_local.param("frequency",frequency, 1.0);
	    n_local.param("Max_Amplitude",Max_Amplitude, CONSTANT_AMPLITUDE);
	    n_local.param("Herz",Herz, 50.0);
	    n_local.param("Max_constant_V_Y",Max_constant_V_Y, 0.5);
	    n_local.param("Min_constant_V_Y",Min_constant_V_Y, 0.05);
	   // n_local.param("delta_omega",delta_omega, 0.05);
	    n_local.param("Speed_Noise_Variance",Speed_Noise_Variance, 0.01);
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    constantV_X = Min_constant_V_X;
    constantV_Y = Min_constant_V_Y;
   // Herz = 50;
   // frequency = 1.0;
    Amplitude = 0;
    counter  = 0;
    constantOmega = 0;
//    heading = 0;
//    headingToBeUed = heading;
    dirty = false;
    vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
 //   state_sub = n_.subscribe<nav_msgs::Odometry>("/ground_truth/state", 1, &RobotKeyControl::stateCallback,this);

    ros::NodeHandle n_private("~");




  }

  ~RobotKeyControl()   { }
  void keyboardLoop();
  void rosLoop();


};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}


void RobotKeyControl::keyboardLoop()
{
  char c;
   dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);


  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WASD' to control the robot");



   while (1)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }




    switch(c)
    {
      // Walking
    case KEYCODE_W:
  
      constantV_X = constantV_X + delta_v;
      if(constantV_X>=Max_constant_V_X)
    	  constantV_X  = Max_constant_V_X;
      Amplitude = Max_Amplitude;
      constantOmega =0;
      cmd.linear.x = constantV_X;
      cmd.linear.y = 0;
     // headingToBeUed = heading;
  	 ROS_INFO("constantV_X is %f", constantV_X);
       dirty = true;
      break;
    case KEYCODE_S:
    	constantV_X = constantV_X - delta_v;

        if(constantV_X<= -Max_constant_V_X)
      	  constantV_X  = -Max_constant_V_X;
    	Amplitude = Max_Amplitude;
    	constantOmega = 0;
    	cmd.linear.x = constantV_X;
        cmd.linear.y = 0;
      dirty = true;
   	 ROS_INFO("constantV_X is %f", constantV_X);
      break;
    case KEYCODE_A:
      constantV_Y = constantV_Y + delta_v;
      if(constantV_Y>=Max_constant_V_Y)
    	  constantV_Y  = Max_constant_V_Y;
      Amplitude = Max_Amplitude;
      constantOmega =0;
      cmd.linear.x = 0;
      cmd.linear.y = constantV_Y;
     // headingToBeUed = heading;
  	 ROS_INFO("constantV_Y is %f", constantV_Y);
      dirty = true;
      break;
    case KEYCODE_D:
    	constantV_Y = constantV_Y - delta_v;
        if(constantV_Y<= -Max_constant_V_Y)
      	  constantV_Y  = -Max_constant_V_Y;
    	Amplitude = Max_Amplitude;
    	constantOmega = 0;
    	cmd.linear.x = 0;
        cmd.linear.y = constantV_Y;
       ROS_INFO("constantV_Y is %f", constantV_Y);
      dirty = true;
      break;
      case KEYCODE_T:
      case KEYCODE_Y:
      case KEYCODE_U:
      case KEYCODE_I:
      case KEYCODE_G:     
      case KEYCODE_H:
      case KEYCODE_J:
         constantV_Y = 0;
          constantV_X = 0;
         cmd.linear.x = 0;
         cmd.linear.y = 0;
       break;
      }
      
      vel_pub_.publish(cmd);




  }
}

void RobotKeyControl::rosLoop()
{
	   cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

	   ros::Rate loop_rate(Herz);

	   boost::mt19937 rng; // I don't seed it on purpouse (it's not relevant)

	    boost::normal_distribution<> nd(0.0, Speed_Noise_Variance);

	    boost::variate_generator<boost::mt19937&,
	                             boost::normal_distribution<> > var_nor(rng, nd);


		  while(ros::ok())
		  {
             counter = counter +1 ;
		     double t = counter*1/Herz;
			 cmd.angular.z = Amplitude * sin(2 * PI * frequency * t) + constantOmega;/*+*/
			if (dirty == true)
				{
				 
					 
				}
			ros::spinOnce();
			loop_rate.sleep();
			//ROS_INFO("cmd.linear.x y angular z is %f, %f,  %f", cmd.linear.x, cmd.linear.y, cmd.angular.z);
		  }


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_key_control");

  RobotKeyControl tpk;
  tpk.init();

  signal(SIGINT,quit);

  boost::thread spin_thread(boost::bind(&RobotKeyControl::rosLoop,&tpk));

  tpk.keyboardLoop();

  return(0);
}

