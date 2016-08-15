/*
 * mechanum_control_odom_main3.cpp
 *
 *  Created on: Aug 16, 2015
 *      Author: siti
 */

/*
 * mechanum_control_odom_main2.cpp
 *
 *  Created on: Aug 15, 2015
 *      Author: siti
 */
/*
 *
 * Copyright (c) 2012, SITI
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

// this software  is used to convert cmd_vel commands to the speeds of the four mechanum wheels of a mobile robot
// based on the content in mechanum kinematic analysis
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "SerialPort.h"
#include <stdio.h>
#include <vector>
#include <stdint.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>

#define SD_SIZE 25
#define VALID_DATA_LENGTH 8
#define BOGUS_COUNTER_THRESHOLD 5

class MechanumControlOdom
{
private:
	double robot_width_;
	double robot_length_;
	std::string arduino_serial;
	SerialPort its_serial;
	double loop_rate_control;
	double loop_rate_read;
	ros::NodeHandle n_;
    ros::Subscriber cmd_vel_sub;

	int16_t cur_wheelUL; //wheel number 1,
	int16_t cur_wheelLL; //wheel number 2;
	int16_t cur_wheelLR; //wheel number 3;
	int16_t cur_wheelUR; //wheel number 4

	int16_t prev_wheelUL; //wheel number 1,
	int16_t prev_wheelLL; //wheel number 2;
	int16_t prev_wheelLR; //wheel number 3;
	int16_t prev_wheelUR; //wheel number 4

	int sleep_time_in_milisecond;


	bool itsDebugMode ;
	    enum StateType
	    {
	      WaitingForDollar  = 0, //!< Waiting for an $ to appear
	      WaitingForSharp = 1, //!<  Waiting for an # to appear
	      ReadingData   = 2, //!< The header has been read, now we're just reading the data
	      WaitingForPercent = 3,
	      WaitingForClaim= 4,
	    };
	StateType itsState;
	uint8_t itsRawMsg[SD_SIZE];
	int itsRawMsg_index ;
	int bogusCounter;

	double		x ;
	double		y ;
	double		th ;
	ros::Time	last_time;
	ros::Time	current_time ;
	double curr_dx, curr_dy, curr_dth;
	double		prev_dx ;
	double		prev_dy ;
	double		prev_dth ;
	double k;
	ros::Publisher odom_method1_pub_;
	ros::Publisher odom_method2_pub_;
	double maximum_speed;

public:
	MechanumControlOdom();
	void cmd_vel_callback(const geometry_msgs::TwistConstPtr &cmd_msg);
	void ros_loop_control_odom();//publish odometry from the speeds of four wheels and sending speeds to arduino
	void send_speed(); //send the speeds to arduino
	void resetMessage();
	void processByte(uint8_t byte);
	void processRawMsgAndPublishOdom(uint8_t *itsRawMsg);
	~MechanumControlOdom()   { }

};

MechanumControlOdom::MechanumControlOdom()
{
	ros::NodeHandle n_local("~");
	n_local.param("robot_width",robot_width_, 0.307);
	n_local.param("robot_length",robot_length_, 0.4);
	n_local.param("arduino_serial",arduino_serial, std::string("/dev/ttyUSB0"));
	n_local.param("loop_rate_control",loop_rate_control, 5.0);
	n_local.param("loop_rate_read",loop_rate_read, 20.0);
	n_local.param("sleep_time_in_milisecond",sleep_time_in_milisecond, 10);
	n_local.param<bool>("itsDebugMode",itsDebugMode, false);
	n_local.param("maximum_speed",maximum_speed, 1.0);

	k = (robot_width_ + robot_length_)*0.5;
	last_time = ros::Time::now();
	current_time = ros::Time::now();
	itsRawMsg_index = 0;
	x = 0.0;
	y = 0.0;
	th = 0.0;

    prev_dx = 0;
	prev_dy = 0;
	prev_dth = 0;

	cur_wheelUL = prev_wheelUL = 0;
	cur_wheelLL = prev_wheelLL = 0;
	cur_wheelLR = prev_wheelLR = 0;
	cur_wheelUR = prev_wheelUR = 0;

	odom_method1_pub_ =  n_.advertise<nav_msgs::Odometry>("odom_method1", 10);
    odom_method2_pub_ =  n_.advertise<nav_msgs::Odometry>("odom_method2", 10);

	cmd_vel_sub = n_.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &MechanumControlOdom::cmd_vel_callback,this);
	its_serial.setPort((char*)arduino_serial.c_str());
	its_serial.setBaudRate(B115200);
	itsState = WaitingForDollar;
	if(!its_serial.openPort())
	  {
	    std::cerr << "Could not connect to serial port!" << std::endl;
	    exit(-1);//false;
	  }
}



void MechanumControlOdom::ros_loop_control_odom()
{
	  //ros::Rate LoopRateRead(loop_rate_read);
	  while(ros::ok())
	  {
		  //LoopRateRead.sleep();
	    std::vector<uint8_t> bytes = its_serial.readVector(12); //read the speed from arduino
	    if(bytes.size() > 0)
	    {
	      for(size_t i = 0; i < bytes.size(); ++i)
	        {processByte(bytes[i]);
	         if(itsDebugMode) printf("%d ", bytes[i]);
	        }
	    }


		if(cur_wheelUL!=prev_wheelUL || cur_wheelLL!=prev_wheelLL ||cur_wheelLR!=prev_wheelLR ||cur_wheelUR!=prev_wheelUR )
			{
			 send_speed(); //send speed only when current ones are different from the previous ones
		     prev_wheelUL = cur_wheelUL;
		     prev_wheelLL = cur_wheelLL;
		     prev_wheelLR = cur_wheelLR;
		     prev_wheelUR = cur_wheelUR;
		     ROS_INFO("speeds are: %d,%d,%d,%d\n",cur_wheelUL,cur_wheelLL,cur_wheelLR,cur_wheelUR);
			}

	  }
}

void MechanumControlOdom::send_speed()
{
	std::vector<char> bytes;
	bytes.clear();
	bytes.push_back('$');
	bytes.push_back('#');
	bytes.push_back(char(cur_wheelUL & 0x00FF));//low 8 bits
	bytes.push_back(char(cur_wheelUL >>8));//high 8 bits
	bytes.push_back(char(cur_wheelLL & 0x00FF));//low 8 bits
	bytes.push_back(char(cur_wheelLL >>8));//high 8 bits
	bytes.push_back(char(cur_wheelLR & 0x00FF));//low 8 bits
	bytes.push_back(char(cur_wheelLR >>8));//high 8 bits
	bytes.push_back(char(cur_wheelUR & 0x00FF));//low 8 bits
	bytes.push_back(char(cur_wheelUR >>8));//high 8 bits
	bytes.push_back('%');
	bytes.push_back('!');
	for(int i=0;i<bytes.size();i++)
	{
		its_serial.writeData(&bytes[i],1);
		//usleep(1000*sleep_time_in_milisecond);
	}
}

void MechanumControlOdom::cmd_vel_callback(const geometry_msgs::TwistConstPtr &cmd_msg)
{

	double vx = cmd_msg->linear.x;
	double vy = cmd_msg->linear.y;
	double omega = cmd_msg->angular.z;


	cur_wheelUL = (int16_t)((vx - vy - k*omega)*1000);
	cur_wheelLL = (int16_t)((vx + vy - k*omega)*1000);
	cur_wheelLR = (int16_t)((vx - vy + k*omega)*1000);
	cur_wheelUR = (int16_t)((vx + vy + k*omega)*1000);

}

void MechanumControlOdom::resetMessage()
{
    itsState = WaitingForDollar;
      for(int i=0;i<SD_SIZE;i++)
		itsRawMsg[i] = 0;
    itsRawMsg_index = 0;
}

void MechanumControlOdom::processByte(uint8_t byte)
{
    if(itsState==WaitingForDollar)
    {
        for(int i=0;i<SD_SIZE;i++)
    		itsRawMsg[i] = 0;
        if(byte == '$')
          {
            itsRawMsg[itsRawMsg_index++] = byte;
            itsState = WaitingForSharp;
            return;
          }
        else
        {if(itsDebugMode) std::cout << "bogus byte: " << std::hex << int(byte) << std::dec << std::endl;
	        bogusCounter++;
	        if((bogusCounter>BOGUS_COUNTER_THRESHOLD))
	        {
		        resetMessage();
		         bogusCounter = 0;
	        }
	    return;
	    }
    }

    if(itsState==WaitingForSharp)
    {
         if(byte == '#')
          {
            itsRawMsg[itsRawMsg_index++] = byte;
            itsState = ReadingData;
            return;
          }
          else
          {
          if(itsDebugMode) std::cout << "bogus byte: " << std::hex << int(byte) << std::dec << std::endl;
	        bogusCounter++;
	        if((bogusCounter>BOGUS_COUNTER_THRESHOLD))
	        {
		        resetMessage();
		         bogusCounter = 0;
	        }
	    return;

          }

    }

    if(itsState==ReadingData)
    {
        itsRawMsg[itsRawMsg_index++] = byte;
        if(int(VALID_DATA_LENGTH - (itsRawMsg_index-2))==0)
        {

            itsState = WaitingForPercent;
            return;
        }

    }

    if(itsState==WaitingForPercent)
    {
         if(byte == '%')
          {
            itsRawMsg[itsRawMsg_index++] = byte;
            itsState = WaitingForClaim;
            return;
          }
          else
          {
          if(itsDebugMode) std::cout << "bogus byte: " << std::hex << int(byte) << std::dec << std::endl;
	        bogusCounter++;
	        if((bogusCounter>BOGUS_COUNTER_THRESHOLD))
	        {
		        resetMessage();
		         bogusCounter = 0;
	        }
	    return;

          }

    }

     if(itsState==WaitingForClaim)
    {
         if(byte == '!')
          {
            itsRawMsg[itsRawMsg_index++] = byte;
            processRawMsgAndPublishOdom(itsRawMsg);
            resetMessage(); //prepare for next reading
            return;
          }
          else
          {
          if(itsDebugMode) std::cout << "bogus byte: " << std::hex << int(byte) << std::dec << std::endl;
	        bogusCounter++;
	        if((bogusCounter>BOGUS_COUNTER_THRESHOLD))
	        {
		        resetMessage();
		         bogusCounter = 0;
	        }
	    return;

          }

    }

}


void MechanumControlOdom::processRawMsgAndPublishOdom(uint8_t *itsRawMsg)
{

    double wheelUL = double (int16_t((uint16_t(itsRawMsg[2])|(uint16_t(itsRawMsg[3])<<8))));
    double wheelLL = double (int16_t((uint16_t(itsRawMsg[4])|(uint16_t(itsRawMsg[5])<<8))));
    double wheelLR = double (int16_t((uint16_t(itsRawMsg[6])|(uint16_t(itsRawMsg[7])<<8))));
    double wheelUR = double (int16_t((uint16_t(itsRawMsg[8])|(uint16_t(itsRawMsg[9])<<8))));
    wheelUL /=1000;
    wheelLL /=1000;
    wheelLR /=1000;
    wheelUR /=1000;
    if(wheelUL>=maximum_speed || wheelLL>=maximum_speed||
    		wheelLR>=maximum_speed||wheelUR>=maximum_speed)
    {
    	ROS_ERROR("The parsed wheel speeds is  greater than the maximum speed");
    	return;
    }
    printf("real wheel speeds are: %f,%f,%f,%f\n",wheelUL, wheelLL, wheelLR, wheelUR);

    curr_dx = 1/4.0*(wheelUL + wheelLL + wheelUR+ wheelLR);
	curr_dy = 1/4.0*(-wheelUL + wheelLL - wheelUR+ wheelLR);
	curr_dth = 1/(4.0*k)*(-wheelUL - wheelLL +  wheelUR+ wheelLR);
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	x = x + prev_dx*dt*cos(th) - prev_dy*dt*sin(th);
	y = y + prev_dx*dt*sin(th) + prev_dy*dt*cos(th);
	th = th + prev_dth*dt;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    nav_msgs::Odometry odom1;
	odom1.header.stamp = current_time;
			odom1.header.frame_id = "odom1";

			//set the position
			odom1.pose.pose.position.x = x;
			odom1.pose.pose.position.y = y;
			odom1.pose.pose.position.z = 0.0;
			odom1.pose.pose.orientation = odom_quat;

			//set the velocity
			odom1.child_frame_id = "base_link";
			odom1.twist.twist.linear.x = curr_dx;
			odom1.twist.twist.linear.y = curr_dy;
			odom1.twist.twist.angular.z = curr_dth;

			//publish the message
			odom_method1_pub_.publish(odom1);

	prev_dx = curr_dx;
	prev_dy = curr_dy;
	prev_dth = curr_dth;
	last_time = current_time;

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mechnum_control_odom");

	MechanumControlOdom MCO;
	boost::thread spin_thread1(boost::bind(&MechanumControlOdom::ros_loop_control_odom,&MCO));

	ros::spin();

	return 0;

}

