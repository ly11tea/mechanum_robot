/* cmd_vel_publisher.cpp
 *
 *  Created on: Aug 15, 2015
 *      Author: Ling
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

class CmdVelConverter
{
private:
	double robot_width_;
	double robot_length_;
	std::string arduino_serial;
	SerialPort its_serial;
	double loop_rate_control;
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

public:
	CmdVelConverter();
	void cmd_vel_callback(const geometry_msgs::TwistConstPtr &cmd_msg);
	void ros_loop_control();
	void send_speed();
	~CmdVelConverter()   { }

};

CmdVelConverter::CmdVelConverter()
{
	ros::NodeHandle n_local("~");
	n_local.param("robot_width",robot_width_, 0.307);
	n_local.param("robot_length",robot_length_, 0.4);
	n_local.param("arduino_serial",arduino_serial, std::string("/dev/ttyUSB0"));
	n_local.param("loop_rate_control",loop_rate_control, 10.0);
	n_local.param("sleep_time_in_milisecond",sleep_time_in_milisecond, 0);
	cmd_vel_sub = n_.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &CmdVelConverter::cmd_vel_callback,this);
	its_serial.setPort((char*)arduino_serial.c_str());
	its_serial.setBaudRate(B115200);
	if(!its_serial.openPort())
	  {
	    std::cerr << "Could not connect to serial port!" << std::endl;
	    exit(-1);//false;
	  }
	cur_wheelUL = prev_wheelUL = 0;
	cur_wheelLL = prev_wheelLL = 0;
	cur_wheelLR = prev_wheelLR = 0;
	cur_wheelUR = prev_wheelUR = 0;
}

void CmdVelConverter::ros_loop_control()
{	ros::Rate LoopRateControl(loop_rate_control);
	while(ros::ok())
	{
		LoopRateControl.sleep();
		 std::vector<uint8_t> bytes = its_serial.readVector(20);
		 if(bytes.size() > 0)
		 {
			 for(int i=0;i<bytes.size();i++)
				 printf("%d,",bytes[i]);
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

void CmdVelConverter::send_speed()
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
	printf("data sent are:");
	for(int i=0;i<bytes.size();i++)
	{
		its_serial.writeData(&bytes[i],1);
		printf("%d,",bytes[i]);
		usleep(1000*sleep_time_in_milisecond);
	}
	printf("\n");
}

void CmdVelConverter::cmd_vel_callback(const geometry_msgs::TwistConstPtr &cmd_msg)
{
	double k = (robot_width_ + robot_length_)*0.5;
	double vx = cmd_msg->linear.x;
	double vy = cmd_msg->linear.y;
	double omega = cmd_msg->angular.z;

	cur_wheelUL = (int16_t)((vx - vy - k*omega)*1000);
	cur_wheelLL = (int16_t)((vx + vy - k*omega)*1000);
	cur_wheelLR = (int16_t)((vx - vy + k*omega)*1000);
	cur_wheelUR = (int16_t)((vx + vy + k*omega)*1000);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cmd_vel_to_four_wheel_speed");

	CmdVelConverter CVC;
	boost::thread spin_thread(boost::bind(&CmdVelConverter::ros_loop_control,&CVC));

	ros::spin();

	return 0;

}
