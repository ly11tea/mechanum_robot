#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <math.h>
#include <iostream>
#include <vector>
#include "SerialPort.h"
#include <stdint.h>

#define SD_SIZE 25
#define VALID_DATA_LENGTH 8
#define BOGUS_COUNTER_THRESHOLD 5

std::string arduino_serial("/dev/ttyUSB0");
SerialPort its_serial; 
double loop_rate = 20.0;
bool itsDebugMode = false;
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
int itsRawMsg_index = 0;
int bogusCounter;

double		x = 0.0;
double		y = 0.0;
double		th = 0.0;
ros::Time	last_time;
ros::Time	current_time ;
double curr_dx, curr_dy, curr_dth;
double		prev_dx = 0;
double		prev_dy = 0;
double		prev_dth = 0;
double k;
double robot_width_, robot_length_;
ros::Publisher odom_method1_pub_;
ros::Publisher odom_method2_pub_;

void resetMessage();
void processByte(uint8_t byte);
void processRawMsgAndPublishOdom(uint8_t *itsRawMsg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mechanum_control_odom");
    ros::NodeHandle n_;
    ros::NodeHandle n_local("~");
    //parameters here
    n_local.param<std::string>("arduino_serial",arduino_serial, "/dev/ttyUSB0");
    n_local.param<double>("loop_rate",loop_rate, 20.0);
    n_local.param<bool>("itsDebugMode",itsDebugMode, false);
	n_local.param("robot_width",robot_width_, 0.307);
	n_local.param("robot_length",robot_length_, 0.4);
	k = (robot_width_ + robot_length_)*0.5;
   last_time = ros::Time::now();
   current_time = ros::Time::now();
    
    odom_method1_pub_ =  n_.advertise<nav_msgs::Odometry>("odom_method1", 10);
    odom_method2_pub_ =  n_.advertise<nav_msgs::Odometry>("odom_method2", 10);
    
   its_serial.setPort((char*)arduino_serial.c_str());
   its_serial.setBaudRate(B115200);
   itsState = WaitingForDollar;
  if(!its_serial.openPort())
  {
    std::cerr << "Could not connect to serial port!" << std::endl;
    return -1;//false;
  }
  //ros::Rate LoopRate(loop_rate);
  while(ros::ok())
  {
    ros::spinOnce();
    //LoopRate.sleep();
    std::vector<uint8_t> bytes = its_serial.readVector(12);
    if(bytes.size() > 0)
    {
      for(size_t i = 0; i < bytes.size(); ++i)
        {processByte(bytes[i]); 
         if(itsDebugMode) printf("%d ", bytes[i]);
        }
    }
  }
  
}

void resetMessage()
{
    itsState = WaitingForDollar;
      for(int i=0;i<SD_SIZE;i++)
		itsRawMsg[i] = 0;
    itsRawMsg_index = 0;
}

void processByte(uint8_t byte)
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


void processRawMsgAndPublishOdom(uint8_t *itsRawMsg)
{

    double wheelUL = double (uint16_t(itsRawMsg[2])|(uint16_t(itsRawMsg[3])<<8));
    double wheelLL = double (uint16_t(itsRawMsg[4])|(uint16_t(itsRawMsg[5])<<8));
    double wheelLR = double (uint16_t(itsRawMsg[6])|(uint16_t(itsRawMsg[7])<<8));
    double wheelUR = double (uint16_t(itsRawMsg[8])|(uint16_t(itsRawMsg[9])<<8));
    wheelUL /=1000;
    wheelLL /=1000;
    wheelLR /=1000;
    wheelUR /=1000;
    printf("wheel speeds are: %f,%f,%f,%f\n",wheelUL, wheelLL, wheelLR, wheelUR);
    curr_dx = 1/4.0*(wheelUL + wheelLL + wheelUR+ wheelLR);
	curr_dy = 1/4.0*(-wheelUL + wheelLL - wheelUR+ wheelLR);
	curr_dth = 1/(4.0*k)*(-wheelUL - wheelLL +  wheelUR+ wheelLR);

	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	printf("dt is %f\n",dt);
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
