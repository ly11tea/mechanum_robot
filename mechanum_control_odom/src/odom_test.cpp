/*
 * odom_test.cpp
 *
 *  Created on: Aug 17, 2015
 *      Author: siti
 */

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <math.h>

class OdomTest
{
	private:
		bool straight_or_side; //test straight or side walk;
		ros::NodeHandle n_;
	    ros::Subscriber odom_sub;
	    ros::Publisher cmd_pub;
	    double loop_rate;
	    double constant_speed;
	    double stop_at_displacement;
	    nav_msgs::Odometry odometry;

	public:
		OdomTest();
		void OdomCallBack(const nav_msgs::OdometryConstPtr &odom_msg);
		void ros_loop();
		~OdomTest(){}

};

OdomTest::OdomTest()
{
	ros::NodeHandle n_local("~");
	n_local.param("straight_or_side",straight_or_side, true);//true means straight
	n_local.param("loop_rate",loop_rate, 20.0);//true means straight
	n_local.param("constant_speed",constant_speed, 0.1);//true means straight
	n_local.param("stop_at_displacement",stop_at_displacement, 1.0);//true means straight

    odom_sub = n_.subscribe<nav_msgs::Odometry>("odom", 5, &OdomTest::OdomCallBack,this);
    cmd_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel",5);
    odometry.pose.pose.position.x = 0;
    odometry.pose.pose.position.y = 0;
}

void OdomTest::OdomCallBack(const nav_msgs::OdometryConstPtr &odom_msg)
{
	odometry = *odom_msg;
}
void OdomTest::ros_loop()
{
	ros::Rate LoopRate(loop_rate);
	  geometry_msgs::Twist cmd;
	while(ros::ok())
	{
		LoopRate.sleep();
		double x = odometry.pose.pose.position.x;
		double y = odometry.pose.pose.position.y;
		double displacement = sqrt(x*x + y*y);
		if(straight_or_side)
		{
			if(displacement>=stop_at_displacement)
			  {
				cmd.linear.x = 0;
			  }
			else
				cmd.linear.x = constant_speed;
			cmd.linear.y = 0;
		}
		else
		{
			if(displacement>=stop_at_displacement)
			  {
				cmd.linear.y = 0;
			  }
			else
				cmd.linear.y = constant_speed;
			cmd.linear.x = 0;
		}
		cmd_pub.publish(cmd);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_test");

	OdomTest OT;
	boost::thread spin_thread1(boost::bind(&OdomTest::ros_loop,&OT));

	ros::spin();

	return 0;

}


