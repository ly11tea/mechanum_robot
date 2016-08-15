#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <vector>
#include <queue>
#include <stdint.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

class OdomFromCmdVel{
private:
	ros::NodeHandle n_;
    ros::Subscriber cmd_vel_sub;
	double		x ;
	double		y ;
	double		th ;
	ros::Time	last_time;
	ros::Time	current_time ;
	ros::Publisher odom_pub_;
	double vx ;
	double vy ;
	double omega ;
	double loop_rate;

	geometry_msgs::TransformStamped _odomTf;
	tf::TransformBroadcaster _odomBroadcast;
	std::string _frameBase;
	std::string cmd_vel_sub_name;
public:
	OdomFromCmdVel();
	void cmd_vel_callback(const geometry_msgs::TwistConstPtr &cmd_msg);
	void ros_loop_odom();
	~OdomFromCmdVel()   { }

};

OdomFromCmdVel::OdomFromCmdVel()
{
	ros::NodeHandle n_local("~");
	n_local.param("frameBase",_frameBase, std::string("base_footprint"));
	n_local.param("cmd_vel_sub_name",cmd_vel_sub_name, std::string("cmd_vel"));
	n_local.param("loop_rate",loop_rate, 50.0);
	last_time = ros::Time::now();
	current_time = ros::Time::now();
	odom_pub_ =  n_.advertise<nav_msgs::Odometry>("odom", 10);
	cmd_vel_sub = n_.subscribe<geometry_msgs::Twist>(cmd_vel_sub_name.c_str(), 10, &OdomFromCmdVel::cmd_vel_callback,this);

	x = 0.0;
	y = 0.0;
	th = 0.0;
	vx = 0.0;
	vy = 0.0;
	omega = 0.0;
}

void OdomFromCmdVel::cmd_vel_callback(const geometry_msgs::TwistConstPtr &cmd_msg)
{
	 vx = cmd_msg->linear.x;
	 vy = cmd_msg->linear.y;
	 omega = cmd_msg->angular.z;

}

void OdomFromCmdVel::ros_loop_odom()
{
	ros::Rate lr(loop_rate);
	 while(ros::ok())
	 {
		 current_time = ros::Time::now();
		 double dt = (current_time-last_time).toSec();
		 x = x +vx*cos(th)*dt - vy*sin(th)*dt;
		 y = y +vx*sin(th)*dt + vy*cos(th)*dt;
		 th = th + omega*dt;
		 last_time = current_time;

		   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		    nav_msgs::Odometry odom;
		    odom.header.stamp = current_time;
		    odom.header.frame_id = "odom";

		    //set the position
		    odom.pose.pose.position.x = x;
		    odom.pose.pose.position.y = y;
		    odom.pose.pose.position.z = 0.0;
		    odom.pose.pose.orientation = odom_quat;

		    //set the velocity
		    odom.child_frame_id = _frameBase;
		    odom.twist.twist.linear.x = vx;
		    odom.twist.twist.linear.y = vy;
		    odom.twist.twist.angular.z = omega;

		    //publish the message
		    odom_pub_.publish(odom);

		    _odomTf.header.stamp = current_time;
		    _odomTf.header.frame_id = "/odom";
		    _odomTf.child_frame_id = _frameBase;

		    _odomTf.transform.translation.x = x;
		    _odomTf.transform.translation.y = y;
		    _odomTf.transform.translation.z = 0.0;
		    _odomTf.transform.rotation = odom_quat;

		    //send the transform
		    _odomBroadcast.sendTransform(_odomTf);


		    lr.sleep();
	 }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_from_cmd_vel");

	OdomFromCmdVel OFCV;
	boost::thread spin_thread1(boost::bind(&OdomFromCmdVel::ros_loop_odom,&OFCV));

	ros::spin();

	return 0;

}

