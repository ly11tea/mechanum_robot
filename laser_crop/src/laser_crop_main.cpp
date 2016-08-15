/*
 * laser_crop_main.cpp
 *
 *  Created on: Aug 30, 2015
 *      Author: siti
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace std;
class LaserCrop
{
private:
	double angle_min_cropped;
	double angle_max_cropped;
	ros::Publisher scan_pub_;
	ros::Subscriber scan_sub_;
	  string scan_sub_topic;
	  string scan_pub_topic;
	  ros::NodeHandle n_;

public:
	LaserCrop();
	~LaserCrop() {}
	void scanCallBack(const sensor_msgs::LaserScanConstPtr &scan_msg);
};

LaserCrop::LaserCrop()
{
	ros::NodeHandle n_local("~");
    n_local.param("angle_min_cropped",angle_min_cropped, DEG2RAD(90.0f));
    n_local.param("angle_max_cropped",angle_max_cropped, DEG2RAD(-90.0f));
    n_local.param("scan_sub_topic",scan_sub_topic, std::string("scan_rplidar"));
    n_local.param("scan_pub_topic",scan_pub_topic, std::string("scan"));

    if(angle_min_cropped != -angle_max_cropped)
    	ROS_ERROR("angle_min_cropped is not -angle_max_cropped");

    scan_pub_ = n_.advertise<sensor_msgs::LaserScan>(scan_pub_topic.c_str(), 10);
    scan_sub_ = n_.subscribe<sensor_msgs::LaserScan>(scan_sub_topic.c_str(), 10, &LaserCrop::scanCallBack,this);

}

void LaserCrop::scanCallBack(const sensor_msgs::LaserScanConstPtr &scan_msg)
{
	if(abs(angle_min_cropped)>= abs(scan_msg->angle_min) || abs(angle_max_cropped)>= abs(scan_msg->angle_max) )
	{
		ROS_ERROR("can't be cropped as the angle range of cropped scan is larger than the original one");
		return;
	}
	else
	{
		sensor_msgs::LaserScan scan_croped;
		scan_croped.header.stamp = scan_msg->header.stamp;
		scan_croped.header.frame_id = scan_msg->header.frame_id;
		scan_croped.angle_max = angle_max_cropped;
		scan_croped.angle_min = angle_min_cropped;
		scan_croped.angle_increment = scan_msg->angle_increment;
		scan_croped.scan_time = scan_msg->scan_time;

		scan_croped.time_increment = scan_msg->time_increment ;
		scan_croped.range_min = scan_msg->range_min;
		scan_croped.range_max = scan_msg->range_max;



		int start_index = floor((angle_min_cropped - scan_msg->angle_min)/ scan_msg->angle_increment);
		int N = floor(abs((angle_max_cropped-angle_min_cropped)/scan_msg->angle_increment)) + 1;

		scan_croped.intensities.resize(N);
		scan_croped.ranges.resize(N);

		for (size_t i = 0; i < N; i++)
		{
			scan_croped.ranges[i] = scan_msg->ranges[i+start_index];
			scan_croped.intensities[i] = scan_msg->intensities[i+start_index];
		}
		scan_pub_.publish(scan_croped);
	}
}

int main(int argc, char** argv)
{
	  ros::init(argc, argv, "robot_laser_crop");
	  LaserCrop LC;
	  ros::spin();

	  return(0);

}
