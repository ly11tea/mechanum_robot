/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/**
\author Tully Foote
@b ScanFootprintFilter takes input scans and corrects for footprint angle assuming a flat target.  
This is useful for ground plane extraction
**/



#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"


using namespace std;
class LaserScanFootprintFilter
{
private:
	  tf::TransformListener tf_;
	  laser_geometry::LaserProjection projector_;
	  double inscribed_radius_;
	  bool up_and_running_;
	  ros::NodeHandle n_;
	  ros::Publisher scan_pub_;
	  ros::Subscriber scan_sub_;
	  string scan_sub_topic;
	  string scan_pub_topic;
	  string base_link_name;

public:
  LaserScanFootprintFilter()
  {
	  ros::NodeHandle n_local("~");
	  n_local.param("inscribed_radius",inscribed_radius_, 0.2);
	  n_local.param("scan_sub_topic",scan_sub_topic, std::string("scan_sub"));
	  n_local.param("scan_pub_topic",scan_pub_topic, std::string("scan_pub"));
	  n_local.param("base_link_name",base_link_name, std::string("base_link"));
	  scan_pub_ = n_.advertise<sensor_msgs::LaserScan>(scan_pub_topic.c_str(), 10);
	  scan_sub_ = n_.subscribe<sensor_msgs::LaserScan>(scan_sub_topic.c_str(), 10, &LaserScanFootprintFilter::scanCallBack,this);

   // ROS_WARN("LaserScanFootprintFilter has been deprecated.  Please use PR2LaserScanFootprintFilter instead.\n");
  }


  virtual ~LaserScanFootprintFilter()
  { 

  }

  void scanCallBack(const sensor_msgs::LaserScanConstPtr &scan_msg)
  {
	sensor_msgs::LaserScan filtered_scan = *scan_msg ;
    sensor_msgs::PointCloud laser_cloud;

    try{
      projector_.transformLaserScanToPointCloud(base_link_name.c_str(), *scan_msg, laser_cloud, tf_);
    }
    catch(tf::TransformException& ex){

        ROS_WARN_THROTTLE(1, "Dropping Scan: Transform unavailable %s", ex.what());
        return ;
    }

    int c_idx = indexChannel(laser_cloud);

    if (c_idx == -1 || laser_cloud.channels[c_idx].values.size () == 0){
      ROS_ERROR("We need an index channel to be able to filter out the footprint");
      return ;
    }
    
    for (unsigned int i=0; i < laser_cloud.points.size(); i++)  
    {
      if (inFootprint(laser_cloud.points[i])){
        int index = laser_cloud.channels[c_idx].values[i];
        filtered_scan.ranges[index] = filtered_scan.range_max + 1.0 ; // If so, then make it a value bigger than the max range
      }
    }
    scan_pub_.publish(filtered_scan);
    return;
  }

  int indexChannel(const sensor_msgs::PointCloud& scan_cloud){
      int c_idx = -1;
      for (unsigned int d = 0; d < scan_cloud.channels.size (); d++)
      {
        if (scan_cloud.channels[d].name == "index")
        {
          c_idx = d;
          break;
        }
      }
      return c_idx;
  }

  bool inFootprint(const geometry_msgs::Point32& scan_pt){
    if(scan_pt.x < -1.0 * inscribed_radius_ || scan_pt.x > inscribed_radius_ || scan_pt.y < -1.0 * inscribed_radius_ || scan_pt.y > inscribed_radius_)
      return false;
    return true;
  }

} ;

int main(int argc, char** argv)
{
	  ros::init(argc, argv, "robot_laser_footprint_filter");
	  LaserScanFootprintFilter LSFF;
	  ros::spin();

	  return(0);
}



