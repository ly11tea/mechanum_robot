#include "ros/ros.h"
#include "std_srvs/Empty.h"

#include "stdio.h"

using namespace std;


double calling_frequency;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "call_clear_costmap_srv");

  ros::NodeHandle n;
  ros::NodeHandle n_local("~");
  n_local.param("calling_frequency",calling_frequency, 0.5);


  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  std_srvs::Empty srv;
  ros::Rate rate(calling_frequency);
  while(ros::ok())
  {
	  rate.sleep();
	  if (client.call(srv))
	  {
		  ROS_INFO("calling /move_base/clear_costmaps is successful");
	  }
	  else
	  {
	    ROS_ERROR("Failed to call /move_base/clear_costmaps");
	    //return 1;
	  }
  }


   return 0;
}
