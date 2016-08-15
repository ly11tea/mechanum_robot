#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include "stdio.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "math.h"

#define PI 3.141592
using namespace std;

struct waypoint
{
	double x;
	double y;
	double theta;
};

typedef std::vector<struct waypoint> Waypoints;

class my_planner
{
private:
	tf::TransformListener& tf_;
	std::string global_frame_;
	std::string robot_base_frame_;
	double transform_tolerance_;
	double control_frequency_;
	Waypoints pts;
	geometry_msgs::Twist cmd;
	ros::Publisher vel_pub_;
	std::string cmd_pub_topic;
	ros::NodeHandle n_;

public:
	my_planner(tf::TransformListener &tf);
	~my_planner()   { }
	std::vector<struct waypoint> makeWayPointsFromParams(ros::NodeHandle& nh);
    double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);
    std::vector<struct waypoint> makeWaypointsFromXMLRPC(XmlRpc::XmlRpcValue& waypoints_xmlrpc,
                                const std::string& full_param_name);
    bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;
    void control_loop();
    bool Planner_2(double c_x, double c_y, double c_th, Waypoints pts, double &vx, double &vy, double &w, int &nextWpt);

};




my_planner::my_planner(tf::TransformListener &tf):tf_(tf)
{
	ros::NodeHandle n_local("~");
	n_local.param("global_frame",global_frame_, std::string("map"));
	n_local.param("robot_base_frame",robot_base_frame_, std::string("base_link"));
	n_local.param("transform_tolerance",transform_tolerance_, 0.8);
	n_local.param("control_frequency",control_frequency_, 10.0);
    n_local.param("cmd_pub_topic",cmd_pub_topic, std::string("cmd_vel"));

    vel_pub_ = n_.advertise<geometry_msgs::Twist>(cmd_pub_topic.c_str(), 10);

    pts = makeWayPointsFromParams(n_local);
    for(int i=0;i<pts.size();i++)
	  {
		  printf("%d th point is: %f, %f, %f\n",i,pts[i].x, pts[i].y, pts[i].theta );
	  }

}


bool my_planner::Planner_2(double c_x, double c_y, double c_th, Waypoints pts, double &vx, double &vy, double &w, int &nextWpt)
{
	double dir_slope = 0;
	double dir_diff = 0;
	double dir_rad = 0; 

	if(nextWpt < pts.size())
	{
		double dist_CurrPt2Waypt = sqrt((pts[nextWpt].y-c_y)*(pts[nextWpt].y-c_y)+(pts[nextWpt].x-c_x)*(pts[nextWpt].x-c_x));

		if(dist_CurrPt2Waypt > 0.2)			//within 0.2m
		{
			if(pts[nextWpt].x-c_x != 0)
			{
				dir_rad = atan2(pts[nextWpt].y-c_y, (pts[nextWpt].x-c_x));
			}
			else
			{
				if( pts[nextWpt].y-c_y > 0) {dir_rad = PI/2;}
				else {dir_rad = -PI/2;}			
			}
				
			while(dir_rad > PI)
			{
				dir_rad -= 2*PI;
			}
			while(dir_rad < -PI)				
			{
				dir_rad += 2*PI;
			}

			dir_diff = dir_rad - c_th;
			if( abs(abs(dir_diff) - PI) < 0.1 )   //back up
			{
				vx = -0.2;
				vy = 0;
				w = 0;
			}
			else if( abs(abs(dir_diff) - PI/2) < 0.1 && dist_CurrPt2Waypt < 1.0)    //Within PI/2 and within 0.5m
			{
				if(dir_diff > 0)    //Left
				{
					vx = 0;
					vy = 0.2;
					w = 0;
				}
				else				//Right
				{
					vx = 0;
					vy = -0.2;
					w = 0;
				}

			}
			else if( abs(abs(dir_diff) - 3*PI/2) < 0.1  && dist_CurrPt2Waypt < 1.0 )    //Within 3*PI/2 and within 0.5m
			{
				if(dir_diff > 0)    //Right
				{
					vx = 0;
					vy = -0.2;
					w = 0;
				}
				else				//Left
				{
					vx = 0;
					vy = 0.2;
					w = 0;
				}
			}
			else
			{
				if( abs(dir_diff) >= 0.2)     //Æ«²îŽóÓÚPI/6»¡¶È
				{
					if(dir_diff > 0)	{w = 0.4;}
					else {w = -0.4;}
					vx = 0.15;
					vy = 0;
				}
				else if( abs(dir_diff) < 0.2 && abs(dir_diff) > 0.1 )
				{
					if(dir_diff > 0)	{w = 0.2;}
					else	{w = -0.2;}	
					vx = 0.15;
					vy = 0;
				}
				else
				{
					w = 0;
					vx = 0.2;
					vy = 0;
				}		
			}
		}
		else
		{
			nextWpt++;
		}
	
	}
	else
	{
		vx = 0;
		vy = 0;
		w = 0;
	}

	return 1;
}

void my_planner::control_loop()
{
  ros::Rate rate(control_frequency_);
  tf::Stamped < tf::Pose > pose;
  double vx=0,vy=0,w=0.0;
  int nextWpt = 1;
  while(ros::ok())
  {
	   if (getRobotPose (pose))
	   {
	     double c_x = pose.getOrigin().x(),
	            c_y = pose.getOrigin().y(),
	            c_th = tf::getYaw(pose.getRotation());
	     if(Planner_2(c_x, c_y, c_th, pts, vx, vy, w, nextWpt))
	     {
	    	 cmd.linear.x = vx;
	    	 cmd.linear.y = vy;
	    	 cmd.angular.z = w;
	    	 vel_pub_.publish(cmd);
	     }
        ROS_INFO("c_x,c_y,c_th,vx,vy,w,nextWpt:%.3f, %.3f, %.3f,%.3f,%.3f,%.3f,%d",c_x, c_y, c_th, vx, vy, w, nextWpt);
	   }
	  rate.sleep();
	  ros::spinOnce();
  }

}

bool my_planner::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const
{
  global_pose.setIdentity();
  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = robot_base_frame_;
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try
  {
    tf_.transformPose(global_frame_, robot_pose, global_pose);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.stamp_.toSec(), transform_tolerance_);
    return false;
  }

  return true;
}


std::vector<struct waypoint> my_planner::makeWayPointsFromParams(ros::NodeHandle& nh)
{
  std::string full_param_name;
  std::string full_radius_param_name;
  std::vector<struct waypoint> points;

  if (nh.searchParam("waypoints", full_param_name))
  {
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    nh.getParam(full_param_name, footprint_xmlrpc);
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      points = makeWaypointsFromXMLRPC(footprint_xmlrpc, full_param_name);
    }
  }
  
  return points;
}



double my_planner::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
    throw std::runtime_error("Values in the footprint specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

std::vector<struct waypoint> my_planner::makeWaypointsFromXMLRPC(XmlRpc::XmlRpcValue& waypoints_xmlrpc,
                                const std::string& full_param_name)
{
  // Make sure we have an array of at least 3 elements.
  if (waypoints_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
               full_param_name.c_str(), std::string(waypoints_xmlrpc).c_str());
    throw std::runtime_error("The footprint must be specified as list of lists on the parameter server  "
                             "eg: [[x1, y1, theta1], [x2, y2, theta2], ..., [xn, yn, thetan]]");
  }

  std::vector<struct waypoint> waypoints;
  struct waypoint pt;

  for (int i = 0; i < waypoints_xmlrpc.size(); ++i)
  {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = waypoints_xmlrpc[ i ];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        point.size() != 3)
    {
      ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                "[[x1, y1, theta1], [x2, y2, theta2], ..., [xn, yn,thetan]], but this spec is not of that form.",
                 full_param_name.c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1, theta1], [x2, y2, theta2], ..., [xn, yn,thetan]], but this spec is not of that form");
    }

    pt.x = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y = getNumberFromXMLRPC(point[ 1 ], full_param_name);
    pt.theta = getNumberFromXMLRPC(point[ 2 ], full_param_name);
    waypoints.push_back(pt);
  }
  return waypoints;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_planner");
  ros::NodeHandle n("~");
  tf::TransformListener tf;
  my_planner mp(tf);
  mp.control_loop();

  return 0;


}
