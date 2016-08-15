#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include "stdio.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "math.h"
#include "XmlRpcValue.h"
#include <boost/bind.hpp>

#define PI 3.141592
#define INFINITY 9999.0
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
	std::vector<std::vector<double> > WP_array;
	std::vector<std::vector<double> > Adj_Pair_array;
	double **Graph;
	Waypoints pts;
	geometry_msgs::Twist cmd;
	ros::Publisher vel_pub_;
	ros::Subscriber goal_sub_;
	std::string cmd_pub_topic;
	std::string goal_sub_topic;
	ros::NodeHandle n_;
	bool goal_changed;
	bool triggered;
	geometry_msgs::PoseStamped goal_;

public:
	my_planner(tf::TransformListener &tf);
	~my_planner()   { }


	std::vector<std::vector<double> > makeArrayFromParams(ros::NodeHandle& nh, const std::string& param_name);
	double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);
	std::vector<std::vector<double> > makeArrayFromXMLRPC(XmlRpc::XmlRpcValue& array_xmlrpc,
                                const std::string& full_param_name);
    bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;
    bool createGraph(double **Graph, Waypoints pts, 	std::vector<std::vector<double> > Adj_Pair_array);
    void display_graph(double **Graph, int r, int c);
    void control_loop();
    vector<int> Floyd(double **matr_topo, int size, int startIdx, int endIdx);
    bool Planner_2(double c_x, double c_y, double c_th, Waypoints pts, double &vx, double &vy, double &w, int &nextWpt);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
    int FindTheClosestWayPoints(double x, double y, Waypoints W_pts); //find the index of the closest way points to the current x,y
    void redefine_waypoints(double c_x, double c_y, double c_th,
    		double g_x, double g_y, double g_th, Waypoints &W_pts);

};




my_planner::my_planner(tf::TransformListener &tf):tf_(tf),goal_changed(false)
{
	ros::NodeHandle n_local("~");
	n_local.param("global_frame",global_frame_, std::string("map"));
	n_local.param("robot_base_frame",robot_base_frame_, std::string("base_link"));
	n_local.param("transform_tolerance",transform_tolerance_, 0.8);
	n_local.param("control_frequency",control_frequency_, 10.0);
    n_local.param("cmd_pub_topic",cmd_pub_topic, std::string("cmd_vel"));
    n_local.param("goal_sub_topic",goal_sub_topic, std::string("goal"));


    vel_pub_ = n_.advertise<geometry_msgs::Twist>(cmd_pub_topic.c_str(), 10);

    goal_sub_ = n_.subscribe<geometry_msgs::PoseStamped>(goal_sub_topic.c_str(), 10, boost::bind(&my_planner::goalCB, this, _1));
    //printf("starting to read parameters\n");
    WP_array = makeArrayFromParams(n_local, std::string("waypoints"));
    Adj_Pair_array = makeArrayFromParams(n_local, std::string("adj_pairs"));

    //printf("waypoints are read successfully\n");
    struct waypoint tmp_wp;
    int i,j;
    int len_wp_array = WP_array.size();
    for(j=0;j<len_wp_array;j++)
    {
    	tmp_wp.x = WP_array[j][0];
    	tmp_wp.y = WP_array[j][1];
    	tmp_wp.theta = WP_array[j][2];
    	pts.push_back(tmp_wp);
    }

    Graph = (double**)(new double[len_wp_array]);

    for(i = 0; i < len_wp_array; ++ i)
    	Graph[i] = (double*)new double[len_wp_array];
    for(i = 0; i < len_wp_array; ++ i)
       for(j = 0; j < len_wp_array; ++ j)
       {
    	   if(i==j)
    		   Graph[i][j] = 0.0;
    	   else
    	       Graph[i][j] = INFINITY;
       }

    createGraph(Graph, pts,  Adj_Pair_array);

    display_graph(Graph, len_wp_array, len_wp_array);

    for(i=0;i<pts.size();i++)
	  {
		  printf("%d th point is: %f, %f, %f\n",i,pts[i].x, pts[i].y, pts[i].theta );
	  }

    for(i=0;i<Adj_Pair_array.size();i++)
    {	printf("%d th row is:",i);
    	for(j=0;j<Adj_Pair_array[i].size();j++)
	  {
		  printf("%f ",Adj_Pair_array[i][j]);
	  }
    	printf("\n");
    }


/*    std::vector<int> wp_index =  Floyd(Graph, pts.size(), 5, 1);
    for(i=0;i<wp_index.size();i++)
    {
    	printf("%d->",wp_index[i]);
    }
    printf("\n");*/


}

void my_planner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{

	goal_changed = true;
	goal_ = *goal;
	ROS_INFO("goal received");
	triggered = true;

}

bool my_planner::createGraph(double **Graph, Waypoints pts, 	std::vector<std::vector<double> > Adj_Pair_array)
{
	int paired_index_i, paired_index_j;
	double dis, xi, yi, xj, yj;
	if(Graph==NULL)
	{
		ROS_ERROR("Graph is not initialized!");
		return false;
	}
	for(int i=0;i<Adj_Pair_array.size();i++)
	{
		paired_index_i = (int)Adj_Pair_array[i][0];
		paired_index_j = (int)Adj_Pair_array[i][1];
		xi = pts[paired_index_i].x;
		yi = pts[paired_index_i].y;
		xj = pts[paired_index_j].x;
		yj = pts[paired_index_j].y;
		dis  = sqrt((xi-xj)*(xi-xj) + (yi-yj)*(yi-yj) );
		Graph[paired_index_i][paired_index_j] = dis;
		Graph[paired_index_j][paired_index_i] = dis;
	}
	return true;
}

void my_planner::display_graph(double **Graph, int r, int c)
{
	int i,j;
	printf("The Graph is:\n");
	for(i = 0; i < r; ++ i)
	    {for(j = 0; j < c; ++ j)
	        printf("%f ",Graph[i][j]);
	     printf("\n");
	    }

}



///////////////////////////////////////////////////////////////////////
// This function is used to find the shortest  path from starting position to  ending position

vector<int> my_planner::Floyd(double **matr_topo, int size, int startIdx, int endIdx)
{
	int max_hig = size;
	int max_len = size;
	int i, j, k;
	vector<int> wayP_index;
	vector<int>::iterator it;
	it = wayP_index.begin();

/************************************************************************/
/*       ¶¯Ì¬Éú³É¶þÎ¬Êý×é Dist							                */
/************************************************************************/
	double **Dist = new double*[max_hig];
	for(int g1=0; g1<max_hig; g1++)
	{
		Dist[g1]=new double[max_len];
	}

	/*Initial the array*/
	for(int m1=0; m1<max_hig; m1++)
	{
		for (int n1=0; n1<max_len; n1++)
		{
			Dist[m1][n1] =matr_topo[m1][n1];
		}
	}

/************************************************************************/
/*       ¶¯Ì¬Éú³É¶þÎ¬Êý×é  Route								                */
/************************************************************************/
	int **Route = new int*[max_hig];
	for(int g2=0; g2<max_hig; g2++)
	{
		Route[g2]=new int[max_len];
	}

	/*Initial the array*/
	for(int m2=0; m2<max_hig; m2++)
	{
		for (int n2=0; n2<max_len; n2++)
		{
			Route[m2][n2] =0;
		}
	}

	for(i=0; i<max_hig; i++)
	{
		for(j=0; j<max_len; j++)
		{
			Dist[i][j]=matr_topo[i][j];
		}
	}

	int n = size;
	for(i=0; i<n; i++)
	{
		for(j=0; j<n; j++)
		{
			Route[i][j] = i;
		}

	}

	for(k=0; k<n; k++)
	{
		for(i=0; i<n; i++)
		{
			for(j=0; j<n; j++)
			{
				if(Dist[i][k]+Dist[k][j] < Dist[i][j])
				{
					Dist[i][j] = Dist[i][k] + Dist[k][j];
					Route[i][j] = Route[k][j];
				}

			}

		}

	}

	wayP_index.push_back(startIdx);
	wayP_index.push_back(endIdx);

	it = wayP_index.begin();

	while(1)
	{
		int prev_size = wayP_index.size();
		//vector<int> wayP_index_cpy = wayP_index;
		for(int idx=0; idx<prev_size-1; idx++ )
			{
				if(Route[wayP_index[idx]][wayP_index[idx+1]] != wayP_index[idx] &&
					Route[wayP_index[idx]][wayP_index[idx+1]] != wayP_index[idx+1] )
				{
					 wayP_index.insert(it+idx+1, Route[wayP_index[idx]][wayP_index[idx+1]]);
					 printf("inserted %d\n",Route[wayP_index[idx]][wayP_index[idx+1]]);
					 printf("size of wayP_index is %d\n",wayP_index.size());
					 it = wayP_index.begin();
					 break;
				}

			}
		int curr_size = wayP_index.size();
		if(curr_size==prev_size)
			break;
	}

	printf("Route is: \n");
	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
			printf("%d ",Route[i][j]);
		printf("\n");

	}

	for(i=0; i<max_hig; i++)
	{
		delete[] Dist[i];
		delete[] Route[i];
	}
	delete[] Dist;
	delete[] Route;

	return wayP_index;
}


bool my_planner::Planner_2(double c_x, double c_y, double c_th, Waypoints pts, double &vx, double &vy, double &w, int &nextWpt)
{
	double dir_slope = 0;
	double dir_diff = 0;
	double dir_rad = 0; 

	if(nextWpt < pts.size())
	{
		double dist_CurrPt2Waypt = sqrt((pts[nextWpt].y-c_y)*(pts[nextWpt].y-c_y)+(pts[nextWpt].x-c_x)*(pts[nextWpt].x-c_x));

		if(dist_CurrPt2Waypt > 0.15)			//within 0.2m
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
			while(dir_diff  > PI)
			{
				dir_diff  -= 2*PI;
			}
			while(dir_diff  < -PI)
			{
				dir_diff  += 2*PI;
			}

			if( abs(abs(dir_diff) - PI) < 0.1 )   //back up
			{
				vx = -0.2;
				vy = 0;
				w = 0;
			}
			else if( abs(abs(dir_diff) - PI/2) < 0.2 && dist_CurrPt2Waypt < 1.5)    //Within PI/2 and within 0.5m
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
			else if( abs(abs(dir_diff) - 3*PI/2) < 0.2  && dist_CurrPt2Waypt < 1.5 )    //Within 3*PI/2 and within 0.5m
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
					if(dir_diff > 0)	{w = 0.6;}
					else {w = -0.6;}
					vx = 0.0;
					vy = 0;
				}
				else
				{
					vx = 0.2;
					vy = 0;

					if(abs(dir_diff) > 0.02)
					{
						if(dir_diff > 0)
						{
							w = 0.1;
						}
						else
						{
							w = -0.1;
						}
					}
					else
						w =0;
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
  Waypoints W_pts;
  int nextWpt = 1;
  while(ros::ok())
  {


	   if (getRobotPose (pose))
	   {
	     double c_x = pose.getOrigin().x(),
	            c_y = pose.getOrigin().y(),
	            c_th = tf::getYaw(pose.getRotation());

	     if(goal_changed)
	     {
	    	 goal_changed = false;
	    	 W_pts = pts;
	    	 redefine_waypoints(c_x, c_y, c_th,
	    	 		goal_.pose.position.x, goal_.pose.position.y, tf::getYaw(goal_.pose.orientation), W_pts);
	    	 nextWpt = 1;
	     }

	     if(triggered && !W_pts.empty())
	     {
	    	 if(Planner_2(c_x, c_y, c_th, W_pts, vx, vy, w, nextWpt))
	    	 {
	    		 cmd.linear.x = vx;
	    		 cmd.linear.y = vy;
	    		 cmd.angular.z = w;
	    		 vel_pub_.publish(cmd);
	    	 }
	    	 //ROS_INFO("c_x,c_y,c_th,vx,vy,w,nextWpt:%.3f, %.3f, %.3f,%.3f,%.3f,%.3f,%d",c_x, c_y, c_th, vx, vy, w, nextWpt);
	     }
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


std::vector<std::vector<double> >  my_planner::makeArrayFromXMLRPC(XmlRpc::XmlRpcValue& array_xmlrpc,
                            const std::string& full_param_name)
{
	 // Make sure we have an array of at least 3 elements.
	  if (array_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray)
	  {
	    ROS_FATAL("The array must be specified as list of lists on the parameter server, %s was specified as %s",
	               full_param_name.c_str(), std::string(array_xmlrpc).c_str());
	    throw std::runtime_error("The array must be specified as list of lists on the parameter server  "
	                             "eg: [[x1, y1, z1...], [x2, y2, z2...], ..., [xn, yn, zn...]]");
	  }

	  std::vector<std::vector<double> > array;
	  std::vector<double> pt;

	 // printf("starting to make array!\n");

	  for (int i = 0; i < array_xmlrpc.size(); ++i)
	  {
	    // Make sure each element of the list is an array of size 2. (x and y coordinates)
	    XmlRpc::XmlRpcValue point = array_xmlrpc[ i ];
	    //printf("XmlRpcValue to point!\n");

	    XmlRpc::XmlRpcValue::Type t= point.getType() ;
	    // printf("get type successfully!\n");
	     //printf("get number from xml\n");
	   /*if (t != XmlRpc::XmlRpcValue::TypeArray )
	    {
	    	printf("point.getType() != XmlRpc::XmlRpcValue::TypeArray!\n");
	      ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
	                "[[x1, y1, theta1], [x2, y2, theta2], ..., [xn, yn,thetan]], but this spec is not of that form.",
	                 full_param_name.c_str());
	      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
	                               "[[x1, y1, z1 ...], [x2, y2, z2...], ..., [xn, yn,zn...]], but this spec is not of that form");
	    }*/
	   // printf("size of point is %d\n",point.size());
	    pt.clear();
	    for(int j=0;j<point.size();j++)
	    {
	    	pt.push_back(getNumberFromXMLRPC(point[ j ], full_param_name));
	    }

	    array.push_back(pt);
	  }
	  return array;

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


std::vector<std::vector<double> > my_planner::makeArrayFromParams(ros::NodeHandle& nh, const std::string& param_name)
{

	 std::string full_param_name;
	  std::string full_radius_param_name;
	  std::vector<std::vector<double> > points;

	  if (nh.searchParam(param_name.c_str(), full_param_name))
	  {
		//printf("search param successfully!\n");
	    XmlRpc::XmlRpcValue footprint_xmlrpc;
	    nh.getParam(full_param_name, footprint_xmlrpc);
	    //printf("getParam successfully!\n");
	    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
	    {
	      points = makeArrayFromXMLRPC(footprint_xmlrpc, full_param_name);
	    }
	  }

	  return points;

}

int my_planner::FindTheClosestWayPoints(double x, double y, Waypoints W_pts)
{
	double min_dist;
	int min_index;
	for(int i=0;i<W_pts.size();i++)
	{
		double dist = sqrt((x-W_pts[i].x)*(x-W_pts[i].x)+(y-W_pts[i].y)*(y-W_pts[i].y));
		if(i==0) min_dist = dist;
		if(dist<=min_dist)
		{
			min_dist = dist;
			min_index = i;
		}
	}

	return min_index;
}

void my_planner::redefine_waypoints(double c_x, double c_y, double c_th,
		double g_x, double g_y, double g_th, Waypoints &W_pts)
{
	int start_index = FindTheClosestWayPoints(c_x, c_y, W_pts);
	int end_index = FindTheClosestWayPoints(g_x, g_y, W_pts);

	std::vector<int> wp_index =  Floyd(Graph, W_pts.size(), start_index, end_index);
	W_pts.clear();

	struct waypoint start_wp;
	start_wp.x = c_x;
	start_wp.y = c_y;
	start_wp.theta = c_th;
	W_pts.push_back(start_wp);

	for(int i=0;i<wp_index.size();i++)
	{
		W_pts.push_back(pts[wp_index[i]]);
	}

	struct waypoint end_wp;
	end_wp.x = g_x;
	end_wp.y = g_y;
	end_wp.theta = g_th;
	W_pts.push_back(end_wp);


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
