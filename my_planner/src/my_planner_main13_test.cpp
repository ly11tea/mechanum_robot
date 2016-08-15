#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include "stdio.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "math.h"
#include <cmath>
#include <algorithm>
#include "XmlRpcValue.h"
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>
#include <queue>

#define PI 3.141592
#define INFINITY 9999.0



using namespace std;


////////////////////////////////////////////////////////////////////////

struct waypoint
{
	double x;
	double y;
	double theta;
};

//typedef vector<struct waypoint> Waypoints;


struct point_2d
{
    double x;
    double y;
	double c2laser_dist;
	double lat_dist;
	double forw_dist;
	double diff_angle;
};

typedef std::vector<struct waypoint> Waypoints;
typedef std::vector<struct point_2d> LaserPs;

struct Obst_Mode_Wpts
{
	int motionMode;
	Waypoints pts_local;
};

struct Avoid_Obst_Wpts
{
	bool valid_Wpts;
	Waypoints pts_local;
};

//////////////////////////////////////////////////////////////////////////////////////////
struct obstacleState
{
    bool obst_exist_C2N;
	point_2d ForwLasP_Farthest;
	point_2d ForwLasP_Nearest;

	waypoint feasible_Wpt;

};



bool less_angle(const point_2d& s1, const point_2d& s2)
{  	return s1.diff_angle < s2.diff_angle;  }
bool greater_angle(const point_2d& s1, const point_2d& s2)
{  	return s1.diff_angle > s2.diff_angle;  }

bool less_LatDist(const point_2d& s1, const point_2d& s2)
{  	return s1.lat_dist < s2.lat_dist;   }
bool greater_LatDist(const point_2d& s1, const point_2d& s2)
{  	return s1.lat_dist > s2.lat_dist;   }

bool less_ForwDist(const point_2d& s1, const point_2d& s2)
{  	return s1.forw_dist < s2.forw_dist;   }
bool greater_ForwDist(const point_2d& s1, const point_2d& s2)
{  	return s1.forw_dist > s2.forw_dist;   }


//struct waypoint FindWptToAvoidObst(std::vector <struct point_2d> laser_ps, double c_x, double c_y, double c_th,
//									   Waypoints &pts, int nextWpt, double tol_obst_Forw, double tol_obst_lat)


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
	Waypoints pts;//Initial Waypoints;
	Waypoints Redefine_pts;//redefined
	Waypoints pts_local;
	Waypoints pts_local_saved;
	geometry_msgs::Twist cmd;
	ros::Publisher vel_pub_;
	ros::Subscriber goal_sub_;
	std::string cmd_pub_topic;
	std::string goal_sub_topic;
	std::string scan_sub_topic;
	std::string cloud_pub_topic;
	ros::NodeHandle n_;
	bool goal_changed;
	bool triggered;
	geometry_msgs::PoseStamped goal_;
	ros::Publisher point_cloud_publisher_;
	ros::Publisher marker_pub ;
    ros::Subscriber scan_sub_;
    Obst_Mode_Wpts wpt_mode;
    double c_x,c_y,c_th;
    bool get_pose_successful;
    int nextWpt;
    int nextWpt_local;
    laser_geometry::LaserProjection projector_;
    queue<double> q_motion_mode;

    double avg_motion_mode ;
    ros::Time time_avoid_obs;

    boost::thread* ObsAvoid_thread_;

    //queue of laser scan
    queue<sensor_msgs::PointCloud> q_laser_scan;
    int queue_size_laser_scan;
    boost::mutex obstacle_avoidance_mutex_;

    double tol_obst_Forw_;
    double tol_obst_lat_;
    double online_angle_adj;//oneline angle slight adjustment

    int last_motion_mode; //the last motion mode
    bool in_obstacle_avoidance;
    bool global_redefined;//whether global defined
    int Global_Planner;
    int Local_Planner;

    double DIST_TOLE ;
    double DIST_TOLE_LOCAL ;
    double DIR_TOLE  ;  //0.2rad
    double NORM_SPEED ;  //0.2 m/s
    double DIR_TOLE_LOCAL;
    double NORM_SPEED_LOCAL;
    double MAX_LAT_SPEED;
    double MIN_LAT_SPEED;
    double BIG_REGU;
    double SMALL_REGU;
    double MED_REGU;
    double LAT_DIST_TOL;



public:
	my_planner(tf::TransformListener &tf);
	~my_planner()   {

		ObsAvoid_thread_->interrupt();
		ObsAvoid_thread_->join();

	    delete ObsAvoid_thread_;

	}


	std::vector<std::vector<double> > makeArrayFromParams(ros::NodeHandle& nh, const std::string& param_name);
	double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);
	std::vector<std::vector<double> > makeArrayFromXMLRPC(XmlRpc::XmlRpcValue& array_xmlrpc,
                                const std::string& full_param_name);
    bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;
    bool createGraph(double **Graph, Waypoints pts, 	std::vector<std::vector<double> > Adj_Pair_array);
    void display_graph(double **Graph, int r, int c);
    void control_loop();
    vector<int> Floyd(double **matr_topo, int size, int startIdx, int endIdx);
    bool Planner_2(double c_x, double c_y, double c_th, Waypoints &pts, double &vx, double &vy, double &w, int &nextWpt);
    bool Planner_Local(double c_x, double c_y, double c_th, Waypoints &pts, double &vx, double &vy, double &w, int &nextWpt);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
    int FindTheClosestWayPoints(double x, double y, Waypoints W_pts); //find the index of the closest way points to the current x,y
    void redefine_waypoints(double c_x, double c_y, double c_th,
    		double g_x, double g_y, double g_th, Waypoints &W_pts);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void PublishMarker(double x, double y, std::string frame, uint32_t shape,
    		std::string ns, int32_t id, bool red, bool green, bool blue);
    void PublishAllInitWpts(); //publish all the initial waypoints
    void PublishAllPtsLocal();

    double AverageFilter(queue<double> &q, int window_size, double curr_value);
    double FindAverage(queue<double> q);

   void find_points_dis_parrallel(double A1, double B1,double C1,double x0,double y0,double L,
    							   double &x1, double &x2, double &y1, double &y2);

    void find_points_dis_perpen(double A1, double B1,double C1,double x0,double y0,double L,
    							double &x1, double &x2, double &y1, double &y2);

    double getDist_P2L(double currPoint_x, double currPoint_y, double A, double B, double C);

    void getParaLineEqu(double Point_x, double Point_y,double A, double B, double C, double &A_para, double &B_para, double &C_para);

    void get_LineEqu(double PrevP_x, double PrevP_y, double NextP_x, double NextP_y,
    							double &A, double &B, double &C);

    double cal_diff_rad(double start_rad, double target_rad );

    void find_vertical_points(double A0, double B0, double C0, double x0, double y0,
    							double &vert_x, double &vert_y);

    obstacleState DetectObstacle(LaserPs &laser_ps, waypoint curr_point, waypoint next_point, double tol_obst_Forw, double tol_obst_lat);

    Avoid_Obst_Wpts FindWptToAvoidObst( obstacleState Obst_State, double c_x, double c_y, double c_th, waypoint prev_Wpt, waypoint next_Wpt);

    bool Planner_Local(double c_x, double c_y, double c_th, Waypoints &pts_Local, int &nextWpt_Local, double &vx, double &vy, double &w);
};

my_planner::my_planner(tf::TransformListener &tf):tf_(tf),goal_changed(false),get_pose_successful(false),triggered(false)
{
	ros::NodeHandle n_local("~");
	n_local.param("global_frame",global_frame_, std::string("map"));
	n_local.param("robot_base_frame",robot_base_frame_, std::string("base_link"));
	n_local.param("transform_tolerance",transform_tolerance_, 0.8);
	n_local.param("control_frequency",control_frequency_, 10.0);
    n_local.param("cmd_pub_topic",cmd_pub_topic, std::string("cmd_vel"));
    n_local.param("goal_sub_topic",goal_sub_topic, std::string("goal"));
    n_local.param("scan_sub_topic",scan_sub_topic, std::string("scan"));
    n_local.param("cloud_pub_topic",cloud_pub_topic, std::string("cloud"));
    n_local.param("queue_size_laser_scan",queue_size_laser_scan, 5);
    n_local.param("tol_obst_Forw",tol_obst_Forw_, 0.6);
    n_local.param("tol_obst_lat",tol_obst_lat_, 0.2);
    n_local.param("online_angle_adj",online_angle_adj, 0.03);


    n_local.param("DIST_TOLE",DIST_TOLE, 0.15);
    n_local.param("DIST_TOLE_LOCAL",DIST_TOLE_LOCAL, 0.2);
    n_local.param("DIR_TOLE",DIR_TOLE, 0.2);
    n_local.param("NORM_SPEED",NORM_SPEED, 0.22);
    n_local.param("DIR_TOLE_LOCAL",DIR_TOLE_LOCAL, 0.15);

    n_local.param("NORM_SPEED_LOCAL",NORM_SPEED_LOCAL, 0.2);
    n_local.param("MAX_LAT_SPEED",MAX_LAT_SPEED, 0.05);
    n_local.param("MIN_LAT_SPEED",MIN_LAT_SPEED, 0.04);
    n_local.param("BIG_REGU",BIG_REGU, 0.6);
    n_local.param("SMALL_REGU",SMALL_REGU, 0.03);
    n_local.param("MED_REGU",MED_REGU, 0.2);
    n_local.param("LAT_DIST_TOL",LAT_DIST_TOL, 0.05);

    vel_pub_ = n_.advertise<geometry_msgs::Twist>(cmd_pub_topic.c_str(), 10);

    marker_pub = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    goal_sub_ = n_.subscribe<geometry_msgs::PoseStamped>(goal_sub_topic.c_str(), 10, boost::bind(&my_planner::goalCB, this, _1));
    
    scan_sub_ = n_.subscribe<sensor_msgs::LaserScan> (scan_sub_topic.c_str(), 100, &my_planner::scanCallback, this);
    
    point_cloud_publisher_ = n_.advertise<sensor_msgs::PointCloud> (cloud_pub_topic.c_str(), 100, false);
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

    wpt_mode.motionMode = 0;


/*    std::vector<int> wp_index =  Floyd(Graph, pts.size(), 5, 1);
    for(i=0;i<wp_index.size();i++)
    {
    	printf("%d->",wp_index[i]);
    }
    printf("\n");*/

    nextWpt = 1;
    nextWpt_local = 1;

    last_motion_mode = 0;

    in_obstacle_avoidance = false;
    global_redefined = false;


    Global_Planner = 1;
    Local_Planner = 0;
    // publish all the initial wpts;

}

void my_planner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{

	goal_changed = true;
	goal_ = *goal;
	ROS_INFO("goal received");
	triggered = true;

}

void my_planner::PublishAllInitWpts()
{
	if(pts.size()==0)
		return;
	std::string ns_wps("Init_Wpts");
	for(int i=0;i<pts.size();i++)
	{
		double x = pts[i].x;
		double y = pts[i].y;
		PublishMarker( x,  y, "/map", visualization_msgs::Marker::CUBE, ns_wps,i,false,true,false);//green color cube
	}
}

void my_planner::PublishAllPtsLocal()
{

	if(pts_local.size()==0)
		return;
	std::string ns_wps("Local_Wpts");
	for(int i=0;i<pts_local_saved.size();i++)
	{
		double x = pts_local_saved[i].x;
		double y = pts_local_saved[i].y;
		PublishMarker( x,  y, "/map", visualization_msgs::Marker::SPHERE, ns_wps,i,true,false,false);//red color cube
	}

}

void my_planner::PublishMarker(double x, double y, std::string frame, uint32_t shape,
		std::string ns, int32_t id, bool red, bool green, bool blue)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame.c_str();
	marker.header.stamp = ros::Time::now();
	marker.type = shape;
	marker.ns = ns;
	marker.id = id;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.color.r = 1.0f*red;

	marker.color.g = 1.0f*green;

	marker.color.b = 1.0f*blue;

	marker.color.a = 1.0;
	marker.scale.x = 0.05;//5cm
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.lifetime = ros::Duration();
	if(marker_pub.getNumSubscribers() < 1)
		return;
	marker_pub.publish(marker);
	//ROS_INFO("publishing marker");

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

vector<int> my_planner::Floyd(double **matr_topo, int size, int startIdx, int endIdx)
{
	int max_hig = size;
	int max_len = size;
	int i, j, k;
	vector<int> wayP_index;
	if(startIdx==endIdx)
	{
		wayP_index.push_back(startIdx);
		return wayP_index;
	}
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


////////////////////////////////////////////////////////////////////////
bool my_planner::Planner_2(double c_x, double c_y, double c_th, Waypoints &pts, double &vx, double &vy, double &w, int &nextWpt)
{
waypoint  past_Wp;
waypoint  next_Wp;
double A, B, C, D;
double lat_dist = 0.0;
double p2c_slope = 0.0;
double p2n_slope = 0.0;
double c2n_slope = 0.0;

//	double dir_diff = 0.0;
double dirLOS_diff = 0.0;
double c2p2n_diff = 0.0;

if(nextWpt < pts.size())
{
	/*if(nextWpt == 1 && nextWpt == pts.size()-2)
	{
		past_Wp = pts[nextWpt];
		next_Wp = pts[nextWpt+1];
		A = next_Wp.y - past_Wp.y; B = past_Wp.x - next_Wp.x; C = next_Wp.x*past_Wp.y - past_Wp.x*next_Wp.y;
		D = c_y*A - c_x*B;
		pts[nextWpt].x = (A*D-B*C)/(A*A+B*B);
		pts[nextWpt].y = (-A*C-B*D)/(A*A+B*B);
	}
	else*/

	if(nextWpt == 1 )
	{
		double rad_1_2 = atan2(pts[nextWpt+1].y-pts[nextWpt].y, pts[nextWpt+1].x-pts[nextWpt].x);
		double rad_1_0 = atan2(pts[nextWpt-1].y-pts[nextWpt].y, pts[nextWpt-1].x-pts[nextWpt].x);
		double diff_0_1_2 = rad_1_2 - rad_1_0;
		while(diff_0_1_2 > PI)  {diff_0_1_2 -= 2*PI;}
		while(diff_0_1_2 < -PI) {diff_0_1_2 += 2*PI;}

		if(abs(diff_0_1_2) < PI/2)
		{
			past_Wp = pts[nextWpt];
			next_Wp = pts[nextWpt+1];
			A = next_Wp.y - past_Wp.y; B = past_Wp.x - next_Wp.x; C = next_Wp.x*past_Wp.y - past_Wp.x*next_Wp.y;
			D = c_y*A - c_x*B;
			pts[nextWpt].x = (-A*C-B*D)/(A*A+B*B);
			pts[nextWpt].y = (A*D-B*C)/(A*A+B*B);
		}
	}
	else if(nextWpt == pts.size()-2)
	{
		double rad_a_b = atan2(pts[nextWpt].y-pts[nextWpt-1].y, pts[nextWpt].x-pts[nextWpt-1].x);
		double rad_b_c = atan2(pts[nextWpt+1].y-pts[nextWpt].y, pts[nextWpt+1].x-pts[nextWpt].x);
		double diff_a_b_c = rad_a_b - rad_b_c;
		while(diff_a_b_c > PI)  {diff_a_b_c -= 2*PI;}
		while(diff_a_b_c < -PI) {diff_a_b_c += 2*PI;}

		if(abs(diff_a_b_c) > PI/2)
		{
			past_Wp = pts[nextWpt-1];
			next_Wp = pts[nextWpt];
			A = next_Wp.y - past_Wp.y; B = past_Wp.x - next_Wp.x; C = next_Wp.x*past_Wp.y - past_Wp.x*next_Wp.y;
			D = pts[nextWpt+1].y*A - pts[nextWpt+1].x*B;
			pts[nextWpt].x = (-A*C-B*D)/(A*A+B*B);
			pts[nextWpt].y = (A*D-B*C)/(A*A+B*B);
		}
	}

	double dist_CurrPt2Waypt = sqrt((pts[nextWpt].y-c_y)*(pts[nextWpt].y-c_y)+(pts[nextWpt].x-c_x)*(pts[nextWpt].x-c_x));

	p2n_slope = atan2(pts[nextWpt].y-pts[nextWpt-1].y, pts[nextWpt].x-pts[nextWpt-1].x);
	p2c_slope = atan2(c_y-pts[nextWpt-1].y, c_x-pts[nextWpt-1].x);
	c2n_slope = atan2(pts[nextWpt].y-c_y, (pts[nextWpt].x-c_x));
	while(p2n_slope > PI)  {p2n_slope -= 2*PI;}
	while(p2n_slope < -PI) {p2n_slope += 2*PI;}
	while(p2c_slope > PI)  {p2c_slope -= 2*PI;}
	while(p2c_slope < -PI) {p2c_slope += 2*PI;}
	while(c2n_slope > PI)  {c2n_slope -= 2*PI;}
	while(c2n_slope < -PI) {c2n_slope += 2*PI;}

//		dir_diff = p2n_slope - c_th;
//		while(dir_diff > PI)  { dir_diff -= 2*PI;}
//		while(dir_diff < -PI) { dir_diff += 2*PI;}

	c2p2n_diff = p2n_slope - p2c_slope;
	while(c2p2n_diff > PI)  {c2p2n_diff  -= 2*PI;}
	while(c2p2n_diff < -PI) {c2p2n_diff  += 2*PI;}

	dirLOS_diff = c2n_slope - c_th;
	while(dirLOS_diff > PI)  {dirLOS_diff -= 2*PI;}
	while(dirLOS_diff < -PI) {dirLOS_diff += 2*PI;}

	past_Wp = pts[nextWpt-1];
	next_Wp = pts[nextWpt];
	A = next_Wp.y - past_Wp.y; B = past_Wp.x - next_Wp.x; C = next_Wp.x*past_Wp.y - past_Wp.x*next_Wp.y;  //X2*Y1 - X1*Y2
	lat_dist = abs(A*c_x + B*c_y + C)/(sqrt(A*A + B*B));

	if(dist_CurrPt2Waypt > DIST_TOLE)			//without
	{
		if(dist_CurrPt2Waypt > 1.5)   //>=1.0m
		{
			if( abs(dirLOS_diff) <= PI/2 )
			{
				if( abs(dirLOS_diff) >= DIR_TOLE)     //Æ«²îŽóÓÚ
				{
					vx = 0.0;
					vy = 0.0;
					if(dirLOS_diff > 0)	{w = 0.6;}
					else {w = -0.6;}
				}
				else
				{
					vx = NORM_SPEED;
					if(lat_dist > LAT_DIST_TOL)
					{
						if(c2p2n_diff > 0) {vy = MIN_LAT_SPEED;}
						else {vy = -MIN_LAT_SPEED;}
					}
					else {vy = 0;}

					if(abs(dirLOS_diff) > DIR_TOLE/3)			//
					{
						if(dirLOS_diff > 0) {w = online_angle_adj;}
						else {w = -online_angle_adj;}
					}
					else {w =0;}
				}
			}
			else
			{
				double diff_Posi180 = c2n_slope + PI - c_th;
				while(diff_Posi180 > PI)  {diff_Posi180 -= 2*PI;}
				while(diff_Posi180 < -PI) {diff_Posi180 += 2*PI;}

				if( abs(diff_Posi180) >= DIR_TOLE)     //Æ«²îŽóÓÚ
				{
					vx = 0.0;
					vy = 0.0;
					if(diff_Posi180 > 0)	{w = 0.6;}
					else {w = -0.6;}
				}
				else
				{
					vx = -NORM_SPEED;
					if(lat_dist > LAT_DIST_TOL)
					{
						if(c2p2n_diff > 0) {vy = -MIN_LAT_SPEED;}
						else {vy = MIN_LAT_SPEED;}
					}
					else {vy = 0;}

					if(abs(diff_Posi180) > DIR_TOLE/3)			//
					{
						if(diff_Posi180 > 0) {w = online_angle_adj;}
						else {w = -online_angle_adj;}
					}
					else {w =0;}
				}
			}

		}
		else   //within 1.5m
		{
			if( abs(dirLOS_diff) <= PI/3 )
			{
				if( abs(dirLOS_diff) >= DIR_TOLE)     //Æ«²îŽóÓÚ
				{
					vx = 0.0;
					vy = 0.0;
					if(dirLOS_diff > 0)	{w = 0.6;}
					else {w = -0.6;}
				}
				else
				{
					vx = NORM_SPEED;
					if(lat_dist > LAT_DIST_TOL)
					{
						if(c2p2n_diff > 0) {vy = MIN_LAT_SPEED;}
						else {vy = -MIN_LAT_SPEED;}
					}
					else {vy = 0;}

					if(abs(dirLOS_diff) > DIR_TOLE/3)			//
					{
						if(dirLOS_diff > 0) {w = online_angle_adj;}
						else {w = -online_angle_adj;}
					}
					else {w =0;}
				}
			}
			else if( abs(dirLOS_diff) >= 2*PI/3 )
			{
				double diff_Posi180 = c2n_slope + PI - c_th;
				while(diff_Posi180 > PI)  {diff_Posi180 -= 2*PI;}
				while(diff_Posi180 < -PI) {diff_Posi180 += 2*PI;}

				if( abs(diff_Posi180) >= DIR_TOLE)     //Æ«²îŽóÓÚ
				{
					vx = 0.0;
					vy = 0.0;
					if(diff_Posi180 > 0)	{w = 0.6;}
					else {w = -0.6;}
				}
				else
				{
					vx = -NORM_SPEED;
					if(lat_dist > LAT_DIST_TOL)
					{
						if(c2p2n_diff > 0) {vy = -MIN_LAT_SPEED;}
						else {vy = MIN_LAT_SPEED;}
					}
					else {vy = 0;}

					if(abs(diff_Posi180) > DIR_TOLE/3)			//
					{
						if(diff_Posi180 > 0) {w = online_angle_adj;}
						else {w = -online_angle_adj;}
					}
					else {w =0;}
				}
			}
			else
			{
				if( dirLOS_diff <= 0 )
				{
					double diff_Posi90 = c2n_slope + PI/2 - c_th;
					while(diff_Posi90 > PI)  {diff_Posi90 -= 2*PI;}
					while(diff_Posi90 < -PI) {diff_Posi90 += 2*PI;}

					if( abs(diff_Posi90) >= DIR_TOLE)     //Æ«²îŽóÓÚ
					{
						vx = 0.0;
						vy = 0.0;
						if(diff_Posi90 > 0)	{w = 0.6;}
						else {w = -0.6;}
					}
					else
					{
						vx = 0;
						vy = -NORM_SPEED;
						if(abs(diff_Posi90) > DIR_TOLE/3)			//
						{
							if(diff_Posi90 > 0) {w = online_angle_adj;}
							else {w = -online_angle_adj;}
						}
						else {w =0;}
					}
				}
				else
				{
					double diff_Anti90 = c2n_slope - PI/2 - c_th;
					while(diff_Anti90 > PI)  {diff_Anti90 -= 2*PI;}
					while(diff_Anti90 < -PI) {diff_Anti90 += 2*PI;}

					if( abs(diff_Anti90) >= DIR_TOLE)     //Æ«²îŽóÓÚ
					{
						vx = 0.0;
						vy = 0.0;
						if(diff_Anti90 > 0)	{w = 0.6;}
						else {w = -0.6;}
					}
					else
					{
						vx = 0;
						vy = NORM_SPEED;
						if(abs(diff_Anti90) > DIR_TOLE/3)			//
						{
							if(diff_Anti90 > 0) {w = online_angle_adj;}
							else {w = -online_angle_adj;}
						}
						else {w =0;}
					}
				}
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

	double dir_diff_final = pts[pts.size()-1].theta - c_th;
	while(dir_diff_final > PI) {dir_diff_final -= 2*PI;}
	while(dir_diff_final < -PI) {dir_diff_final += 2*PI;}

	if( abs(dir_diff_final) >= 0.2)
	{
		if(dir_diff_final > 0)	{w = 0.6;}
		else {w = -0.6;}
	}
	else {w = 0;nextWpt++;}



}

return true;
}


void my_planner::control_loop()
{
  ros::Rate rate(control_frequency_);
  tf::Stamped < tf::Pose > pose;
  double vx=0,vy=0,w=0.0;
  LaserPs laser_ps;
  //Waypoints W_pts;

  bool plan_succ_local;
  Avoid_Obst_Wpts wpts_Local;
  waypoint curr_point;
  waypoint next_point;
  obstacleState Curr_state;
  int nextWpt_Local = 1;
  bool task_finished = true;
  while(ros::ok())
  {
	  if (getRobotPose (pose))
	  {
		  get_pose_successful = true;
		  c_x = pose.getOrigin().x(),
		  c_y = pose.getOrigin().y(),
		  c_th = tf::getYaw(pose.getRotation());

		  if(triggered && !pts.empty())
		  {
			  if(goal_changed)
			  {
				  Redefine_pts = pts;
				  redefine_waypoints(c_x, c_y, c_th,
						  goal_.pose.position.x, goal_.pose.position.y, tf::getYaw(goal_.pose.orientation), Redefine_pts);
				  nextWpt = 1;

				  global_redefined = true;
				  goal_changed = false;
				  task_finished = false;
			  }

			  if(Global_Planner == 1)
			  {

				  next_point.x = Redefine_pts[nextWpt].x;
				  next_point.y = Redefine_pts[nextWpt].y;
				  next_point.theta = Redefine_pts[nextWpt].theta;
			  }
			  else
			  {
				  next_point = wpts_Local.pts_local[nextWpt_Local];
			  }


			  int q_size = q_laser_scan.size();
			  if(q_size >= 1)
			  {
				  sensor_msgs::PointCloud  cloud = q_laser_scan.back();
				  laser_ps.clear();
				  for(int i=0;i<cloud.points.size();i++)
				  {
					  struct point_2d tmp_point;
					  tmp_point.x = cloud.points[i].x;
					  tmp_point.y = cloud.points[i].y;
					  laser_ps.push_back(tmp_point);
				  }

			  }
			  else
				  continue;


			  if(task_finished==false)
			  {
				  curr_point.x = c_x; curr_point.y = c_y; curr_point.theta = c_th;
				  Curr_state = DetectObstacle(laser_ps, curr_point, next_point, tol_obst_Forw_, tol_obst_lat_);
				  if(Curr_state.obst_exist_C2N == false  && Global_Planner == 1)
				  {
					  Local_Planner = 0;
					  Planner_2(c_x, c_y, c_th, Redefine_pts, vx, vy, w, nextWpt);
					  if(nextWpt>Redefine_pts.size())
					  {
						  task_finished = true;
					  }

				  }
				  else if(Curr_state.obst_exist_C2N == true && Global_Planner == 1  )
				  {
					  waypoint prev_Wpt = Redefine_pts[nextWpt-1];
					  waypoint next_Wpt = Redefine_pts[nextWpt];
					  wpts_Local = FindWptToAvoidObst( Curr_state, c_x, c_y, c_th, prev_Wpt, next_Wpt);
					  //display markers
					  if(wpts_Local.pts_local.size()==0)
						  return;
					  std::string ns_wps("Local_Wpts");
					  for(int i=0;i<wpts_Local.pts_local.size();i++)
					  {
						  double x = wpts_Local.pts_local[i].x;
						  double y = wpts_Local.pts_local[i].y;
						  PublishMarker( x,  y, "/map", visualization_msgs::Marker::SPHERE, ns_wps,i,true,false,false);//red color cube
					  }
					  //display markers

					  if(wpts_Local.valid_Wpts == false)
					  {
						  continue;
					  }
					  else
					  {

						  Planner_Local(c_x, c_y, c_th, wpts_Local.pts_local, nextWpt_Local, vx, vy, w);
						  Local_Planner = 1;
						  Global_Planner = 0;
						  next_point = wpts_Local.pts_local[nextWpt_Local];
					  }
				  }
				  else if(Curr_state.obst_exist_C2N == true && Global_Planner == 0 )
				  {
					  vx = vy = w = 0;
				  }
				  else if(Curr_state.obst_exist_C2N == false && Global_Planner == 0 )
				  {
					  if(nextWpt_Local >= wpts_Local.pts_local.size()-1)
					  {
						  Global_Planner = 1 ;
						  Local_Planner = 0;
						  nextWpt_Local = 1;
					  }
					  else
					  {
						  Planner_Local(c_x, c_y, c_th, wpts_Local.pts_local, nextWpt_Local, vx, vy, w);
						  next_point = wpts_Local.pts_local[nextWpt_Local];
					  }
				  }
			  }
			  else
			  {
				  vx = 0;
				  vy = 0;
				  w = 0;
			  }


		  }

		  cmd.linear.x = vx;
		  cmd.linear.y = vy;
		  cmd.angular.z = w;

		  vel_pub_.publish(cmd);
	  }
	  //rate.sleep();
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

	printf("all the waypoints:");
	for(int i=0;i<wp_index.size();i++)
	{
		printf("%d", wp_index[i]);
		W_pts.push_back(pts[wp_index[i]]);
	}
	printf("\n");

	struct waypoint end_wp;
	end_wp.x = g_x;
	end_wp.y = g_y;
	end_wp.theta = g_th;
	W_pts.push_back(end_wp);


}


void my_planner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){

//ROS_INFO("SCAN called back!");


	//boost::unique_lock<boost::mutex> lock(obstacle_avoidance_mutex_);

	// lock.lock();

	if(!tf_.waitForTransform(
			scan_in->header.frame_id,
			"/map",
			scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
			ros::Duration(1.0))){
		return;
	}

	sensor_msgs::PointCloud cloud;
	projector_.transformLaserScanToPointCloud("/map",*scan_in,
			cloud,tf_);

	point_cloud_publisher_.publish(cloud);

	if(q_laser_scan.size()<queue_size_laser_scan)
	{
		q_laser_scan.push(cloud);
		return;
	}
	else
	{
		q_laser_scan.pop();
		q_laser_scan.push(cloud);
		//return FindAverage(q);
	}
}


double my_planner::AverageFilter(queue<double> &q, int window_size, double curr_value)
{
  if(q.size()<window_size)
  {
	  q.push(curr_value);
	  return curr_value;
  }
  else
  {
	  q.pop();
	  q.push(curr_value);
	  return FindAverage(q);
  }
}

double my_planner::FindAverage(queue<double> q)
{
	double sum = 0;
	int N = q.size();
	while(!q.empty())
	{
		sum = sum + q.front();
		q.pop();
	}
	return sum/N;
}



////////////////////////////////////////////////////////////////////////
void my_planner::find_points_dis_parrallel(double A1, double B1,double C1,double x0,double y0,double L,
						   double &x1, double &x2, double &y1, double &y2)
{ // find the two points whose distance to point(x0,y0) are L and who are on the line that is parrallel to the line represented by (A1*x + B1*y+C1=0)
double A = A1;
double B = B1;
double C = -A1*x0-B1*y0;
double A_A = A*A;
double B_B = B*B;
double square_A_B = A_A + B_B;
double re_sqrt =  sqrt((1/(square_A_B)));
x1 = (A*x0 + B*y0 - B*(A*L*re_sqrt + (A_A*y0)/(square_A_B) + (B_B*y0)/(square_A_B)))/A;
x2 =  (A*x0 + B*y0 - B*((A_A*y0)/(square_A_B) - A*L*re_sqrt + (B_B*y0)/(square_A_B)))/A;
y1 =  A*L*re_sqrt + (A_A*y0)/(square_A_B) + (B_B*y0)/(square_A_B);
y2 =  (A_A*y0)/(square_A_B) - A*L*re_sqrt + (B_B*y0)/(square_A_B);
if(A==0)
   {
   x1 = x0-L;
   x2 = x0+L;
   y1 = -C/B;
   y2 = -C/B;
   }
}

////////////////////////////////////////////////////////////////////////
void my_planner::find_points_dis_perpen(double A1, double B1,double C1,double x0,double y0,double L,
							double &x1, double &x2, double &y1, double &y2)
// find the two points whose distance to point(x0,y0) are L and who are on the line that is penpendicular to the line represented by (A1*x + B1*y+C1=0)
{
double A = B1;
double B = -A1;
double C = A1*y0-B1*x0;
double A_A = A*A;
double B_B = B*B;
double square_A_B = A_A + B_B;
double re_sqrt =  sqrt((1/(square_A_B)));
x1 = (A*x0 + B*y0 - B*(A*L*re_sqrt + (A_A*y0)/(square_A_B) + (B_B*y0)/(square_A_B)))/A;
x2 =  (A*x0 + B*y0 - B*((A_A*y0)/(square_A_B) - A*L*re_sqrt + (B_B*y0)/(square_A_B)))/A;
y1 =  A*L*re_sqrt + (A_A*y0)/(square_A_B) + (B_B*y0)/(square_A_B);
y2 =  (A_A*y0)/(square_A_B) - A*L*re_sqrt + (B_B*y0)/(square_A_B);
if(A==0)
   {
   x1 = x0-L;
   x2 = x0+L;
   y1 = -C/B;
   y2 = -C/B;
   }
}


////////////////////////////////////////////////////////////////////////
void my_planner::getParaLineEqu(double Point_x, double Point_y,double A, double B, double C, double &A_para, double &B_para, double &C_para)
{
	A_para = A; B_para = B;
	C_para = -A*Point_x - B*Point_y;
}

void my_planner::get_LineEqu(double PrevP_x, double PrevP_y, double NextP_x, double NextP_y,
							double &A, double &B, double &C)
{
	A = NextP_y - PrevP_y;
	B = PrevP_x - NextP_x;
	C = NextP_x*PrevP_y - PrevP_x*NextP_y;
}

double my_planner::getDist_P2L(double currPoint_x, double currPoint_y, double A, double B, double C)
{
	double dist = 0;
	if(A == 0 && B == 0) { A = B = 0.0000001; };
	dist = 	abs(A*currPoint_x + B*currPoint_y + C)/(sqrt(A*A + B*B));
	return dist;
}

double my_planner::cal_diff_rad(double start_rad, double target_rad )
{
	double LCF_diff_angle = target_rad - start_rad;
	while(LCF_diff_angle > PI)  {LCF_diff_angle -= 2*PI;}
	while(LCF_diff_angle < -PI) {LCF_diff_angle += 2*PI;}
	return LCF_diff_angle;
}

////////////////////////////////////////////////////////////////////////
void my_planner::find_vertical_points(double A0, double B0, double C0, double x0, double y0,
							double &vert_x, double &vert_y)
{
	double D0 = y0*A0 - x0*B0;
	vert_x = (-A0*C0-B0*D0)/(A0*A0+B0*B0);
	vert_y = (A0*D0-B0*C0)/(A0*A0+B0*B0);
}



obstacleState my_planner::DetectObstacle(LaserPs &laser_ps, waypoint curr_point, waypoint next_point, double tol_obst_Forw, double tol_obst_lat)
{
	obstacleState Obst_State;

	double A, B, C;
	//double A_p2n, B_p2n, C_p2n;
	double rob_size, rob_leng, rob_wid;
	rob_size = rob_leng = rob_wid = 0.40;

	LaserPs pts_Obst;
	LaserPs pts_Obst_InLat;

	LaserPs pts_Obst_Left;
	LaserPs pts_Obst_Right;
	LaserPs pts_Gap_Left;
	LaserPs pts_Gap_Right;

	double dist_Cp2Np = sqrt((next_point.y-curr_point.y)*(next_point.y-curr_point.y) + (next_point.x-curr_point.x)*(next_point.x-curr_point.x));

	A = next_point.y - curr_point.y; B = curr_point.x - next_point.x; C = next_point.x*curr_point.y - curr_point.x*next_point.y;  //X2*Y1 - X1*Y2
	if(A == 0 && B == 0) {A = 0.000001;  B = 0.000001; }

	vector<struct point_2d>::iterator it;
	for(it=laser_ps.begin(); it!=laser_ps.end(); it++)
	{
		double laser2C_dist =  sqrt(((*it).y - curr_point.y)*((*it).y - curr_point.y) + ((*it).x - curr_point.x)*((*it).x - curr_point.x));
		double laserP_lat_dist = abs(A*(*it).x + B*(*it).y + C)/(sqrt(A*A + B*B));
		double curr2Forw_dist = sqrt(laser2C_dist*laser2C_dist - laserP_lat_dist*laserP_lat_dist);
		(*it).c2laser_dist = laser2C_dist;
		(*it).lat_dist = laserP_lat_dist;
		(*it).forw_dist = curr2Forw_dist;

		if(curr2Forw_dist <= tol_obst_Forw)
		{
			double c2Forw_angle = atan2(next_point.y - curr_point.y, next_point.x - curr_point.x);
			double c2LasP_angle = atan2((*it).y - curr_point.y, (*it).x - curr_point.x);
			double LCF_diff_angle = cal_diff_rad(c2Forw_angle, c2LasP_angle );   //c2LasP_angle - c2Forw_angle;
			if(abs(LCF_diff_angle) < PI/2)  //0.92rad = max(arctan(0.15/0.20), arctan(0.2/0.15))
			{
				(*it).diff_angle = LCF_diff_angle;
				pts_Obst.push_back(*it);
			}
		}
	}

	if(pts_Obst.size()>0)
	{
		for(int i=0; i<pts_Obst.size(); i++)
		{
			if(pts_Obst[i].lat_dist <= tol_obst_lat)
			{
				pts_Obst_InLat.push_back(pts_Obst[i]);
			}
		}

		if( pts_Obst_InLat.size() >= 2 )  //2: Default value,  meet obastacles
		{
			sort(pts_Obst_InLat.begin(), pts_Obst_InLat.end(), less_ForwDist);

			if(dist_Cp2Np < pts_Obst_InLat[0].c2laser_dist - rob_size/2)
			{
				Obst_State.obst_exist_C2N = false;
			}
			else
			{
				Obst_State.obst_exist_C2N = true;
				Obst_State.ForwLasP_Nearest = pts_Obst_InLat[0];
				Obst_State.ForwLasP_Farthest = pts_Obst_InLat[pts_Obst_InLat.size()-1];

				sort(pts_Obst.begin(), pts_Obst.end(), less_angle);   //
				for(it=pts_Obst.begin(); it!=pts_Obst.end(); it++)
				{
					if((*it).diff_angle >= 0)	{ pts_Obst_Left.push_back(*it);	}
					else { pts_Obst_Right.push_back(*it); }
				}

				sort(pts_Obst_Left.begin(), pts_Obst_Left.end(), less_LatDist);
				sort(pts_Obst_Right.begin(), pts_Obst_Right.end(), less_LatDist);

				waypoint Temp_wpt_1;
				waypoint Temp_wpt_2;

				double TempDist_1 = 0;
				double TempDist_2 = 0;

				if(pts_Obst_Left.size() >= 2 && pts_Obst_Right.size() < 2)
				{
					find_points_dis_perpen(A, B, C, pts_Obst_Left[0].x, pts_Obst_Left[0].y, rob_size/2+0.15,
											Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
					TempDist_1 = getDist_P2L(Temp_wpt_1.x, Temp_wpt_1.y, A, B, C);
					TempDist_2 = getDist_P2L(Temp_wpt_2.x, Temp_wpt_2.y, A, B, C);
					if(TempDist_1 <= TempDist_2)
					{ Obst_State.feasible_Wpt.x = Temp_wpt_1.x; Obst_State.feasible_Wpt.y = Temp_wpt_1.y; }
					else
					{ Obst_State.feasible_Wpt.x = Temp_wpt_2.x; Obst_State.feasible_Wpt.y = Temp_wpt_2.y; }
				}
				else if(pts_Obst_Left.size() < 2 && pts_Obst_Right.size() >=2)
				{
					find_points_dis_perpen(A, B, C, pts_Obst_Right[0].x, pts_Obst_Right[0].y, rob_size/2+0.15,
											Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
					TempDist_1 = getDist_P2L(Temp_wpt_1.x, Temp_wpt_1.y, A, B, C);
					TempDist_2 = getDist_P2L(Temp_wpt_2.x, Temp_wpt_2.y, A, B, C);
					if(TempDist_1 <= TempDist_2)
					{ Obst_State.feasible_Wpt.x = Temp_wpt_1.x; Obst_State.feasible_Wpt.y = Temp_wpt_1.y; }
					else
					{ Obst_State.feasible_Wpt.x = Temp_wpt_2.x; Obst_State.feasible_Wpt.y = Temp_wpt_2.y; }
				}
				else
				{
					double min_LatDist_Left = pts_Obst_Left[0].lat_dist;
					double max_LatDist_Left = pts_Obst_Left[pts_Obst_Left.size()-1].lat_dist;
					double min_LatDist_Right = pts_Obst_Right[0].lat_dist;
					double max_LatDist_Right = pts_Obst_Right[pts_Obst_Right.size()-1].lat_dist;

					waypoint Temp_wpt_1;
					waypoint Temp_wpt_2;

					if( min_LatDist_Left + min_LatDist_Right > rob_size+0.15)
					{
						if(min_LatDist_Left-min_LatDist_Right <=0)
						{
							find_points_dis_perpen(A, B, C, pts_Obst_Left[0].x, pts_Obst_Left[0].y, rob_size/2+0.15,
												Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
						}
						else
						{
							find_points_dis_perpen(A, B, C, pts_Obst_Right[0].x, pts_Obst_Right[0].y, rob_size/2+0.15,
												Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
						}

						TempDist_1 = getDist_P2L(Temp_wpt_1.x, Temp_wpt_1.y, A, B, C);
						TempDist_2 = getDist_P2L(Temp_wpt_2.x, Temp_wpt_2.y, A, B, C);
						if(TempDist_1 <= TempDist_2)
						{ Obst_State.feasible_Wpt.x = Temp_wpt_1.x; Obst_State.feasible_Wpt.y = Temp_wpt_1.y; }
						else
						{ Obst_State.feasible_Wpt.x = Temp_wpt_2.x; Obst_State.feasible_Wpt.y = Temp_wpt_2.y; }
					}
					else
					{
						int i;
						for(i=0; i<=pts_Obst_Left.size()-1; i++)
						{
							double neighb_Dist_Left = pts_Obst_Left[i+1].lat_dist - pts_Obst_Left[i].lat_dist;
							if(abs(neighb_Dist_Left) > rob_size)
							{
								pts_Gap_Left.push_back(pts_Obst_Left[i]);
								pts_Gap_Left.push_back(pts_Obst_Left[i+1]);
							}

							if(pts_Gap_Left.size() != 0) {	break; }
						}

						for(i=0; i<=pts_Obst_Right.size()-1; i++)
						{
							double neighb_Dist_Right = pts_Obst_Right[i+1].lat_dist - pts_Obst_Right[i].lat_dist;
							if(abs(neighb_Dist_Right) > rob_size)
							{
								pts_Gap_Right.push_back(pts_Obst_Right[i]);
								pts_Gap_Right.push_back(pts_Obst_Right[i+1]);
							}

							if(pts_Gap_Right.size() != 0) {	break; }
						}

						if(pts_Gap_Left.size() != 0 && pts_Gap_Right.size() != 0)
						{
							if(pts_Gap_Left[0].lat_dist <= pts_Gap_Right[0].lat_dist)
							{
								find_points_dis_perpen(A, B, C, pts_Gap_Left[0].x, pts_Gap_Left[0].y, rob_size/2+0.15,
												Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
							}
							else
							{
								find_points_dis_perpen(A, B, C, pts_Gap_Right[0].x, pts_Gap_Right[0].y, rob_size/2+0.15,
												Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
							}
						}
						else if(pts_Gap_Left.size() == 0 && pts_Gap_Right.size() == 0)
						{
							if(max_LatDist_Left <= max_LatDist_Right)
							{
								find_points_dis_perpen(A, B, C, pts_Obst_Left[pts_Obst_Left.size()-1].x, pts_Obst_Left[pts_Obst_Left.size()-1].y, rob_size/2+0.15,
												Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
							}
							else
							{
								find_points_dis_perpen(A, B, C, pts_Obst_Right[pts_Obst_Right.size()-1].x, pts_Obst_Right[pts_Obst_Right.size()-1].y, rob_size/2+0.15,
												Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
							}
						}
						else if(pts_Gap_Left.size() != 0 && pts_Gap_Right.size() == 0)
						{
							if(pts_Gap_Left[0].lat_dist <= pts_Obst_Right[pts_Obst_Right.size()-1].lat_dist)
							{
								find_points_dis_perpen(A, B, C, pts_Gap_Left[0].x, pts_Gap_Left[0].y, rob_size/2+0.15,
												Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
							}
							else
							{
								find_points_dis_perpen(A, B, C, pts_Obst_Right[pts_Obst_Right.size()-1].x, pts_Obst_Right[pts_Obst_Right.size()-1].y, rob_size/2+0.15,
												Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
							}

						}
						else if(pts_Gap_Left.size() == 0 && pts_Gap_Right.size() != 0)
						{
							if(pts_Obst_Left[pts_Obst_Left.size()-1].lat_dist <= pts_Gap_Right[0].lat_dist)
							{
								find_points_dis_perpen(A, B, C, pts_Obst_Left[pts_Obst_Left.size()-1].x, pts_Obst_Left[pts_Obst_Left.size()-1].y, rob_size/2+0.15,
												Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
							}
							else
							{
								find_points_dis_perpen(A, B, C, pts_Gap_Right[0].x, pts_Gap_Right[0].y, rob_size/2+0.15,
												Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
							}
						}

						TempDist_1 = getDist_P2L(Temp_wpt_1.x, Temp_wpt_1.y, A, B, C);
						TempDist_2 = getDist_P2L(Temp_wpt_2.x, Temp_wpt_2.y, A, B, C);
						if(TempDist_1 >= TempDist_2)
						{ Obst_State.feasible_Wpt.x = Temp_wpt_1.x; Obst_State.feasible_Wpt.y = Temp_wpt_1.y; }
						else
						{ Obst_State.feasible_Wpt.x = Temp_wpt_2.x; Obst_State.feasible_Wpt.y = Temp_wpt_2.y; }
					}
				}
			}
		}
		else
		{
			Obst_State.obst_exist_C2N = false;
		}

	}
	else
	{
		Obst_State.obst_exist_C2N = false;
	}

	return  Obst_State;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
Avoid_Obst_Wpts my_planner::FindWptToAvoidObst( obstacleState Obst_State, double c_x, double c_y, double c_th, waypoint prev_Wpt, waypoint next_Wpt)
{
	double A_p2n, B_p2n, C_p2n;
	double A_para, B_para, C_para;
	double vert_x, vert_y;
	double rob_size = 0.4;

	Avoid_Obst_Wpts valid_Wpts;
	waypoint Temp_wpt_1, Temp_wpt_2;

	get_LineEqu(prev_Wpt.x, prev_Wpt.y, next_Wpt.x, next_Wpt.y, A_p2n, B_p2n, C_p2n);

	if(Obst_State.obst_exist_C2N == false)	{valid_Wpts.valid_Wpts = false ; return valid_Wpts;}

	valid_Wpts.valid_Wpts = true;

	waypoint curr_Wpt, temp_Wpt;
	curr_Wpt.x = c_x; curr_Wpt.y = c_y; curr_Wpt.theta = c_th;
	find_vertical_points(A_p2n, B_p2n, C_p2n, c_x, c_y, vert_x, vert_y);
	temp_Wpt.x = vert_x; temp_Wpt.y = vert_y; temp_Wpt.theta = 0;
	valid_Wpts.pts_local.push_back(temp_Wpt);

	getParaLineEqu(Obst_State.feasible_Wpt.x, Obst_State.feasible_Wpt.y, A_p2n, B_p2n, C_p2n, A_para, B_para, C_para);
	find_vertical_points(A_para, B_para, C_para, c_x, c_y, vert_x, vert_y);
	temp_Wpt.x = vert_x; temp_Wpt.y = vert_y; temp_Wpt.theta = 0;
	valid_Wpts.pts_local.push_back(temp_Wpt);

	valid_Wpts.pts_local.push_back(Obst_State.feasible_Wpt);

	find_points_dis_parrallel(A_p2n, B_p2n, C_p2n, Obst_State.feasible_Wpt.x, Obst_State.feasible_Wpt.y, rob_size+0.1,
								Temp_wpt_1.x, Temp_wpt_2.x, Temp_wpt_1.y, Temp_wpt_2.y);
	double dist_c2No1 = sqrt((Temp_wpt_1.x-curr_Wpt.x)*(Temp_wpt_1.x-curr_Wpt.x)
											+(Temp_wpt_1.y-curr_Wpt.y)*(Temp_wpt_1.y-curr_Wpt.y));
	double dist_c2No2 = sqrt((Temp_wpt_2.x-curr_Wpt.x)*(Temp_wpt_2.x-curr_Wpt.x)
											+(Temp_wpt_2.y-curr_Wpt.y)*(Temp_wpt_2.y-curr_Wpt.y));
	if(dist_c2No1 <= dist_c2No2)
	{ valid_Wpts.pts_local.push_back(Temp_wpt_2);	}
	else
	{ valid_Wpts.pts_local.push_back(Temp_wpt_1);	}

	find_vertical_points(A_p2n, B_p2n, C_p2n, valid_Wpts.pts_local[valid_Wpts.pts_local.size()-1].x,
												valid_Wpts.pts_local[valid_Wpts.pts_local.size()-1].y, vert_x, vert_y);
	temp_Wpt.x = vert_x; temp_Wpt.y = vert_y; temp_Wpt.theta = 0;
	valid_Wpts.pts_local.push_back(temp_Wpt);

	valid_Wpts.pts_local.push_back(next_Wpt);

	return  valid_Wpts;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool my_planner::Planner_Local(double c_x, double c_y, double c_th, Waypoints &pts_Local, int &nextWpt_Local, double &vx, double &vy, double &w)
{
	waypoint  past_Wp, next_Wp;

	double A, B, C;
	double lat_dist = 0.0;
	double p2c_slope = 0.0;
	double p2n_slope = 0.0;
	double c2n_slope = 0.0;

	double dirLOS_diff = 0.0;
	double c2p2n_diff = 0.0;

	if(nextWpt_Local < pts_Local.size()-1)
	{
		double dist_CurrPt2Waypt = sqrt((pts_Local[nextWpt_Local].y-c_y)*(pts_Local[nextWpt_Local].y-c_y)+
										(pts_Local[nextWpt_Local].x-c_x)*(pts_Local[nextWpt_Local].x-c_x));

		p2n_slope = atan2(pts_Local[nextWpt_Local].y-pts_Local[nextWpt_Local-1].y, pts_Local[nextWpt_Local].x-pts_Local[nextWpt_Local-1].x);
		p2c_slope = atan2(c_y-pts_Local[nextWpt_Local-1].y, c_x-pts_Local[nextWpt_Local-1].x);
		c2n_slope = atan2(pts_Local[nextWpt_Local].y-c_y, (pts_Local[nextWpt_Local].x-c_x));

		c2p2n_diff = cal_diff_rad(p2c_slope, p2n_slope );		//p2n_slope - p2c_slope;
		dirLOS_diff = cal_diff_rad(c_th, c2n_slope );			//c2n_slope - c_th;

		past_Wp = pts_Local[nextWpt_Local-1];
		next_Wp = pts_Local[nextWpt_Local];
		A = next_Wp.y - past_Wp.y; B = past_Wp.x - next_Wp.x; C = next_Wp.x*past_Wp.y - past_Wp.x*next_Wp.y;  //X2*Y1 - X1*Y2
		lat_dist = getDist_P2L(c_x, c_y, A, B, C);

		if(dist_CurrPt2Waypt > DIST_TOLE)			//without
		{
				if( abs(dirLOS_diff) <= PI/3 )
				{
					if( abs(dirLOS_diff) >= DIR_TOLE_LOCAL)     //Æ«²îŽóÓÚ
					{
						vx = 0.0;
						vy = 0.0;
						if(dirLOS_diff > 0)	{w = BIG_REGU;}  //BIG_REGU = 0.5
						else {w = -BIG_REGU;}
					}
					else
					{
						vx = NORM_SPEED_LOCAL;
						if(lat_dist > LAT_DIST_TOL)
						{
							if(c2p2n_diff > 0) {vy = MIN_LAT_SPEED;}
							else {vy = -MIN_LAT_SPEED;}
						}
						else {vy = 0;}

						if(abs(dirLOS_diff) > DIR_TOLE_LOCAL/2)			// DIR_TOLE/3
						{
							if(dirLOS_diff > 0) {w = SMALL_REGU;}  //SMALL_REGU
							else {w = -SMALL_REGU;}
						}
						else {w =0;}
					}
				}
				else if( abs(dirLOS_diff) >= 2*PI/3 )
				{
					double diff_Posi180 = c2n_slope + PI - c_th;
					while(diff_Posi180 > PI)  {diff_Posi180 -= 2*PI;}
					while(diff_Posi180 < -PI) {diff_Posi180 += 2*PI;}

					if( abs(diff_Posi180) >= DIR_TOLE_LOCAL)     //Æ«²îŽóÓÚ
					{
						vx = 0.0;
						vy = 0.0;
						if(diff_Posi180 > 0)	{w = BIG_REGU;}
						else {w = -BIG_REGU;}
					}
					else
					{
						vx = -NORM_SPEED_LOCAL;
						if(lat_dist > LAT_DIST_TOL)
						{
							if(c2p2n_diff > 0) {vy = -MIN_LAT_SPEED;}
							else {vy = MIN_LAT_SPEED;}
						}
						else {vy = 0;}

						if(abs(diff_Posi180) > DIR_TOLE_LOCAL/2)			//
						{
							if(diff_Posi180 > 0) {w = SMALL_REGU;}
							else {w = -SMALL_REGU;}
						}
						else {w =0;}
					}
				}
				else
				{
					if( dirLOS_diff <= 0 )
					{
						double diff_Posi90 = c2n_slope + PI/2 - c_th;
						while(diff_Posi90 > PI)  {diff_Posi90 -= 2*PI;}
						while(diff_Posi90 < -PI) {diff_Posi90 += 2*PI;}

						if( abs(diff_Posi90) >= DIR_TOLE_LOCAL)     //Æ«²îŽóÓÚ
						{
							vx = 0.0;
							vy = -NORM_SPEED_LOCAL;
							if(diff_Posi90 > 0)	{w = MED_REGU;}  //MED_REGU = 0.2
							else {w = -MED_REGU;}
						}
						else
						{
							vx = 0;
							vy = -NORM_SPEED_LOCAL;
							/*if(abs(diff_Posi90) > DIR_TOLE_LOCAL/2)			//
							{
								if(diff_Posi90 > 0) {w = SMALL_REGU;}
								else {w = -SMALL_REGU;}
							}
							else {w =0;}*/
							w =0;
						}
					}
					else
					{
						double diff_Anti90 = c2n_slope - PI/2 - c_th;
						while(diff_Anti90 > PI)  {diff_Anti90 -= 2*PI;}
						while(diff_Anti90 < -PI) {diff_Anti90 += 2*PI;}

						if( abs(diff_Anti90) >= DIR_TOLE_LOCAL)     //Æ«²îŽóÓÚ
						{
							vx = 0.0;
							vy = NORM_SPEED_LOCAL;
							if(diff_Anti90 > 0)	{w = MED_REGU;}
							else {w = -MED_REGU;}
						}
						else
						{
							vx = 0;
							vy = NORM_SPEED_LOCAL;
							/*if(abs(diff_Anti90) > DIR_TOLE_LOCAL/2)			//
							{
								if(diff_Anti90 > 0) {w = SMALL_REGU;}
								else {w = -SMALL_REGU;}
							}
							else {w =0;}*/
							w =0;
						}
					}
				}
		}
		else
		{
			nextWpt_Local++;
		}
	}
	else
	{
		vx = 0;
		vy = 0;
		w = 0;

		//double dir_diff_final = pts[pts.size()-1].theta - c_th;
		//while(dir_diff_final > PI) {dir_diff_final -= 2*PI;}
		//while(dir_diff_final < -PI) {dir_diff_final += 2*PI;}

		//if( abs(dir_diff_final) >= 0.1)
		//{
		//	if(dir_diff_final > 0)	{w = 0.6;}
		//	else {w = -0.6;}
		//}
		//else {w = 0;}
	}

	return true;
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
