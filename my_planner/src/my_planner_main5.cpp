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

#include <queue>

#define PI 3.141592
#define INFINITY 9999.0
#define DIST_TOLE 0.15  //0.15m
#define DIR_TOLE 0.2   //0.2rad
#define NORM_SPEED 0.2   //0.2 m/s
#define LAT_DIST_TOL 0.05  //0.05m
#define MIN_LAT_SPEED 0.04
#define MAX_LAT_SPEED 0.05

using namespace std;

struct waypoint
{
	double x;
	double y;
	double theta;
};

//typedef std::vector<struct waypoint> Waypoints;


struct point_2d
{
    double x;
    double y;
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
    ros::Subscriber scan_sub_;
    Obst_Mode_Wpts wpt_mode;
    double c_x,c_y,c_th;
    bool get_pose_successful;
    int nextWpt;
    laser_geometry::LaserProjection projector_;
    queue<double> q_motion_mode;

    double avg_motion_mode ;
    //ros::Time currnt_time;

    boost::thread* ObsAvoid_thread_;

    //queue of laser scan
    queue<sensor_msgs::PointCloud> q_laser_scan;
    int queue_size_laser_scan;
    boost::mutex obstacle_avoidance_mutex_;

    double tol_obst_Forw_;
    double tol_obst_lat_;





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
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
    int FindTheClosestWayPoints(double x, double y, Waypoints W_pts); //find the index of the closest way points to the current x,y
    void redefine_waypoints(double c_x, double c_y, double c_th,
    		double g_x, double g_y, double g_th, Waypoints &W_pts);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    void ObsAvoid_Thread();//thread for avoiding obstacle.
    Obst_Mode_Wpts FindWptToAvoidObst( LaserPs &laser_ps, double c_x, double c_y, double c_th,
    							Waypoints &pts, int nextWpt, double tol_obst_Forw, double tol_obst_lat);
    double AverageFilter(queue<double> &q, int window_size, double curr_value);
    double FindAverage(queue<double> q);
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

    vel_pub_ = n_.advertise<geometry_msgs::Twist>(cmd_pub_topic.c_str(), 10);

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

    ObsAvoid_thread_ = new boost::thread(boost::bind(&my_planner::ObsAvoid_Thread, this));

}

void my_planner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{

	goal_changed = true;
	goal_ = *goal;
	ROS_INFO("goal received");
	triggered = true;

}


void my_planner::ObsAvoid_Thread()
{

	//ros::NodeHandle n;
	ros::Rate loop_rate(5);//5 herz
	while(ros::ok())
	{
		ROS_INFO("thread is running");
		int q_size = q_laser_scan.size();
		if(q_size >= 1)
		{
			sensor_msgs::PointCloud  cloud = q_laser_scan.back();
			std::vector <struct point_2d> laser_ps;
			laser_ps.clear();
			for(int i=0;i<cloud.points.size();i++)
			{
				struct point_2d tmp_point;
				tmp_point.x = cloud.points[i].x;
				tmp_point.y = cloud.points[i].y;
				laser_ps.push_back(tmp_point);
			}
			if(get_pose_successful && triggered)
			{
				ROS_INFO("laser ps size is %d",laser_ps.size());

				wpt_mode = FindWptToAvoidObst( laser_ps,  c_x,  c_y,  c_th,
						Redefine_pts,  nextWpt, tol_obst_Forw_, tol_obst_lat_);
				//  double avg_motion_mode = AverageFilter(q_motion_mode, 5, (double)(wpt_mode.motionMode));
				printf("motionMode is %d\n",wpt_mode.motionMode);
			}
		}
    	loop_rate.sleep();

	}




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
						if(dirLOS_diff > 0) {w = 0.03;}
						else {w = -0.03;}
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
						if(diff_Posi180 > 0) {w = 0.03;}
						else {w = -0.03;}
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
						if(dirLOS_diff > 0) {w = 0.03;}
						else {w = -0.03;}
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
						if(diff_Posi180 > 0) {w = 0.03;}
						else {w = -0.03;}
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
							if(diff_Posi90 > 0) {w = 0.03;}
							else {w = -0.03;}
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
							if(diff_Anti90 > 0) {w = 0.03;}
							else {w = -0.03;}
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

	if( abs(dir_diff_final) >= 0.1)
	{
		if(dir_diff_final > 0)	{w = 0.6;}
		else {w = -0.6;}
	}
	else {w = 0;}

}

return true;
}

void my_planner::control_loop()
{
  ros::Rate rate(control_frequency_);
  tf::Stamped < tf::Pose > pose;
  double vx=0,vy=0,w=0.0;
  //Waypoints W_pts;


  while(ros::ok())
  {

      //ROS_INFO("control loop is running");
	   if (getRobotPose (pose))
	   {
		    get_pose_successful = true;
	        c_x = pose.getOrigin().x(),
	        c_y = pose.getOrigin().y(),
	        c_th = tf::getYaw(pose.getRotation());


	     if(triggered && !pts.empty())
	     {
	    	 if(wpt_mode.motionMode==1)
	    	 {
	    		 cmd.linear.x = 0.0;
	    		 cmd.linear.y = 0.0;
	    		 cmd.angular.z = 0.0;
	    	 }
	    	 else
	    	 {
	    	     if(goal_changed)
	    	     {

	    	    	 Redefine_pts = pts;
	    	    	// boost::unique_lock<boost::mutex> lock(obstacle_avoidance_mutex_);
	    	    	// lock.lock();
	    	    	 redefine_waypoints(c_x, c_y, c_th,
	    	    	 		goal_.pose.position.x, goal_.pose.position.y, tf::getYaw(goal_.pose.orientation), Redefine_pts);
	    	    	// lock.unlock();
	    	    	 nextWpt = 1;
	    	    	 goal_changed = false;
	    	     }


	    	     bool plan_succ = Planner_2(c_x, c_y, c_th, Redefine_pts, vx, vy, w, nextWpt);

	    	     if( plan_succ )
	    	     {
	    	    	 cmd.linear.x = vx;
	    	    	 cmd.linear.y = vy;
	    	    	 cmd.angular.z = w;
	    	     }

	    	 }

         	 vel_pub_.publish(cmd);
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

Obst_Mode_Wpts my_planner::FindWptToAvoidObst( LaserPs &laser_ps, double c_x, double c_y, double c_th,
							Waypoints &pts, int nextWpt, double tol_obst_Forw, double tol_obst_lat)
{
	double A, B, C;
	double rob_size, rob_leng, rob_wid;
	rob_size = rob_leng = rob_wid = 0.40;

	Obst_Mode_Wpts  model_Wpts;

	Waypoints pts_AvoObst;
	LaserPs pts_Obst;
	LaserPs pts_Obst_Left;
	LaserPs pts_Obst_Right;
	LaserPs pts_Gap_Left;
	LaserPs pts_Gap_Right;

	waypoint curr_Wp, next_Wp;
	curr_Wp.x = c_x;  curr_Wp.y = c_y;  curr_Wp.theta = c_th;
	next_Wp = pts[nextWpt];
	A = next_Wp.y - curr_Wp.y; B = curr_Wp.x - next_Wp.x; C = next_Wp.x*curr_Wp.y - curr_Wp.x*next_Wp.y;  //X2*Y1 - X1*Y2

	vector<struct point_2d>::iterator it;
	for(it=laser_ps.begin(); it!=laser_ps.end(); it++)
	{
		double laser2C_dist =  sqrt(((*it).y - curr_Wp.y)*((*it).y - curr_Wp.y) + ((*it).x - curr_Wp.x)*((*it).x - curr_Wp.x));
		double tmp_sqrt_a_b = sqrt(A*A + B*B);
		//printf("tmp_sqrt_a_b is %f\n",tmp_sqrt_a_b);
		if(tmp_sqrt_a_b==0.0)
			ROS_INFO("tmp_sqrt_a_b = 0");
		double laserP_lat_dist = abs(A*(*it).x + B*(*it).y + C)/(tmp_sqrt_a_b);
		(*it).lat_dist = laserP_lat_dist;

		double curr2Forw_dist = sqrt(laser2C_dist*laser2C_dist - laserP_lat_dist*laserP_lat_dist);
		(*it).forw_dist = curr2Forw_dist;
		if(curr2Forw_dist <= tol_obst_Forw)
		{
			double c2Forw_angle = atan2(next_Wp.y - curr_Wp.y, next_Wp.x - curr_Wp.x);
			double c2LasP_angle = atan2((*it).y - curr_Wp.y, (*it).x - curr_Wp.x);
			double LCF_diff_angle = c2LasP_angle - c2Forw_angle;
			while(LCF_diff_angle > PI)  {LCF_diff_angle -= 2*PI;}
			while(LCF_diff_angle < -PI) {LCF_diff_angle += 2*PI;}
			if(abs(LCF_diff_angle) < PI/2)
			{
				(*it).diff_angle = LCF_diff_angle;
				pts_Obst.push_back(*it);
			}
		}

	}

	if(pts_Obst.size()>0)
	{
		sort(pts_Obst.begin(), pts_Obst.end(), less_LatDist);
		if(pts_Obst[0].lat_dist <= tol_obst_lat)
		{
			double c2n_dist = sqrt((next_Wp.y-curr_Wp.y)*(next_Wp.y-curr_Wp.y) + (next_Wp.x-curr_Wp.x)*(next_Wp.x-curr_Wp.x));
			if( pts_Obst[0].forw_dist - c2n_dist < 0.4)   // œøÈë±ÜÕÏÄ£Êœ
			{
				model_Wpts.motionMode = 1;
			}

			sort(pts_Obst.begin(), pts_Obst.end(), less_angle);   //

			for(it=pts_Obst.begin(); it!=pts_Obst.end(); it++)
			{
				if((*it).diff_angle >= 0)	{ pts_Obst_Left.push_back(*it);	}
				else { pts_Obst_Right.push_back(*it); }
			}

			for(int i=0; i<pts_Obst_Left.size()-1; i++)
			{
				double neighb_Dist = pts_Obst_Left[i+1].lat_dist - pts_Obst_Left[i].lat_dist;
				if(neighb_Dist > rob_size)
				{
					pts_Gap_Left.push_back(pts_Obst_Left[i]);
					pts_Gap_Left.push_back(pts_Obst_Left[i+1]);
				}
			}

			for(int j=pts_Obst_Right.size()-1; j>0; j--)
			{
				//this j-- is problematic cause j could be 0, and j-1 = -1 which is wrong;
				double neighb_Dist = pts_Obst_Right[j-1].lat_dist - pts_Obst_Right[j].lat_dist;
				if(neighb_Dist > rob_size)
				{
					pts_Gap_Right.push_back(pts_Obst_Left[j]);
					pts_Gap_Right.push_back(pts_Obst_Left[j-1]);
				}
			}
		}
		else
		{
			model_Wpts.motionMode = 0;
		}
	}

	return model_Wpts;
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
