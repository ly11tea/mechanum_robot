#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
#include "tf/tf.h"
#include "stdio.h"

using namespace std;

double start_x;
double start_y;
double start_th;
double goal_x;
double goal_y;
double goal_th;
string file_name;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "obtain_plan_client");

  ros::NodeHandle n;
  ros::NodeHandle n_local("~");
  n_local.param("start_x",start_x, 0.340);
  n_local.param("start_y",start_y, 0.223);
  n_local.param("start_th",start_th, 0.019);
  n_local.param("goal_x",goal_x, 2.310);
  n_local.param("goal_y",goal_y, 7.991);
  n_local.param("goal_th",goal_th, 1.606);
  n_local.param("file_name",file_name, std::string("~/obtained_plan.txt"));
  
  
	 FILE *file = fopen(file_name.c_str(), //打开文件的名称
	                     "w"); // 文件打开方式 如果原来有内容也会销毁
	 if(!file)
	 {
	    printf("openning file failed!\n");
	    exit(-1);
	    return -1;
	 }

  ros::ServiceClient client = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  nav_msgs::GetPlan srv;
  srv.request.start.header.frame_id = "map";
  srv.request.start.header.stamp = ros::Time::now();
  srv.request.start.pose.position.x = start_x;
  srv.request.start.pose.position.y = start_y;
  srv.request.start.pose.position.z = 0.0;
  srv.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(start_th);

  srv.request.goal.header.frame_id = "map";
  srv.request.goal.header.stamp = ros::Time::now();
  srv.request.goal.pose.position.x = goal_x;
  srv.request.goal.pose.position.y = goal_y;
  srv.request.goal.pose.position.z = 0.0;
  srv.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_th);
  if (client.call(srv))
  {
    //ROS_INFO("Sum: %ld", (long int)srv.response.plan.poses[1]);
      printf("The length of the returned plan is %d\n",srv.response.plan.poses.size());
	  for(int i=0;i<srv.response.plan.poses.size();i++)
	  {
		  double tmp_x = srv.response.plan.poses[i].pose.position.x;
		  double tmp_y = srv.response.plan.poses[i].pose.position.y;
		  double tmp_th = tf::getYaw(srv.response.plan.poses[i].pose.orientation);
		  printf("The %d th pose in the plan is:%f,%f,%f\n",i, tmp_x, tmp_y, tmp_th);
		  fprintf(file,"%.2f,%.2f,\n",i, tmp_x, tmp_y);
	  }
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
  fclose(file);
   return 0;
}
