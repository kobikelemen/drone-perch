


#include <gnc_functions.hpp>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>


using namespace std;
vector<float> trajectory = {0.0, -0.232, -0.496, -0.744, -0.928, -1.0, -0.912, -0.616, -0.064, 0.792};
float qw, qx, qy, qz;


double r;
double theta;
double counter = 0;
double wn;


ros::Publisher local_attitude_pub;
ros::Publisher thrust_pub;
ros::Publisher trajectory_pub;

float distance(geometry_msgs::Point vec)
{
	return sqrt(pow(vec.x,2) + pow(vec.y,2) + pow(vec.z,2));
}

geometry_msgs::Quaternion euler2quat(vector<float> euler)
{	

	// MAKE SURE EULER ANGLES ARE RADIANS <<< !!!!!!!

	geometry_msgs::Quaternion q;
	q.w = cos(euler[1]/2)*cos(euler[0]/2)*cos(euler[2]/2) - sin(euler[1]/2)*sin(euler[0]/2)*sin(euler[2]/2);
	q.x = sin(euler[1]/2)*sin(euler[0]/2)*cos(euler[2]/2) + cos(euler[1]/2)*cos(euler[0]/2)*sin(euler[2]/2);
	q.y = sin(euler[1]/2)*cos(euler[0]/2)*cos(euler[2]/2) + cos(euler[1]/2)*sin(euler[0]/2)*sin(euler[2]/2);
	q.z = cos(euler[1]/2)*sin(euler[0]/2)*cos(euler[2]/2) - sin(euler[1]/2)*cos(euler[0]/2)*sin(euler[2]/2);
	return q;
}


// void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
// {
//   current_pose_g = *msg;
//   qw = current_pose_g.pose.pose.orientation.w;
//   qx = current_pose_g.pose.pose.orientation.x;
//   qy = current_pose_g.pose.pose.orientation.y;
//   qz = current_pose_g.pose.pose.orientation.z;
// }


geometry_msgs::Quaternion get_required_angles(geometry_msgs::Point diff)
{	

	// Idea of this is angle of desired point needed, not just of drone

	float magnitude = sqrt(pow(diff.x, 2) + pow(diff.y, 2) + pow(diff.z, 2));
	float x_angle = acos(diff.x / magnitude);
	float y_angle = acos(diff.y / magnitude);
	float z_angle = acos(diff.z / magnitude);

	vector<float> euler = {x_angle, y_angle, z_angle};
	geometry_msgs::Quaternion q = euler2quat(euler);

	return q;

}



int main(int argc, char** argv)
{
	std::string ros_namespace;
	ros::init(argc, argv, "test_trajectory");
	ros::NodeHandle n;
	init_publisher_subscriber(n);

	
	ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>((ros_namespace+"mavros/setpoint_position/local").c_str(), 10);
	

	n.param("pub_setpoints_traj/wn", wn, 3.0);
	n.param("pub_setpoints_traj/r", r, 4.0);



	//currentPos = n.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);
	
	wait4connect();
	wait4start();
	takeoff(2);

	ros::Rate rate(2.0);

	//geometry_msgs::PoseStamped pose;
	// pose.pose.position.x = 0;
	// pose.pose.position.y = 0;
	// pose.pose.position.z = 2;

	// local_pos_pub.publish(pose);

	while(ros::ok())
	{	
		set_speed(9);
		geometry_msgs::PoseStamped pose;
		theta = wn * counter * 0.05;

		pose.pose.position.x = r*sin(theta);
		pose.pose.position.y = r*cos(theta);
		pose.pose.position.z = 2;

		counter++;

		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();

	}

	return 0;	
}








// trajectory_pub = n.advertise<mavros_msgs::Trajectory>((ros_namespace+"/mavros/trajectory/generated").c_str(), 10);
// 	mavros_msgs::PositionTarget points;

// 	points.point_1.x = 0;
// 	points.point_1.y = 0.2;
// 	points.point_1.z = trajectory[0];

// 	points.point_2.x = 0;
// 	points.point_2.y = 0.4;
// 	points.point_2.z = trajectory[1];

// 	points.point_3.x = 0;
// 	points.point_3.y = 0.6;
// 	points.point_3.z = trajectory[2];

// 	points.point_4.x = 0;
// 	points.point_4.y = 0.8;
// 	points.point_4.z = trajectory[3];

// 	points.point_5.x = 0;
// 	points.point_5.y = 1;
// 	points.point_5.z = trajectory[4];

// 	trajectory_pub.publish(points);


