






#include <gnc_functions.hpp>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
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
vector<float> delta_z = {-0.232, -0.496, -0.744, -0.928, -1.0, -0.912, -0.616, -0.064, 0.792, 1};
vector<geometry_msgs::Point> trajectory;
float qw, qx, qy, qz;

ros::Publisher local_attitude_pub;
ros::Publisher thrust_pub;

float distance(geometry_msgs::Twist vec)
{
	return sqrt(pow(vec.linear.x,2) + pow(vec.linear.y,2) + pow(vec.linear.z,2));
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
	//local_attitude_pub = n.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/setpoint_attitude/attitude").c_str(), 10);
	//thrust_pub = n.advertise<mavros_msgs::Thrust>((ros_namespace + "/mavros/setpoint_attitude/thrust").c_str(), 10);
	ros::Publisher glob_vel_pub = n.advertise<geometry_msgs::Twist>((ros_namespace + "/mavros/setpoint_velocity/cmd_vel_unstamped").c_str(), 10);

	//currentPos = n.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);
	
	wait4connect();
	wait4start();
	

	initialize_local_frame();
	set_speed(2);
	takeoff(2);
	ros::Rate rate(10.0);

	float y_ = 0.2;

	geometry_msgs::Point current_location = get_current_location();

	while (!(current_location.z>1.9 && current_location.z<2.1) )
	{
		rate.sleep();
		ros::spinOnce();
		current_location = get_current_location();
	}


	for(int i=0; i<delta_z.size(); i++)
	{

		geometry_msgs::Point p;
		p.x = current_location.x;
		p.y = current_location.y + 0.2*(i+1);
		p.z = current_location.z + delta_z[i];

		trajectory.push_back(p);
	}

	int counter = 0;


	current_location = get_current_location();
	geometry_msgs::Twist diff;
	while(ros::ok())
	{	
		
		ros::spinOnce();
		current_location = get_current_location();
		
		//if (counter == 0)
		//{
		diff.linear.x = (trajectory[counter].x - current_location.x) * 1;
		diff.linear.y = (trajectory[counter].y - current_location.y) * 1;
		diff.linear.z = (trajectory[counter].z - current_location.z) * 1;
		glob_vel_pub.publish(diff);
		//}
		//current_location = get_current_location();
		//geometry_msgs::Quaternion required_q_angles = get_required_angles(diff);

		//local_attitude_pub.publish(required_q_angles);


		if (distance(diff) < 0.1 || counter == 0)
		{	
			//cout << " in <0.05" << endl;

			if (counter < trajectory.size())
			{	
				//cout << " in counter < waypointlist.size() " << endl;
				


				counter++;
				ros::spinOnce();
				current_location = get_current_location();
				// diff.linear.x = (trajectory[counter].x - current_location.x) * 1;
				// diff.linear.y = (trajectory[counter].y - current_location.y) * 1;
				// diff.linear.z = (trajectory[counter].z - current_location.z) * 1;
				// glob_vel_pub.publish(diff);

				cout << " current trajectory (x,y,z): " << trajectory[counter].x << " " << trajectory[counter].y << " " << trajectory[counter].z << endl;
		
				cout << " current location (x,y,z): " << current_location.x << " " << current_location.y << " " << current_location.z << endl;
				cout << " diff (x,y,z) " << diff.linear.x << " " << diff.linear.y << " " << diff.linear.z << " " << endl;

			}			
		}


		ros::spinOnce();
		
		current_location = get_current_location();
		if (current_location.z > (trajectory.back().z-0.1))
		{
			cout << " trajectory[-1]-0.1: " << trajectory.back().z-0.1 << endl;
			geometry_msgs::Twist vel;
			vel.linear.x = 0;
			vel.linear.y = 0;
			vel.linear.z = 0;
			glob_vel_pub.publish(vel);
			break;
		}

		rate.sleep();

	}

	return 0;	
}


