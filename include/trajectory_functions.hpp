

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
#include <perch.h>

using namespace std;
//vector<float> delta_z = {-0.232, -0.496, -0.744, -0.928, -1.0, -0.912, -0.616, -0.064, 0.792, 1};
vector<float> delta_z = {0.035, 0.01, -0.005, 0.06, 0.2, 0.35};
float delta_y = 0.2;
vector<geometry_msgs::Point> trajectory;


ros::Publisher local_attitude_pub;
ros::Publisher thrust_pub;
ros::Publisher glob_vel_pub;

float distance(geometry_msgs::Twist vec)
{
	return sqrt(pow(vec.linear.x,2) + pow(vec.linear.y,2) + pow(vec.linear.z,2));
}

geometry_msgs::Twist get_diff(float counter, geometry_msgs::Point current_location, float velocity_multiplier)
{
	geometry_msgs::Twist diff;
	diff.linear.x = (trajectory[counter].x - current_location.x) * velocity_multiplier;
	diff.linear.y = (trajectory[counter].y - current_location.y) * velocity_multiplier;
	diff.linear.z = (trajectory[counter].z - current_location.z) * velocity_multiplier;
	return diff;
}


int perform_trajectory(ros::NodeHandle controlnode)
{
	std::string ros_namespace;
	//ros::init(argc, argv, "perform_trajectory");
	//ros::NodeHandle n;
	//init_publisher_subscriber(n);

	//local_attitude_pub = n.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/setpoint_attitude/attitude").c_str(), 10);
	//thrust_pub = n.advertise<mavros_msgs::Thrust>((ros_namespace + "/mavros/setpoint_attitude/thrust").c_str(), 10);
	cout << " in perform_trajectory" << endl;
	glob_vel_pub = controlnode.advertise<geometry_msgs::Twist>((ros_namespace + "/mavros/setpoint_velocity/cmd_vel_unstamped").c_str(), 10);

	//currentPos = n.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);

	//initialize_local_frame();
	//ros::Rate rate(25.0);
	move_start = true;
	ros::spinOnce();
	current_location = get_current_location();
	cout << " current_location before setting trajectory " << current_location << endl;
	
	for(int i=0; i<delta_z.size(); i++)
	{
		geometry_msgs::Point p;
		p.x = current_location.x;
		p.y = current_location.y + delta_y*(i+1);
		p.z = current_location.z + delta_z[i];

		trajectory.push_back(p);
	}

	int counter = 0;
	geometry_msgs::Twist diff;
	//vector<float> velocity_multiplier_list = {1, 1.3, 1.8, 0.8, 0.35, 0.2};
	vector<float> velocity_multiplier_list = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

	while(!(current_location.z > trajectory.back().z && current_location.y > trajectory.back().y))
	{	
		cout << " in while loop in perform_trajectory " << endl;
		current_location = get_current_location();
		diff = get_diff(counter, current_location, velocity_multiplier_list[counter]);

		if (counter == 0){
			glob_vel_pub.publish(diff);
			cout << " in if (counter==0) ... publish diff" << endl;
		}

		if (distance(diff) < 0.2 || counter == 0)
		{	

			if (counter < trajectory.size())
			{	
				cout << " in if(counter<trajectory.size() " << endl;
				counter++;
				ros::spinOnce();
				current_location = get_current_location();
				diff = get_diff(counter, current_location, velocity_multiplier_list[counter]);
				glob_vel_pub.publish(diff);

				cout << " current trajectory (x,y,z): " << trajectory[counter].x << " " << trajectory[counter].y << " " << trajectory[counter].z << endl;
				cout << " current location (x,y,z): " << current_location.x << " " << current_location.y << " " << current_location.z << endl;
				cout << " diff (x,y,z) " << diff.linear.x << " " << diff.linear.y << " " << diff.linear.z << " " << endl;

			}			
		}

		ros::spinOnce();
		
		current_location = get_current_location();
		if (current_location.z > trajectory.back().z && current_location.y > trajectory.back().y)
		{
			cout << " trajectory[-1]: " << trajectory.back().z << endl;
			geometry_msgs::Twist vel;
			vel.linear.x = 0;
			vel.linear.y = 0;
			vel.linear.z = 0;
			glob_vel_pub.publish(vel);
			break;
		}

		ros::Duration(0.1).sleep();
	}
	move_start = false;
	return 0;
}