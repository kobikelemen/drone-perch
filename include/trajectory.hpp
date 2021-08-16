

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
#include <sensor_msgs/Imu.h>
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



ros::Publisher local_attitude_pub;
ros::Publisher thrust_pub;
ros::Publisher glob_vel_pub;
ros::Publisher attitude_pub;
ros::Publisher force_pub;
ros::Subscriber attitude_sub;
ros::Subscriber velocity_sub;
drone_perch::action action_msg;

int time_check = 0;



float distance(geometry_msgs::Twist vec)
{
	return sqrt(pow(vec.linear.x,2) + pow(vec.linear.y,2) + pow(vec.linear.z,2));
}



geometry_msgs::Vector3 quat_2_euler(geometry_msgs::Quaternion quat)
{
	geometry_msgs::Vector3 euler;
	float t0 = 2 * (quat.w*quat.x + quat.y*quat.z);
	float t1 = 1 - 2 * (quat.x*quat.x + quat.y*quat.y);
	euler.x = atan2(t0, t1) * 180/M_PI;  //roll

	float t2 = 2 * (quat.w*quat.y - quat.z*quat.x);
	if(t2 > 1.0) {
		t2 = 1.0;
	}else if(t2 < -1){
		t2 = -1;
	}
	euler.y = asin(t2) * 180/M_PI;  //pitch

	float t3 = 2 * (quat.w*quat.z + quat.x*quat.y);
	float t4 = 1 - 2 * (quat.y*quat.y + quat.z*quat.z);
	euler.z = atan2(t3, t4) * 180/M_PI; //yaw

	return euler;

}


geometry_msgs::Quaternion euler_2_quat(float roll, float pitch, float yaw)
{	
	// MAKE SURE EULER ANGLES ARE DEGREES !!!!!!!
	vector<float> euler = {roll, pitch, yaw}; 

	// I think roll pitch yaw from action_msg are already radians ...

	// euler[0] = euler[0] * M_PI/180;
	// euler[1] = euler[1] * M_PI/180;
	// euler[2] = euler[2] * M_PI/180;

	geometry_msgs::Quaternion q;
	q.w = cos(euler[1]/2)*cos(euler[0]/2)*cos(euler[2]/2) - sin(euler[1]/2)*sin(euler[0]/2)*sin(euler[2]/2);
	q.x = sin(euler[1]/2)*sin(euler[0]/2)*cos(euler[2]/2) + cos(euler[1]/2)*cos(euler[0]/2)*sin(euler[2]/2);
	q.y = sin(euler[1]/2)*cos(euler[0]/2)*cos(euler[2]/2) + cos(euler[1]/2)*sin(euler[0]/2)*sin(euler[2]/2);
	q.z = cos(euler[1]/2)*sin(euler[0]/2)*cos(euler[2]/2) - sin(euler[1]/2)*cos(euler[0]/2)*sin(euler[2]/2);
	return q;
}


void attitude_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
	geometry_msgs::Vector3 angles = quat_2_euler(msg->orientation);
	geometry_msgs::Vector3 ang_vel = msg->angular_velocity;
	geometry_msgs::Vector3 acceleration = msg->linear_acceleration;

	state.attitude = angles;
	state.attitude_dot = ang_vel;

}


geometry_msgs::Vector3 thrust_2_foce_vec(float thrust, float roll, float pitch, float yaw)
{
	// Assuing attitude is radians ...

	geometry_msgs::Vector3 force;
	force.x = cos(roll) * thrust;
	force.y = cos(pitch) * thrust;
	force.z = cos(yaw) * thrust;
	return force;
}


void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& vel_msg)
{
	geometry_msgs::Twist temp = vel_msg->twist;
	state.velocity = temp.linear;
}


//void check_null_cb(const )




int perform_trajectory(ros::NodeHandle controlnode)
{
	std::string ros_namespace;

	cout << " in perform_trajectory" << endl;
	glob_vel_pub = controlnode.advertise<geometry_msgs::Twist>((ros_namespace + "/mavros/setpoint_velocity/cmd_vel_unstamped").c_str(), 10);
	state_pub = controlnode.advertise<drone_perch::state>("state", 10);
	attitude_sub = controlnode.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 5, attitude_cb);
	//action_sub = n.subscribe<drone_perch::action>("action", 5, action_cb);
	attitude_pub = controlnode.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 5);
	force_pub = controlnode.advertise<geometry_msgs::Vector3>("/mavros/setpoint_accel/accel", 5);
	velocity_sub = controlnode.subscribe<geometry_msgs::TwistStamped>("/mavros/global_position/raw/gp_vel", 5, velocity_cb);

	ros::spinOnce();
	current_location = get_current_location();
	cout << " current_location before setting trajectory " << current_location << endl;

	int counter = 0;
	geometry_msgs::Twist diff;
	vector<float> velocity_multiplier_list = {1, 1.15, 1.35, 0.8, 0.35, 0.2};
	//vector<float> velocity_multiplier_list = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

	boost::shared_ptr<drone_perch::action> check_null;

	check_null = ros::topic::waitForMessage<drone_perch::action>("action");
	if (check_null != NULL){
		action_msg = *check_null; 
	}
	geometry_msgs::Quaternion set_attitude = euler_2_quat(action_msg.roll, action_msg.pitch, action_msg.yaw);
	geometry_msgs::Vector3 set_force = thrust_2_foce_vec(action_msg.thrust, action_msg.roll, action_msg.pitch, action_msg.yaw);

	attitude_pub.publish(set_attitude);
	force_pub.publish(set_force);


	while(action_msg.done != true)
	{	
		cout << " in while loop in perform_trajectory " << endl;
		ros::spinOnce();
		state.drone_coords = get_current_location();
		
		if (time_check != 0){
			state.header.stamp = (time - ros::Time::now()).toSec();
			ros::Time::now() time;

		}else{
			
			state.header.stamp = 0.1;
			ros::Time::now() time;
			time_check = 1;
		}
		state.radius = 0.01;
		state.cable_angle_theta = 0;
		state.cable_angle_phi = 0;
		state.cable_angle_dot_theta = 0;
		state.cable_angle_dot_phi = 0;
		state.branch_coords = branch_coords;

		// I think state.branch_coords is local coords but should be GLOBAL coords ...

		state_pub.publish(state);
		
		time = ros::Time::now();

		check_null = ros::Topic::waitForMessage<drone_perch::action>("action");
		if (check_null != NULL){
			action_msg = *check_null; 
		}

		set_attitude = euler_2_quat(action_msg.roll, action_msg.pitch, action_msg.yaw);
		set_foce = thrust_2_foce_vec(action_msg.thrust, action_msg.roll, action_msg.pitch, action_msg.yaw);

		attitude_pub.publish(set_attitude);
		force_pub.publish(set_force);

	}
	return 0;
}