#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <gnc_functions.hpp>
#include <pcl/point_cloud.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/lexical_cast.hpp>
#include <stdlib.h>

using namespace std;

ros::Subscriber depth_sub;
ros::Subscriber bounding_box_sub;
ros::Publisher glob_vel_pub;
ros::Publisher thrust_pub;




class Drone
{

public:

	void droneTakeoff(float height);

	struct State
	{
		float speed_mult;
		float traj_x;
		float traj_y;
		float traj_z;
		float vel_x;
		float vel_z;
		float ang_vel_x;
		float ang_vel_z;
		float thrust;
		void setState(float x, float z, float x_dot, float z_dot, float x_ang_vel, float z_ang_vel, float thrust);
	};


	void getTrajectory();
	void depth_cb (const sensor_msgs::PointCloud2ConstPtr & cloud);
	void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
	bool move_start;
	vector<State> trajectory;
	geometry_msgs::Twist getVel(vector<State> state_list, float counter);
	float distanceNextPoint(geometry_msgs::Point current_location, int counter, vector<State> state_list);
	void moveStartTrajectory(geometry_msgs::Point current_location);
	void performTrajectory();
	void stopDrone();
	geometry_msgs::Point set_destination_coords;
	geometry_msgs::Point branch_location;
	geometry_msgs::Point current_location;
	float branch_pos_x;
	float branch_pos_y;


};