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
#include <boost/foreach.hpp>
#include <cmath>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <trajectory_functions.hpp>


using namespace std;

//drone_perch::state state;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
vector<vector<double>> temp_points;
vector<double> temp;
ros::Publisher marker_pub;

float branch_pos_x;
float branch_pos_y;
float branch_pos_z;
float rel_branch_x;
float rel_branch_y;
bool perform_traj_bool = false;


//geometry_msgs::Point current_location;
geometry_msgs::Point set_destination_coords;
geometry_msgs::Point branch_location;

// x coords are range(0,2,100), y coords >>
//vector<double> test_trajectory = {0.0, -0.02, -0.042, -0.063, -0.086, -0.109, -0.133, -0.157, -0.182, -0.207, -0.232, -0.258, -0.284, -0.31, -0.336, -0.363, -0.39, -0.416, -0.443, -0.47, -0.496, -0.522, -0.548, -0.574, -0.6, -0.625, -0.65, -0.674, -0.698, -0.721, -0.744, -0.766, -0.787, -0.808, -0.828, -0.847, -0.865, -0.882, -0.899, -0.914, -0.928, -0.941, -0.953, -0.964, -0.973, -0.981, -0.988, -0.993, -0.997, -0.999, -1.0, -0.999, -0.997, -0.993, -0.987, -0.979, -0.969, -0.958, -0.945, -0.929, -0.912, -0.893, -0.871, -0.847, -0.821, -0.793, -0.762, -0.729, -0.694, -0.656, -0.616, -0.573, -0.528, -0.479, -0.429, -0.375, -0.319, -0.259, -0.197, -0.132, -0.064, 0.007, 0.081, 0.159, 0.239, 0.323, 0.41, 0.5, 0.594, 0.691, 0.792, 0.896, 1.004, 1, 1, 1, 1, 1, 1, 1}; // 1.115, 1.23, 1.349, 1.471, 1.598, 1.728, 1.862}

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{	

	for(int i=0; i < msg->bounding_boxes.size(); i++)
	{	
		// Center of bounding box
		branch_pos_x = msg->bounding_boxes[i].xmin+(msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)/2;
		branch_pos_y = msg->bounding_boxes[i].ymin+(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin)/2;

		rel_branch_y = branch_pos_y / 416;
		rel_branch_x = branch_pos_x / 416;

	}
}

void move_start_trajectory(geometry_msgs::Point current_location)
{
	//geometry_msgs::Point original_location = get_current_location();
	
	branch_location.x = current_location.x + set_destination_coords.y;
	branch_location.y = current_location.y + set_destination_coords.z;
	branch_location.z = current_location.z + set_destination_coords.x;
	// FOR ONLINE TRAJECTORY > 

	// state.branch_coords.x = branch_location.x;
	// state.branch_coords.y = branch_location.y;
	// state.branch_coords.z = branch_location.z;
	// state.radius = 0.01;
	cout << " branch_location:   " << branch_location << endl;
	cout << " current_location:   " << current_location << endl;
	set_destination(branch_location.x, branch_location.y-0.55, branch_location.z-current_location.z, 0);
	set_speed(0.05);
	cout << " in move_start_trajectory() " << endl;
	bool start_reached = false;
	geometry_msgs::Point location;
	while (start_reached == false)
	{
		move_start = true;
		ros::spinOnce();
		location = get_current_location();
		bool check_reached = check_waypoint_reached(0.1);
		if (check_reached == true){
			start_reached = true;
			break;
		}	
		ros::Duration(3).sleep();
		cout << " in WHILE LOOP in move_start_trajectory() " << endl;
		cout << " destination set to: " << branch_location.x << " " << branch_location.y-0.55<< " " << branch_location.z-current_location.z << endl;
		cout << " current_location (WHILE LOOP): " << endl << location << endl;
		cout << " branch_location: " << endl << branch_location << endl;
		// if ((location.y<(branch_location.y-0.4) && location.y>(branch_location.y-1.15)) && (location.z>(branch_location.z-0.2) && location.z<(branch_location.z+0.05))){
		// 	start_reached = true;
		// }

		// current_location is always at 0,0,2 ... not changing as moving !!!!
		// Also set_destination_coords (from pcl) is always positive (even if below half of screen)

	}
	cout << " out of WHILE LOOP " << endl;
	perform_traj_bool = true;
	move_start = false;
}


void callback(const sensor_msgs::PointCloud2ConstPtr & cloud) {
	if (move_start == false)
	{
	  pcl::PointCloud<pcl::PointXYZ> depth;
	  pcl::fromROSMsg( *cloud, depth);
	  pcl::PointXYZ p1 = depth.at(branch_pos_x, branch_pos_y);
	  cout << "(pcl cb) bounding box x pos: " << branch_pos_x << endl;
		cout << "bounding box y pos: " << branch_pos_y << endl;
	  cout << "depth at this point : " << p1 << endl;
	  //cout << " p1 " << endl << p1.x << " " << p1.y << " " << p1.z << endl;
	  ros::Duration(0.1).sleep();

	  if((p1.y>-0.1 && p1.y<0.1) && (p1.z>0.4 && p1.z<0.7))
			{

				cout <<" its good " << endl;
				perform_traj_bool = true;

			}else{

				cout << " current_location: (x,y,z) " << current_location.x << " " << current_location.y << " " << current_location.z << endl;
				if (!((isnan(p1.x) && isnan(p1.y)) && isnan(p1.z)))
				{
					set_destination_coords.x = p1.x;
					set_destination_coords.y = p1.y;
					set_destination_coords.z = p1.z;

					cout << " in else ... not NaN " << endl;
					move_start = true;
				}

				// Calling move_start_trajectory() from here makes the drone do a vertical spiral ..... ?

				//move_start_trajectory();
			}
		}
}


void visualize_cloud()
{
	visualization_msgs::Marker points;
	points.header.frame_id = "my_frame";
	points.header.stamp = ros::Time::now();
	points.ns = "depth";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.scale.x = 0.2;
	points.scale.y = 0.2;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.color.g = 1.0f;
	points.color.a = 1.0;

	for(int j=0; j<temp_points.size(); j++)
	{	
		vector<char> branch_label = {'x', 'y', 'z'};
		geometry_msgs::Point p;
		p.x = temp_points[j][0];
		p.y = temp_points[j][1];
		p.z = temp_points[j][2];
		points.points.push_back(p);
	}

	marker_pub.publish(points);
	ros::spinOnce();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "depth");
	ros::NodeHandle nh;
	init_publisher_subscriber(nh);
	wait4connect();
	wait4start();
  	takeoff(2);
  	ros::Rate r(2);

  //current_location = get_current_location();

	while (!(current_location.z>1.9 && current_location.z<2.1) )
	{
		r.sleep();
		ros::spinOnce();
		current_location = get_current_location();
	}
	ros::Subscriber sub2 = nh.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);
	ros::Subscriber sub = nh.subscribe("/camera/depth/points",1, callback);
	//marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);


	initialize_local_frame();

	while(ros::ok())
	{
		r.sleep();
		current_location = get_current_location();
		cout << " move_start: " << move_start << endl;
		
		if (move_start == true)
		{
			cout << " in if (move_start == true) " << endl;
			move_start_trajectory(current_location);
			move_start = false;
		}
		if (perform_traj_bool == true)
		{
			cout << " in if (perform_traj_bool==true) " << endl;
			perform_trajectory(nh);
			perform_traj_bool = false;
		}
		ros::spinOnce();
	}
	return 0;
}

	








