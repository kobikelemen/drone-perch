
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


//hi




using namespace std;


//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// added to cmakelist file in this folder, not sure about it






// void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
// {	
	
// 	for(int i=0; i < msg->bounding_boxes.size(); i++)
// 	{	

// 		cout << "class: " << msg->bounding_boxes[i].Class << endl;

// 		branch_pos_x = msg->bounding_boxes[i].xmin+(msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)/2;
// 		branch_pos_y = msg->bounding_boxes[i].ymin+(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin)/2;

// 		cout << "x pos: " << branch_pos_x << endl;
// 		cout << "y pos: " << branch_pos_y << endl;

// 		//cout<<"xmin: " << msg->bounding_boxes[i].xmin <<endl;



// 		//ROS_INFO("%s detected", msg->bounding_boxes[i].Class);
// 		//ROS_INFO("x position is "+((msg->bounding_boxes[i].xmin+msg->bounding_boxes[i].xmax)/2).c_str());

// 		//ROS_INFO("branch xmin:");
// 		//ROS_INFO(msg->bounding_boxes[0].xmin);


// 		//branch_pos_x = atoi(msg->bounding_boxes[i].xmin.c_str())+(atoi(msg->bounding_boxes[i].xmax.c_str())-atoi(msg->bounding_boxes[i].xmin.c_str()))/2;
// 		//branch_pos_y = atoi(msg->bounding_boxes[i].ymin.c_str())+(atoi(msg->bounding_boxes[i].ymax.c_str())-atoi(msg->bounding_boxes[i].ymin.c_str()))/2;
// 		//ROS_INFO("branch pos: %", branch_pos_x);

// 	}
// }





// 	// cout << "point cloud: " << endl;
// 	// for(int i=0; i<msg->data.size(); i++)
// 	// {	
// 	// 	cout << msg->data[i] << endl;
// 	// //ROS_INFO("point cloud: %", msg->data);
// 	// }

// }








// void detection_depth(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
// {
//     pcl::PCLPointCloud2 pcl_pc2;
//     pcl_conversions::toPCL(*input,pcl_pc2);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);


//     for(int i=0; i<temp_cloud->width * temp_cloud->height; i++)
//     {	
//     	cout << temp_cloud->points[i] << endl;
//     }
//     //cout << "coords: " << temp_cloud->points[int (temp_cloud->width)/2] << endl;


//     //cout << "mid y: " << temp_cloud->points[int (temp_cloud->height)/2].y << endl;

//     // for(int i=0; i<temp_cloud->height; i++)
//     // {
//     // 	cout << "x pos: " << temp_cloud->points[i].x << endl;
//     // 	cout << "y pos: " << temp_cloud->points[i].y << endl;
//     // 	cout << "z pos: " << temp_cloud->points[i].z << endl;
//     // }
//     // THE ABOVE LINE WORKDS ^^^ IF YOU REMOVE FOR LOOP BELOW >
// }

// int main(int argc, char **argv)
// {

// 	ros::init(argc, argv, "cameras_fuse");
// 	ros::NodeHandle n;

// 	//ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);
// 	ros::Subscriber sub = n.subscribe("/camera/depth/points", 1, detection_depth);
// 	ros::spin();
// }



















// void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
// {	


// 	pcl::PCLPointCloud2 pcl_pc2;
//     pcl_conversions::toPCL(*input,pcl_pc2);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);



//     //ROS_INFO(temp_cloud->points);

// 	for(int i=0; i<temp_cloud->points.size(); i++)
// 	{

// 		ROS_INFO("X : %f",  temp_cloud->points[i].x.c_str());
//     	ROS_INFO("Y : %f",  temp_cloud->points[i].y.c_str());
//     	ROS_INFO("Z : %f",  temp_cloud->points[i].z.c_str());
// 	}
// }




// int main(int argc, char** argv)
// {
// 	ros::init(argc, argv, "sub_pcl");
// 	ros::NodeHandle nh;
// 	ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, callback);
// 	ros::Duration(5).sleep();
// 	//ros::spin();
	
// 	}
























// THIS KINDOF WORKS? >>




// #include <ros/ros.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <boost/foreach.hpp>
// #include <cmath>
// #include <iostream>

// using namespace std;

// #define foreach BOOST_FOREACH

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// void callback(const PointCloud::ConstPtr& msg)
// {	
// 	cout << "hi in callback";
// 	ROS_INFO("Cloud: width = %d, height = %d\n", msg->width, msg->height);
// 	BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
// 	{	
// 		if (isnan(pt.z)==false)
// 		{
//     		ROS_INFO("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
// 		}
// 	}
// }
// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "sub_pcl");
//   ros::NodeHandle nh;
//   cout << "hi";
//   //ROS_INFO("ros info");
//   ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, callback);
//   ros::spin();

// }


























//#define foreach BOOST_FOREACH

// void callback(const PointCloud::ConstPtr& msg)
// {	
// 	cout << "hi in callback";
// 	vector<vector<double>> temp_points;
// 	ROS_INFO("Cloud: width = %d, height = %d\n", msg->width, msg->height);

// 	BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
// 	{	
// 		if (isnan(pt.z)==false)
// 		{
//     		cout << ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
//     		vector<double> temp {pt.x, pt.y, pt.z};
//     		//cout << "type: " << typeid(temp).name() << endl;
//     		temp_points.push_back(temp);

// 		}
// 	}
// }


































#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <cmath>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <stdlib.h>
using namespace std;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
vector<vector<double>> temp_points;
vector<double> temp;

double branch_pos_x;
double branch_pos_y;
double cam_frame_width;
double cam_frame_height;
double depth_frame_width;
double depth_frame_height;
double branch_pos_z;
vector<double> branch_coords;



void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{	
	
	for(int i=0; i < msg->bounding_boxes.size(); i++)
	{	

		// Center of bounding box

		branch_pos_x = msg->bounding_boxes[i].xmin+(msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)/2;
		branch_pos_y = msg->bounding_boxes[i].ymin+(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin)/2;

		//cam_frame_width = msg->width;
		//cam_frame_height = msg->height;

		cout << "x pos: " << branch_pos_x << endl;
		cout << "y pos: " << branch_pos_y << endl;

	}
}

void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
	// Converts raw pointcloud to PCL; this gives access the raw coordinates in the pointcloud

	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*input,pcl_pc2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
	temp_points.erase(temp_points.begin(), temp_points.end());


	// Trying to relate the centre of bounding box to centre of point cloud to get coordinates of branch
	// (This is where the problem is I believe)

	depth_frame_width = input->width;
	depth_frame_height = input->height;

	int x = round(branch_pos_x);
	int y = round(branch_pos_y);
	int width = temp_cloud->width;

	branch_pos_z = temp_cloud->points[width*y + x].z;


	for(int i=0; i<temp_cloud->points.size(); i++)
	{		
		// Filters out NaN
		if(isnan(temp_cloud->points[i].z)==false && isnan(temp_cloud->points[i].y)==false && isnan(temp_cloud->points[i].x)==false)
		{
			temp = {temp_cloud->points[i].x, temp_cloud->points[i].y, temp_cloud->points[i].z};
			temp_points.push_back(temp);
		}
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth");
  ros::NodeHandle nh;
  cout <<"hi";
  // Recieves point cloud from kinect and bounding box of branch 

	ros::Subscriber sub = nh.subscribe("/camera/depth/points",10, callback);
	ros::Subscriber sub2 = nh.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);

	// Publishes markers to rviz (points which aren't NaN)

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	float f = 0.0;
  ros::Rate r(30);

	while(ros::ok())
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
		//cout << "temp_points size: " << temp_points.size() << endl;

		for(int j=0; j<temp_points.size(); j++)
		{	
			//cout << (" temp_points [%f]: ", j);

			// for(vector<double>::const_iterator i=temp_points[j].begin(); i!=temp_points[j].end(); ++i)
			// {
			// 	cout << " " << *i;
			// }


			// Print points
			int count = 0;
			vector<char> branch_label = {'x', 'y', 'z'};
			for(vector<double>::const_iterator i=temp_points[j].begin(); i!=temp_points[j].end(); ++i)
			{	
				// check count is correct

				// if(count==0){
				// 	cout << " branch coords" << endl;
				// }
				// cout <<  branch_label[count] << endl;
				// count++;
				// cout << " " << *i << endl;
			}

			geometry_msgs::Point p;
			p.x = temp_points[j][0];
			p.y = temp_points[j][1];
			p.z = temp_points[j][2];
			points.points.push_back(p);
		}

		marker_pub.publish(points);
		
		// ROS_INFO("branch_pos (x, y, z)");
		// ROS_INFO(" %f", branch_pos_x);
		// ROS_INFO(" %f", branch_pos_y);
		// ROS_INFO(" %f", branch_pos_z);

		ros::spinOnce();
		r.sleep();
		f += 0.04;
	}
	return 0;
}

	
