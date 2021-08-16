#include <ros/ros.h>
#include <gnc_functions>


ros::Subscriber depth_sub;
ros::Subscriber bounding_box_sub;
ros::Publisher glob_vel_pub;
ros::Publisher thrust_pub;