

bool move_start;
geometry_msgs::Point current_location;
ros::Publisher state_pub;
#include "drone_perch/state.h"
#include "drone_perch/action.h"
#include <ros/topic.h>
#include <depth.cpp>
