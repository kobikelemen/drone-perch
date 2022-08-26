
#include <traj.hpp>


ifstream trajectoryFile("/home/kobi/catkin_ws/src/drone_perch/include/trajectories.txt");
using namespace std;


void Drone::State::setState(float x, float z, float x_dot, float z_dot, float x_ang_vel, float z_ang_vel, float thrust)
{
	speed_mult = 1;
	traj_x = x;
	traj_y = 0;
	traj_z = z; 
	vel_x = x_dot;
	vel_z = z_dot;
	ang_vel_x = x_ang_vel;
	ang_vel_z = z_ang_vel;
	thrust = thrust;
}



void Drone::stopDrone()
{
	geometry_msgs::Twist velocity;
	velocity.linear.x = 0;
	velocity.linear.y = 0;
	velocity.linear.z = 0;
	glob_vel_pub.publish(velocity);
}



void Drone::moveStartTrajectory(geometry_msgs::Point current_location)
{
	branch_location.x = current_location.x + set_destination_coords.y;
	branch_location.y = current_location.y + set_destination_coords.z;
	branch_location.z = current_location.z + set_destination_coords.x;
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
		ros::Duration(0.05).sleep();
		if ((location.y<(branch_location.y-0.4) && location.y>(branch_location.y-1.15)) && (location.z>(branch_location.z-0.2) && location.z<(branch_location.z+0.05))){
			start_reached = true;
		}
	}
	performTrajectory();
	move_start = false;
}



void Drone::getTrajectory()
{
	vector<State> out_list;
	for (int i=0; i<x.size(); i++){
		State s;
		s.setState(x[i], z[i], x_vel[i], z_vel[i], x_ang_vel[i], z_ang_vel[i], thrust[i]);
		out_list.push_back(s);
	}
	trajectory = out_list;
}



geometry_msgs::Twist Drone::getVel(vector<State> state_list, float counter)
{
	geometry_msgs::Twist diff;
	diff.angular.x = state_list[counter].ang_vel_x;
	diff.angular.y = 0;
	diff.angular.z = 0; 
	return diff;
}



float Drone::distanceNextPoint(geometry_msgs::Point current_location, int counter, vector<State> state_list)
{
	return sqrt(pow((current_location.y-state_list[counter].traj_y), 2) + pow((current_location.z-state_list[counter].traj_z), 2));
}



void Drone::performTrajectory()
{
	std::string ros_namespace;
	move_start = true;
	ros::spinOnce();
	current_location = get_current_location();
	int counter = 0;
	geometry_msgs::Twist vel;
	float thrust_cmd;

	while(!(current_location.z > trajectory.back().traj_z && current_location.y > trajectory.back().traj_y))
	{	
		ros::spinOnce();
		current_location = get_current_location();
		vel = getVel(trajectory, counter);
		thrust_cmd = trajectory[counter].thrust;
		mavros_msgs::AttitudeTarget thrust_target;
		thrust_target.thrust = thrust_cmd;
		ros::spinOnce();
		vel = getVel(trajectory, counter);
		glob_vel_pub.publish(vel);
		thrust_pub.publish(thrust_target);
		current_location = get_current_location();
		counter++;
		
		if (current_location.z > trajectory.back().traj_z && current_location.y > trajectory.back().traj_y)
		{	
			stopDrone();
			break;
		}

		ros::Duration(45/500).sleep(); 
	}
	move_start = false;
}



void Drone::droneTakeoff(float height)
{
	current_location = get_current_location();
	takeoff(height);
	while (!(current_location.z>height-0.1 && current_location.z<height+0.1) )
	{
		ros::Duration(0.1).sleep();
		ros::spinOnce();
		current_location = get_current_location();
	}
}



void Drone::depth_cb(const sensor_msgs::PointCloud2ConstPtr & cloud)
{
	if (move_start == false)
	{
	  pcl::PointCloud<pcl::PointXYZ> depth;
	  pcl::fromROSMsg( *cloud, depth);
	  pcl::PointXYZ p1 = depth.at(branch_pos_x, branch_pos_y);
	  ros::Duration(0.1).sleep();

	  if((p1.y>-0.1 && p1.y<0.1) && (p1.z>0.4 && p1.z<0.7))
			{
				performTrajectory();
			}else if (!((isnan(p1.x) && isnan(p1.y)) && isnan(p1.z))){
				set_destination_coords.x = p1.x;
				set_destination_coords.y = p1.y;
				set_destination_coords.z = p1.z;
				moveStartTrajectory(current_location);
				}
		}
}

void Drone::detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	for(int i=0; i < msg->bounding_boxes.size(); i++)
	{	
		branch_pos_x = msg->bounding_boxes[i].xmin+(msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)/2;
		branch_pos_y = msg->bounding_boxes[i].ymin+(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin)/2;
	}
}
