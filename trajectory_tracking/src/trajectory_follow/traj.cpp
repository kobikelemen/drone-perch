
#include <traj.hpp>


// trajectory
vector<float> x = {-0.25, -0.24677529335021972, -0.22176969051361084, -0.15932204723358154, -0.06146227717399597, 0.05830221772193909, 0.1839709162712097, 0.3101942777633667};
vector<float> z = {0.10002287626266479, 0.11737909317016601, 0.13531502485275268, 0.14429140090942383, 0.14884157180786134, 0.15308607816696168, 0.16214709281921386, 0.23013572692871093};
vector<float> x_vel = {0.0, 0.10996884107589722, 0.49866690635681155, 0.9007103919982911, 1.2398642539978026, 1.3897202491760254, 1.3896968841552735, 1.4101618766784667};
vector<float> z_vel = {0.022876308858394624, 0.21693117618560792, 0.1979393482208252, 0.04631864726543426, 0.045985659956932066, 0.0375369131565094, 0.34567549228668215, 1.1494595527648925};
vector<float> x_ang_vel = {0.0, -0.03672749698162079, -0.1499849319458008, -0.23910691738128662, -0.23534257411956788, -0.1223604679107666, 0.08954054713249207, 0.3999157428741455};
vector<float> z_ang_vel = {-0.0015890903770923615, -0.3016331434249878, -0.08465824127197266, 0.14257628917694093, 0.23916947841644287, 0.15888304710388185, -0.05927037000656128, 0.15834835767745972};
vector<float> thrust = {187.457213287554, 88.4512331873477, 186.22086090713498, 105.19277744246017, 57.918431629665236, 133.47905407278574, 198.97654275833787, 201.22330479020212};



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
	// bool test = false;
	// vector<string> fileList;
	// string line;
	// while(getline(trajectoryFile, line))
	// {
	// 	if (test == true){
	// 		fileList.push_back(line);
	// 	}
	// 	string st = boost::lexical_cast<string>(line[0]);
	// 	if (st == "2"){
	// 		test = true;
	// 	}
	// }
	

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
	// diff.linear.x = 0;
	// diff.linear.y = state_list[counter].vel_x;   
	// diff.linear.z = state_list[counter].vel_z; 
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
	//glob_vel_pub = controlnode.advertise<geometry_msgs::Twist>((ros_namespace + "/mavros/setpoint_velocity/cmd_vel_unstamped").c_str(), 10);
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
		// if (counter == 0){
		// 	glob_vel_pub.publish(vel);
		// 	thrust_pub .publish(thrust_cmd);
		// }

		// if ((distanceNextPoint(current_location, counter, state_list) < 0.2 || counter == 0) && counter < state_list.size())
		// {	

		// 	counter++;
		// 	ros::spinOnce();
		// 	current_location = get_current_location();
		// 	vel = getVel(state_list, counter, current_location);
		// 	glob_vel_pub.publish(vel);
		// 	thrust_pub .publish(thrust_cmd);
		// }
	
		ros::spinOnce();
		vel = getVel(trajectory, counter);
		glob_vel_pub.publish(vel);
		thrust_pub.publish(thrust_target);
		current_location = get_current_location();
		//ros::spinOnce();
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


// Depth stuff 
void Drone::depth_cb(const sensor_msgs::PointCloud2ConstPtr & cloud)
{
	if (move_start == false)
	{
	  pcl::PointCloud<pcl::PointXYZ> depth;
	  pcl::fromROSMsg( *cloud, depth);
	  pcl::PointXYZ p1 = depth.at(branch_pos_x, branch_pos_y);
	  cout << "depth at this point : " << p1 << endl;
	  ros::Duration(0.1).sleep();

	  if((p1.y>-0.1 && p1.y<0.1) && (p1.z>0.4 && p1.z<0.7))
			{
				cout <<" correct distance from branch" << endl;
				performTrajectory();

			}else if (!((isnan(p1.x) && isnan(p1.y)) && isnan(p1.z))){
				set_destination_coords.x = p1.x;
				set_destination_coords.y = p1.y;
				set_destination_coords.z = p1.z;
				cout << " in else ... not NaN " << endl;
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
