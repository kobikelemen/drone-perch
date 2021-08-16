
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <perch_offline.h>
#include <gnc_functions.hpp>
#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>

using namespace std;

ifstream trajectoryFile("/home/kobi/catkin_ws/src/drone_perch/include/trajectories.txt");


ros::Publisher local_attitude_pub;
ros::Publisher thrust_pub;
ros::Publisher glob_vel_pub;
ros::Publisher ang_vel_pub;


// TEMPORARY 
vector<float> x = {-0.25, -0.24677529335021972, -0.22176969051361084, -0.15932204723358154, -0.06146227717399597, 0.05830221772193909, 0.1839709162712097, 0.3101942777633667};
vector<float> z = {0.10002287626266479, 0.11737909317016601, 0.13531502485275268, 0.14429140090942383, 0.14884157180786134, 0.15308607816696168, 0.16214709281921386, 0.23013572692871093};
vector<float> x_vel = {0.0, 0.10996884107589722, 0.49866690635681155, 0.9007103919982911, 1.2398642539978026, 1.3897202491760254, 1.3896968841552735, 1.4101618766784667};
vector<float> z_vel = {0.022876308858394624, 0.21693117618560792, 0.1979393482208252, 0.04631864726543426, 0.045985659956932066, 0.0375369131565094, 0.34567549228668215, 1.1494595527648925};
vector<float> x_ang_vel = {0.0, -0.03672749698162079, -0.1499849319458008, -0.23910691738128662, -0.23534257411956788, -0.1223604679107666, 0.08954054713249207, 0.3999157428741455};
vector<float> z_ang_vel = {-0.0015890903770923615, -0.3016331434249878, -0.08465824127197266, 0.14257628917694093, 0.23916947841644287, 0.15888304710388185, -0.05927037000656128, 0.15834835767745972};
vector<float> thrust = {18.7457213287554, 8.84512331873477, 18.622086090713498, 10.519277744246017, 5.7918431629665236, 13.347905407278574, 19.897654275833787, 20.122330479020212};
vector<float> z_ang = {0.0, -0.27114072, -0.5189998, -0.44301486, -0.262529, -0.06420123, 0.022701548, 0.036011696};
vector<float> x_ang = {0.0, -0.010121762, -0.08757201, -0.26691356, -0.4930182, -0.66342884, -0.68904305, -0.4471425};

//  Z ANGLE ^^ IS THE ONE YOU WANT !!


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



float distance(geometry_msgs::Twist vec)
{
	return sqrt(pow(vec.linear.x,2) + pow(vec.linear.y,2) + pow(vec.linear.z,2));
}


geometry_msgs::Twist get_diff(vector<State> state_list, float counter, geometry_msgs::Point current_location)
{
	geometry_msgs::Twist diff;
	diff.linear.x = 0;//(state_list[counter].traj_x - current_location.x) * state_list[counter].speed_mult;
	diff.linear.y = 0;//state_list[counter].vel_x;   //(state_list[counter].traj_y - current_location.y) * state_list[counter].speed_mult;
	diff.linear.z = 0;//state_list[counter].vel_z; //(state_list[counter].traj_z - current_location.z) * state_list[counter].speed_mult;
	diff.angular.x = state_list[counter].ang_vel_x;
	diff.angular.y = 0;//state_list[counter].ang_vel_x;
	diff.angular.z = 0; // state_list[counter].ang_vel_z;
	//cout << " diff (x,y,z): " << diff.linear.x << " " << diff.linear.y << " " << diff.linear.z << endl;
	return diff;
}




// vector<State> getTrajectory(geometry_msgs::Point current_location)
// {
// 	bool test = false;
// 	vector<State> out_list;
// 	int count_;
// 	for (string line; getline(trajectoryFile, line); )
// 	{	
// 		if (test == true){
// 			if (count_ == 0){
// 				for (int x_coord=1; x_coord < line.length(); x_coord++){
// 					State s = State();
// 					float delta_x = line[x_coord] - line[x_coord-1];
// 					s.traj_y = current_location.y + delta_x; 
// 					out_list.push_back(s);
// 				}
// 			}else if(count_ == 1){
// 				for (int z_coord=1; z_coord < line.length(); z_coord++){
// 					float delta_z = line[z_coord]-line[z_coord-1];
// 					out_list[z_coord-1].traj_z = current_location.z + delta_z;
// 				}
// 			}else if(count_ == 2){
// 				for (int speed=0; speed < line.length(); speed++){
// 					out_list[speed].speed_mult = line[speed];
// 				}
// 			}else if(count_ == 3){
// 				for (int speed=0; speed < line.length(); speed++){
// 					out_list[speed].speed_mult = sqrt(pow(out_list[speed].speed_mult, 2) + pow(line[speed], 2));
// 				}

// 				return out_list;
// 			}	
// 		}

// 		//check what trajecory is wanted ...
// 		string st = boost::lexical_cast<string>(line[0]);

// 		if (st == "2"){
// 			test = true;
// 			count_ = 0;
// 		}
// 	}
// 	return out_list;
// }






// x, y, x_vel, z_vel >>


// TEMPORARY
vector<State> getTrajectory(geometry_msgs::Point current_location)
{
	vector<State> out_list;

	for (int x_coord=1; x_coord < x.size(); x_coord++){
		State s = State();
		s.traj_x = current_location.y;
		float delta_x = x[x_coord] - x[x_coord-1];
		s.traj_y = current_location.y + delta_x; 
		out_list.push_back(s);
	}

	for (int z_coord=1; z_coord < z.size(); z_coord++){
		float delta_z = z[z_coord]-z[z_coord-1];
		out_list[z_coord-1].traj_z = current_location.z + delta_z;
	}

	for (int speed=1; speed < x_vel.size(); speed++){
		//out_list[speed].speed_mult = x_vel[speed];
		out_list[speed-1].vel_x = x_vel[speed];
	}

	for (int speed=1; speed < z_vel.size(); speed++){
		//out_list[speed].speed_mult = sqrt(pow(out_list[speed].speed_mult, 2) + pow(z_vel[speed], 2));
		out_list[speed-1].vel_z = z_vel[speed]; 
	}
	for (int speed=1; speed < x_ang_vel.size(); speed++){
		//out_list[speed].speed_mult = x_vel[speed];
		out_list[speed-1].ang_vel_x = x_ang_vel[speed];
	}

	for (int speed=1; speed < z_ang_vel.size(); speed++){
		//out_list[speed].speed_mult = sqrt(pow(out_list[speed].speed_mult, 2) + pow(z_vel[speed], 2));
		out_list[speed-1].ang_vel_z = z_ang_vel[speed]; 
	}

	for (int thr=1; thr < thrust.size(); thr++){
		//out_list[speed].speed_mult = sqrt(pow(out_list[speed].speed_mult, 2) + pow(z_vel[speed], 2));
		out_list[thr-1].thrust = thrust[thr]; 
	}

	for (int thr=1; thr < x_ang.size(); thr++){
		//out_list[speed].speed_mult = sqrt(pow(out_list[speed].speed_mult, 2) + pow(z_vel[speed], 2));
		out_list[thr-1].x_ang = x_ang[thr]; 
	}

	for (int thr=1; thr < z_ang.size(); thr++){
		//out_list[speed].speed_mult = sqrt(pow(out_list[speed].speed_mult, 2) + pow(z_vel[speed], 2));
		out_list[thr-1].z_ang = z_ang[thr]; 
	}



	for (std::vector<State>::const_iterator i=out_list.begin(); i!=out_list.end(); ++i){
		std::cout <<  "   X: " << (*i).traj_x << "    Y: " << (*i).traj_y << "    Z: " << (*i).traj_z << std::endl;
		std::cout << "   x_vel: " << (*i).vel_x << "    z_vel: " << (*i).vel_z << endl;
	}


	return out_list;
}



float distanceNextPoint(geometry_msgs::Point current_location, int counter, vector<State> state_list)
{
	return sqrt(pow((current_location.y-state_list[counter].traj_y), 2) + pow((current_location.z-state_list[counter].traj_z), 2));
}






// PROBLEM IS THAT distanceNextPoint() INCREASES AS DRONE MOVES ALONG TRAJECTORY .... PROBABLY DID MINUS WRONG WAY AROUND SOMEWHERE...



int perform_trajectory(ros::NodeHandle controlnode)
{
	std::string ros_namespace;
	//trajectoryFile.open("home/kobi/catkin_ws/src/drone_perch/include/trajectories.txt")

	

	cout << " in perform_trajectory" << endl;
	glob_vel_pub = controlnode.advertise<geometry_msgs::Twist>((ros_namespace + "/mavros/setpoint_velocity/cmd_vel_unstamped").c_str(), 10);
	thrust_pub = controlnode.advertise<mavros_msgs::AttitudeTarget>((ros_namespace + "/mavros/setpoint_raw/attitude").c_str(), 10);
	//ang_vel_pub = controlnode.advertise<geometry_msgs::TwistStamped>((ros_namespace + "/mavros/setpoint_attitude/cmd_vel").c_str(), 10);

	//initialize_local_frame();
	move_start = true;
	ros::spinOnce();
	current_location = get_current_location();
	vector<State> state_list = getTrajectory(current_location);
	cout << " current_location before setting trajectory " << current_location << endl;

	ros::Rate rate(1.0);

	int counter = 0;
	geometry_msgs::Twist diff;
	mavros_msgs::AttitudeTarget thrust_target;
		//geometry_msgs::Vector3 vel_ang;
	//geometry_msgs::TwistStamped ang_vel;
	float thrust_cmd;

	while(!(counter >= state_list.size()))
	{	
		// ros::spinOnce();
		// current_location = get_current_location();
		// diff = get_diff(state_list, counter, current_location);
		

		// if (counter == 0){
		// 	glob_vel_pub.publish(diff);
		// 	thrust_cmd = state_list[counter].thrust;
		// 	thrust_target.thrust = thrust_cmd;
		// 	thrust_pub.publish(thrust_target);
		// 	cout << " first diff publish (x,y,z): " << diff.linear.x << " " << diff.linear.y << " " << diff.linear.z << endl; 
		// 	cout << " first thrust: " << thrust_cmd;
		// }



		// cout << " distanceNextPoint() " << distanceNextPoint(current_location, counter, state_list) << endl;
		// cout << " current_location (x,y,z):   " << current_location << endl; 
		// cout << " counter:    " << counter << endl;
		// cout << "state_list (x,y,z):       " << state_list[counter].traj_x << " " << state_list[counter].traj_y << " " << state_list[counter].traj_z << endl;
		
		// if (distanceNextPoint(current_location, counter, state_list) < 0.2 || counter == 0)
		// {	
		// 	//cout << " in if(distance < 0.2 || ...) " << endl;
		// 	if (counter < state_list.size())
		// 	{	
		// 		//cout << " in if(counter<state_list.size() " << endl;
		// 		ros::spinOnce();
		// 		current_location = get_current_location();
		// 		diff = get_diff(state_list, counter, current_location);
		// 		thrust_cmd = state_list[counter].thrust;
		// 		thrust_target.thrust = thrust_cmd;
		// 		thrust_pub.publish(thrust_target);
		// 		glob_vel_pub.publish(diff);
		// 		counter++;


		// 	}			
		// }


		ros::spinOnce();
		current_location = get_current_location();
		//diff = get_diff(state_list, counter, current_location);
		//thrust_target.thrust = 0.5;//
		vector<float> euler_ = {0, -state_list[counter].z_ang, 0};
		geometry_msgs::Quaternion quat = euler2quat(euler_);

		thrust_target.header.stamp = ros::Time::now();
		thrust_target.orientation.x = quat.x;
		thrust_target.orientation.y = quat.y;
		thrust_target.orientation.z = quat.z;
		thrust_target.orientation.w = quat.w;

		thrust_target.body_rate.x = 0; 
		thrust_target.body_rate.y = state_list[counter].ang_vel_z*(180/M_PI); 
		thrust_target.body_rate.z = 0;
		thrust_target.thrust = state_list[counter].thrust;

		thrust_pub.publish(thrust_target);


		// ang_vel.header.stamp = ros::Time::now();
		// ang_vel.twist.angular.x = 0;
		// ang_vel.twist.angular.y = state_list[counter].ang_vel_x*(180/M_PI);
		// ang_vel.twist.angular.z = 0;
		//ang_vel_pub.publish(ang_vel);
		//glob_vel_pub.publish(vel_cmd);
		//glob_vel_pub.publish(diff);

		counter++;
		cout << " current_location (x,y,z):   " << current_location << endl; 
		cout << " counter: " << counter << endl;
		cout << " state_list[counter] (ang_vel_x, thrust) :   " << state_list[counter].ang_vel_x << " " << state_list[counter].thrust << endl;
		cout << " diff:   " << get_diff(state_list, counter, current_location) << endl;
		

		rate.sleep();

		//ros::Duration(1.0).sleep();
		//ros::Duration(1.0) d = ros::Duration(0.1, 0);
		//d.sleep();
		//ros::Duration(0.1).sleep(); // too high frequency ..?

		if (counter == state_list.size())
		{	
			cout << " counter:   " << counter << endl;
			cout <<  " current_location (x,y,z): " << current_location << endl;
			cout << " trajectory[-1] (y,z): " << state_list.back().traj_y << " " << state_list.back().traj_z << endl;
			// geometry_msgs::Twist vel;
			// vel.linear.x = 0;
			// vel.linear.y = 0;
			// vel.linear.z = 0;
			// glob_vel_pub.publish(vel);
			thrust_target.thrust = 0;
			thrust_pub.publish(thrust_target);


			break;
		}

	}
	move_start = false;
	cout << " end of trajectory" << endl;
	return 0;
}






