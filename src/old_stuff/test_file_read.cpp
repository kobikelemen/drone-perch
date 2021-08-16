
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <boost/lexical_cast.hpp>
//#include <geometry_msgs/Vector3.h>
std::ifstream trajectoryFile("/home/kobi/catkin_ws/src/drone_perch/include/trajectories.txt");
using namespace std;
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

	void setState(float x, float z, float x_dot, float z_dot, float x_ang_vel, float z_ang_vel, float thrust)
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
};




std::vector<State> getTrajectory()
{
	bool test = false;
	vector<State> out_list;
	vector<string> fileList;


	string line;
	while(getline(trajectoryFile, line))
	{
		if (test == true){
			fileList.push_back(line);
		}
		string st = boost::lexical_cast<string>(line[0]);
		if (st == "2"){
			test = true;
		}
	}

	for (int i=0; i<fileList.size(); i++){
		State s;


		cout<<"fileList[1][i]:   "<<fileList[1][i]<<endl;
		cout<<"fileList[2][i]:   "<<fileList[2][i]<<endl;
		cout<<"fileList[3][i]:   "<<fileList[3][i]<<endl;

		// float one = stof(fileList[1][i]);
		// float two = stof(fileList[2][i]);
		// float three = stof(fileList[3][i]);
		// float four = stof(fileList[4][i]);
		// float five = stof(fileList[5][i]);
		// float six = stof(fileList[6][i]);
		// float seven = stof(fileList[7][i]);
		// s.setState(one, two, three, four, five, six, seven);
		// out_list.push_back(s);
	}
	return out_list;
}



int main(void)
{

	// std::vector<State> traj = getTrajectory();
	// for (std::vector<State>::const_iterator i=traj.begin(); i!=traj.end(); ++i){
	// 	std::cout <<  "   X: " << (*i).traj_x << "    Y: " << (*i).traj_y << "    Z: " << (*i).traj_z << std::endl;
	// }
	// std::cout << " HI ";
	getTrajectory();
	return 0;
}