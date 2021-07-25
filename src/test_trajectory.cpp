#include <gnc_functions.hpp>
#include <math.h>
#include <cmath>


using namespace std;
vector<float> trajectory = {0.0, -0.232, -0.496, -0.744, -0.928, -1.0, -0.912, -0.616, -0.064, 0.792};
//vector<float> trajectory = {0.66, 1.32, 2};

// void perform_trajectory(vector<float> trajectory=test_trajectory)
// {	
// 	int i=1;
// 	gnc_api_waypoint nextWaypoint;
// 	nextWaypoint.x = 0;
// 	nextWaypoint.y = 0.2;
// 	nextWaypoint.z = trajectory[0]+1;
// 	nextWaypoint.psi = 0;

// 	set_destination(nextWaypoint.x, nextWaypoint.y, nextWaypoint.z, nextWaypoint.psi);

// 	cout << " inside perform_trajectory " << endl;
// 	while(i<trajectory.size())
// 	{	
// 		cout << " inside while loop" << endl;
// 			//cout << " inside perform_trajectory ";
// 			//float y_diff = trajectory[i] - trajectory[i-1];

// 		if (check_waypoint_reached(0.1) == 1)
// 		{	
// 			cout << " inside check_waypoint_reached " << endl;
// 			set_destination(0, 0.2, trajectory[i]+1, 0);
// 			i++;
// 			//cout << " inside check_waypoint_reached " << endl;
// 		}
// 		rate.sleep()
	
// 	}
// }

// PRETTY SURE TRAJECTORY POINTS ARE TOO CLOSE TOGETHER FOR TOLERANCE <<<<<<


float distance(geometry_msgs::Point vec)
{
	return sqrt(pow(vec.x,2) + pow(vec.y,2) + pow(vec.z,2));
}

vector<float> euler2quat(vector<float> euler)
{	

	// MAKE SURE EULER ANGLES ARE RADIANS <<< !!!!!!!

	float w = cos(euler[1]/2)*cos(euler[0]/2)*cos(euler[2]/2) - sin(euler[1]/2)*sin(euler[0]/2)*sin(euler[2]/2);
	float x = sin(euler[1]/2)*sin(euler[0]/2)*cos(euler[2]/2) + cos(euler[1]/2)*cos(euler[0]/2)*sin(euler[2]/2);
	float y = sin(euler[1]/2)*cos(euler[0]/2)*cos(euler[2]/2) + cos(euler[1]/2)*sin(euler[0]/2)*sin(euler[2]/2);
	float z = cos(euler[1]/2)*sin(euler[0]/2)*cos(euler[]2/2) - sin(euler[1]/2)*cos(euler[0]/2)*sin(euler[2]/2);
	return {w, x, y, z};
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_trajectory");
	ros::NodeHandle n;
	init_publisher_subscriber(n);


	wait4connect();
	wait4start();
	

	initialize_local_frame();
	set_speed(20);
	takeoff(2);
	ros::Rate rate(2.0);

	float y_ = 0.2;


	vector<gnc_api_waypoint> waypointlist;
	gnc_api_waypoint nextWaypoint;
	for(int i=0; i<trajectory.size(); i++)
	{
		nextWaypoint.x = 0;
		nextWaypoint.y = y_;
		nextWaypoint.z = trajectory[i]+2.0;
		nextWaypoint.psi = 0;
		y_ += 0.2;
		waypointlist.push_back(nextWaypoint);

	}




	int counter = 0;

	while(ros::ok())
	{	
		ros::spinOnce();
		rate.sleep();


		// if (check_waypoint_reached(0.1)==1)
		// {	
		// 	set_destination(0,2,2,0);
		// }
		// if (count==0)
		// {
			
		// 	nextWaypoint.x = 0;
		// 	nextWaypoint.y = y;
		// 	nextWaypoint.z = trajectory[0]+1.0;
		// 	nextWaypoint.psi = 0;

		// 	set_destination(nextWaypoint.x, nextWaypoint.y, nextWaypoint.z, nextWaypoint.psi);
		// }


		//if (check_waypoint_reached(0.05)==1)

		geometry_msgs::Point location;
		location = get_current_location();

		geometry_msgs::Point diff;
		diff.x = location.x - waypointlist[counter].x;
		diff.y = location.y - waypointlist[counter].y;
		diff.z = location.z - waypointlist[counter].z;

		cout << " distance : " << distance(diff) << endl;

		// THINK TO DO WITH NOT USING check_distance() FUNCTINO 

		if (distance(diff) < 0.4)
		{	
			cout << " in 0.1 < dis < 0.2" << endl;

			if (counter < waypointlist.size())
			{	
				cout << " in counter < waypointlist.size() " << endl;
				counter++;
				set_destination(waypointlist[counter].x, waypointlist[counter].y, waypointlist[counter].z, waypointlist[counter].psi);
				
			}			
		}	
	}
		
	return 0;	

}