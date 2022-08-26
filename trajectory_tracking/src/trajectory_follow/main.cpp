#include </home/kobi/catkin_ws/src/drone_perch/src/trajectory_follow/traj.cpp>



int main(ros::NodeHandle controlnode)
{
	wait4connect();
	wait4start();
	Drone drone;
	drone.droneTakeoff(2.0);
	drone.getTrajectory();
	glob_vel_pub = controlnode.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	initialize_local_frame();
	ros::Rate r(2.0);

	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}