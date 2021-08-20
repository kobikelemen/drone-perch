#include </home/kobi/catkin_ws/src/drone_perch/src/traj.cpp>



int main(int argc, char** argv)
{

	ros::init(argc, argv, "main");
	ros::NodeHandle nh;
	wait4connect();
	wait4start();
	Drone DroneObject;
	//Drone Traj;
	DroneObject.droneTakeoff(2.0);
	DroneObject.getTrajectory();

	bounding_box_sub = nh.subscribe("/darknet_ros/bounding_boxes", 5, &Drone::detection_cb, &DroneObject);
	depth_sub = nh.subscribe("/camera/depth/points",5, &Drone::depth_cb, &DroneObject);
	glob_vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

	initialize_local_frame();
	ros::Rate r(2.0);

	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}