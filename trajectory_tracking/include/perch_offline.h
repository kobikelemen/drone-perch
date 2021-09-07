bool move_start;
geometry_msgs::Point current_location;
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
	float z_ang;
	float x_ang;
};