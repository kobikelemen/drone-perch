# drone_perch

Summer 2021 UROP project


The goal of this project is to train a drone agent to produce feasible trajectories which result in the cable hanging off the bottum of the drone wrapping around a target branch.

There are two parts, the first is the 'traj_RL' directory. This contains a Soft Actor Critic RL model which is setup to train the agent to produce a series of waypoints which make up a trajectory to perch the drone. The model is trained in a custom Gazebo - ROS envirnment to ensure accurate and reproducable physical results. The aim of the 'waypoint' approach as opposed to the classic 'control' RL approach is the reduce the search space for the RL - leaving the heavy lifting of the control to the Ardupilot controller, allowing it to focus on learning an appropriate policy for perching from many starting points. 

The second part is the 'trajectory_follow' directory which contains a realistic Gazebo - ROS simulation of a drone, equipped with a camera and a depth camera. There is a trained object detection algorithm for detectinng suitable branches for perching, then the depth camera detects the distance and and size of the target. The pre-determined trajectory is then followed.
