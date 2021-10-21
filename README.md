# drone_perch

Summer 2021 UROP project


The goal of this project is to train a drone agent to produce feasible trajectories which result in the cable hanging off the bottum of the drone wrapping around a target branch.

Two approaches were taken. The first was a control approach where the agent's action space is the throttle on the motors. This was done by making a python simulation using the equations of motion for the system acting as the environment. This was done with both the 2D and 3D dynamics with similar results. This was visualised with a matplotlib animation. 

![DCC96FB6-A241-4CF2-9513-434EFA3456A8_1_201_a](https://user-images.githubusercontent.com/85403218/138271286-142f58b5-3445-4b3e-9a87-3869dd68627a.jpeg)


There are two parts, the first is the 'traj_RL' directory. This contains a Soft Actor Critic RL model which is setup to train the agent to produce a series of waypoints which make up a trajectory to perch the drone. The model is trained in a custom Gazebo - ROS envirnment to ensure accurate and reproducable physical results. The aim of the 'waypoint' approach as opposed to the classic 'control' RL approach is the reduce the search space for the RL - leaving the heavy lifting of the control to the Ardupilot controller, allowing it to focus on learning an appropriate policy for perching from many starting points. 

The second part is the 'trajectory_follow' directory which contains a realistic Gazebo - ROS simulation of a drone, equipped with a camera and a depth camera. There is a trained object detection algorithm for detecting suitable branches for perching, then the depth camera detects the distance and and size of the target. The pre-determined trajectory is then followed.
