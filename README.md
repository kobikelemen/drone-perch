# drone_perch

Summer 2021 UROP project


The goal of this project is to train a drone agent to produce feasible trajectories which result in the cable hanging off the bottom of the drone wrapping around a target branch.

Two approaches were taken. The first was a control approach where the agent's action space is the throttle on the motors. This was done by making a python simulation using the equations of motion for the system acting as the environment. This was done with both the 2D and 3D dynamics with similar results and was visualised with a matplotlib animation. 


![DCC96FB6-A241-4CF2-9513-434EFA3456A8_1_201_a](https://user-images.githubusercontent.com/85403218/138347417-4ee84e53-c55d-4cdd-80f0-f835c5653ebd.jpeg)


![CF18599D-3A70-4C54-A175-34CD4FCC03EF_1_201_a](https://user-images.githubusercontent.com/85403218/138347570-346a4a58-12b4-4d87-9581-4567b2eef668.jpeg)


The `traj_RL` directory contains all the RL stuff. The Soft Actor Critic model is in `traj_RL/include/SAC`. The control based training approach is in `traj_RL/include/control_RL` while the trajectory based RL is in `traj_RL/include/traj_RL`. To run either one, use the main file in `traj_RL/src/main.py`. The trajectory approach is trained in a Gazebo - ROS environment to ensure accurate and reproducable physical results. This is contained in the `trajectory_follow/sim` directory. The aim of the 'waypoint' approach as opposed to the classic 'control' RL approach is the reduce the search space for the RL - leaving the heavy lifting of the control to the PID controller, allowing it to focus on learning an appropriate policy for perching from many starting points. The simulation in `trajectory_follow/src/trajectroy_follow` has a drone equiped with a camera running object detection to detect suitable branches, and a depth camera to locate them in 3D space for the RL to work.

