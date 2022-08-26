cmake_minimum_required(VERSION 3.0.2)
project(traj_RL)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros
  mavros_msgs
)

catkin_python_setup()



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set(Torch_DIR "/home/kobi/.local/lib/python2.7/site-packages/torch/share/cmake/Torch")
find_package(Torch REQUIRED)
set(torch_DIR "/home/kobi/.local/lib/python2.7/site-packages/torch")

include_directories(
  ${Torch_INSTALL_INCLUDE}
  ${Torch_DIR}
  ${torch_DIR}
  )

link_directories(
  ${Torch_INSTALL_LIB}
)