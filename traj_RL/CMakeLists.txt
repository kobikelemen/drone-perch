cmake_minimum_required(VERSION 3.0.2)
project(traj_RL)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros
  mavros_msgs
  # WILL NEED darknet_ros_msgs & darknet_ros IN FUTURE ...

)

catkin_python_setup()

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES traj_RL
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#  mavros_msgs
#)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set(Torch_DIR "/home/kobi/.local/lib/python2.7/site-packages/torch/share/cmake/Torch")
#set(TORCH_BUILD_DIR "/.local/lib/python2.7/site-packages/torch/share/cmake/Torch")
find_package(Torch REQUIRED)
set(torch_DIR "/home/kobi/.local/lib/python2.7/site-packages/torch")

include_directories(
  ${Torch_INSTALL_INCLUDE}
  ${Torch_DIR}
  #${TORCH_BUILD_DIR}
  ${torch_DIR}
  )

link_directories(
  ${Torch_INSTALL_LIB}
)

#catkin_install_python(PROGRAMS
#  scripts/main.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
