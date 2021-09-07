
cmake_minimum_required(VERSION 3.0.2)
project(drone_perch)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  darknet_ros_msgs
  darknet_ros
  mavros
  mavros_msgs

)


catkin_python_setup()



 add_message_files(
   FILES
   state.msg
   action.msg
   )
#   Message2.msg
#)

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs

)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drone_perch
#  CATKIN_DEPENDS roscpp rospy std_msgs mavros mavros_msgs
#  DEPENDS system_lib
)

add_compile_options(-std=c++14)

#cmake_minimum_required(VERSION 2.8.3)
#project(inference_pytorch)


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


###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)




add_executable(depth src/trajectory_follow/depth.cpp)
###### DONT INCLUDE THIS >>> target_link_libraries(depth ${catkin_LIBRARIES})
add_dependencies(depth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(main src/trajectory_follow/main.cpp)
target_link_libraries ( main ${PCL_LIBRARIES} ${catkin_LIBRARIES}  )
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(traj src/trajectory_follow/traj.cpp)
###########target_link_libraries ( traj ${PCL_LIBRARIES} ${catkin_LIBRARIES}  )
add_dependencies(traj ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})






add_executable(test_trajectory src/trajectory_follow/test_trajectory.cpp)
target_link_libraries(test_trajectory ${catkin_LIBRARIES})
add_dependencies(test_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_2m src/trajectory_follow/test_2m.cpp)
target_link_libraries(test_2m ${catkin_LIBRARIES})


#############
## Install ##
#############


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

link_directories(
  ${Torch_INSTALL_LIB}
)


## ??? >>>> 

install(PROGRAMS
   src/RL/Model.py
   src/RL/Policy.py
   src/RL/Q.py
   src/RL/ReplayBuffer.py
   src/RL/Value.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 catkin_install_python(PROGRAMS 
                      src/RL/Model.py
                      src/RL/Policy.py
                      src/RL/Q.py
                      src/RL/ReplayBuffer.py
                      src/RL/Value.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


##  ???  <<<< 


#############
## Testing ##
#############



## FOR PCL >>

find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries ( traj ${PCL_LIBRARIES} ${catkin_LIBRARIES}  )
target_link_libraries ( main ${PCL_LIBRARIES} ${catkin_LIBRARIES}  )
target_link_libraries ( depth ${PCL_LIBRARIES} ${catkin_LIBRARIES}  )

##

catkin_package(CATKIN_DEPENDS message_runtime)

target_link_libraries ( test_trajectory ${PCL_LIBRARIES} ${catkin_LIBRARIES}  )

find_package(Boost REQUIRED regex date_time system filesystem thread graph)
find_package( catkin REQUIRED COMPONENTS message_generation)
include_directories(${BOOST_INCLUDE_DIRS})
message("boost lib: ${Boost_LIBRARIES}")
message("boost inc ${Boost_INCLUDE_DIR}")


## USING BOOST LIBRARY BREAKS EVERYTHING ... :(
#target_link_libraries(depth ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})



