#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetModelState

rospy.wait_for_service('/gazebo/get_link_state')
getPayloadState = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

print(' \n \n get_link_state:   \n ', getPayloadState('drone_kinect_fixed_rope2::cam_link', 'ground_plane'))
#print('pos: ', state.link_state.pose.position.x, state.link_state.pose.position.y, state.link_state.pose.position.z)

rospy.wait_for_service('/gazebo/get_model_state')
getModelState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
print(' \n \n get_model_state:   \n ', getPayloadState('drone_kinect_fixed_rope2', 'link_0'))