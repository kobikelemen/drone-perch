#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetModelState


getPayloadState = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
rospy.wait_for_service('/gazebo/get_link_state')
print(getPayloadState('branch_new_2m::link_0', 'ground_plane'))
#print('pos: ', state.link_state.pose.position.x, state.link_state.pose.position.y, state.link_state.pose.position.z)