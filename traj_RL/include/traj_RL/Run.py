#!/usr/bin/env python

from traj_RL import *
from math import pi, atan, atan2, cos, sin, acos, asin, tan, sqrt
from copy import deepcopy
import roslaunch
import rospy
from std_srvs.srv import Empty
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from gazebo_msgs.srv import GetLinkState, DeleteModel, SpawnModel
from gazebo_msgs.srv import GetModelState, SetLinkState, SetModelState
from gazebo_msgs.msg import ModelState, LinkState
from mavros_msgs.srv import CommandTOL, CommandTOLRequest, CommandBool
from mavros_msgs.msg import State, AttitudeTarget
from .py_gnc_functions import *
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import time
import subprocess
import os
import signal


def main():
    rospy.init_node('main', anonymous=True)
    no_steps = 10
    gnc = py_gnc_functions.gnc_api()
    gnc.wait4connect()
    gnc.wait4start()
    Quad = Train.Duocopter(gnc, mode='train') 
    gnc.initialize_local_frame()
    takeoffHeight = 4
    r = rospy.Rate(100)
    first = True
    while not rospy.is_shutdown():
        print('episode: ', ep)
        if first:
        	gnc.takeoff(takeoffHeight)
        	Quad.waitStartPos(takeoffHeight)
        first = False
        Quad.makeNewTrajetory(start_ep=True)
        while not Quad.isDone:  
            trajSegment = Quad.trajectorySegment()
            Quad.follow_trajectory_segment(trajSegment[0], trajSegment[1], trajSegment[2], trajSegment[3])
            #Quad.segmentReward += Quad.trackingError
            if Quad.mode == 'train':
                Quad.train(Quad.isDone)
            if Quad.isDone:
            	break    
            Quad.makeNewTrajetory(start_ep=False)
            Quad.step += 1
        #Quad.reward.append(Quad.total_reward)
        Quad.reset()



def test_traj():

    rospy.init_node('main', anonymous=True)
    gnc = py_gnc_functions.gnc_api()
    gnc.wait4connect()
    gnc.wait4start()
    Quad = Train.Duocopter(gnc, mode='train') 
    gnc.initialize_local_frame()
    gnc.takeoff(2)
    Quad.waitStartPos(2)
    location = gnc.get_current_location()
    Quad.start_pos = [location.y, location.z]
    Quad.polynomial = interp1d([0,0.15,0.3,0.45,0.6,0.75], [0,0.1,0.3,0.1,0.25,0.3], 'cubic')
    Quad.waypoints = [[0,0], [0.15,0.1], [0.3,0.3], [0.45,0.1], [0.6,0.25], [0.75,0.3]]
    Quad.y = 0
    Quad.z = 0
    Quad.follow_trajectory_segment(0, 0, 0.1, 0.7)
    Quad.follow_trajectory_segment(0, 0, 0.7, 0.2)
    Quad.follow_trajectory_segment(0, 0, 0.2, 0.2)
    Quad.follow_trajectory_segment(0, 0, 0.2, 0.75)
    Quad.follow_trajectory_segment(0, 0, 0.75, 0)
    Quad.reset()
