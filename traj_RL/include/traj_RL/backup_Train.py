#!/usr/bin/env python

from math import pi, atan, atan2, cos, sin, acos, asin, tan, sqrt
from copy import deepcopy
import roslaunch
import rospy
from std_srvs.srv import Empty
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from .Model import *
from .utils import *
from gazebo_msgs.srv import GetLinkState, DeleteModel, SpawnModel
from gazebo_msgs.srv import GetModelState, SetLinkState, SetModelState
from gazebo_msgs.msg import ModelState, LinkState
from mavros_msgs.srv import CommandTOL, CommandTOLRequest, CommandBool
from mavros_msgs.msg import State
#from mavros_msgs.msg import Trajectory
from .py_gnc_functions import *
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import time
import subprocess
import os
import signal

class Duocopter():
    def __init__(self,gnc, action_dim =4,state_dim=4,
                initial_state=None,
                model_path='/home/kobi/catkin_ws/traj_RL/trained_models',
                save_command="on",load_command="off",
                mode='train'):
        
        # states are position (x,y), angle (pitch), velocity (x,y), ang vel (pitch rate), acceleration (x,y)
        print('in Duocopter __init__')
        self.gnc = gnc
        self.current_state_g = State()
        self.listener()
        #self.proc = subprocess.Popen(['gnome-terminal','--command', 'roslaunch iq_sim apm.launch'], preexec_fn=os.setsid)
        time.sleep(3)
        self.payloadMass = 0.01
        # self.gnc = gnc_api()
        # self.gnc.wait4connect()
        # self.gnc.wait4start()
        # self.gnc.initialize_local_frame()
        # print('---------gnc.current_state in __init__-----' ,self.gnc.current_state_g)
        # self.gnc.takeoff(2)

        self.getLinkState = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.branchState = self.getLinkState('branch_new_2m::link_0', 'ground_plane')
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', data_class=Twist, queue_size=5)
        self.attitude_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude', data_class=PoseStamped, queue_size=5)
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', data_class=PoseStamped, queue_size=5)

        # self.state_sub = rospy.Subscriber(
        #     name="{}mavros/state".format(self.ns),
        #     data_class=State,
        #     queue_size=10,
        #     callback=self.state_cb,
        # )
        self.mavros_sub = rospy.Subscriber('/mavros/state', data_class=State, queue_size=2, callback=self.mavros_cb)
        rospy.wait_for_service('/gazebo/get_link_state')

        pos = self.getLinkState('drone_kinect_fixed_rope5::cam_link', 'ground_plane')
        teth = self.getLinkState('drone_kinect_fixed_rope5::link_0', 'world')
        
        #print('---------tether initial orientation------',teth)

        payload = self.getLinkState('drone_kinect_fixed_rope5::link_1', 'ground_plane')
        tetherRoll, pitch = quatToEuler(payload.link_state.pose.orientation.w, payload.link_state.pose.orientation.x,
                                            payload.link_state.pose.orientation.y, payload.link_state.pose.orientation.z)
        self.all_state= np.array([pos.link_state.pose.position.x, pos.link_state.pose.position.z,
                            tetherRoll, 0.0, 0.0, 0.0, payload.link_state.twist.angular.y, 0.0, 0.01], dtype=np.float32)
        # y, z, tether-roll, drone-roll, drone-vel-y, drone-vel-z, tether-angle-dot, drone-angle-dot, branch-radius        
        self.listener()

        self.energy_threshold = self.energy_standard()
        self.energy = 0
        
        # trajectory consists of position (x,y), pitch (radians), velocity_magnitude
        self.desired_trajectory = []
        self.actual_trajectory = []
        self.waypoints = []
        self.velocity_list = []
        self.trackingError_list = []
        self.distane_list = []
        self.reward_without_trackingError = []
        self.reward_with_trackingError = []
        self.waypoints.append([0, 0, self.all_state[3], sqrt(self.all_state[4]**2+self.all_state[5]**2)])
        self.trackingError = 0
        self.segmentReward = [0]
        self.segmentLength = 5
        self.tracking_const = 3
        self.mode = mode
        self.time_elapsed = 0

        self.step = 0
        self.episode_num = 0
        self.ep_reward = [0]
        self.reward = []
        self.isDone = False

        self.last_state = None
        self.last_action = None

        self.action_low = np.array([-pi/2, -0.873, 0,-pi/2, -0.873, 0])
        self.action_high = np.array([pi/2, 0.873, 3, pi/2, 0.873, 3])
        
        self.batch_size=30
        self.save_command=save_command
        self.load_command=load_command
        self.model_path = model_path
        self.model = SAC(action_dim =len(self.action_high),state_dim=len(self.all_state),hidden_dim=256,buffer_size=1000000)
        if load_command == 'on':
            self.model_loader(self.model_path)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.setLinkState = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        self.setModelState = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.landQuad = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self._max_retry = 20


    def reset(self):

        print('in def reset')
        print('\n------self.waypoints--------\n', self.waypoints)
        #print('\n--------self.desired_trajectory----------\n', self.desired_trajectory)
        print('\n---------self.actual_trajectory----------\n', self.actual_trajectory)
        #print('--------self.branchPos------------', self.branchPos)
        

        self.poly = []
        #y = self.waypoints[-1][0] - self.waypoints[0][0]
        y = np.linspace(self.waypoints[0][0], self.waypoints[-1][0], num=100, endpoint=True)
        self.poly = [[i, float(self.polynomial(i))] for i in y]
        print('\n---------self.poly--------------\n', self.poly)
        print('\n----------self.velocity_list---------\n', self.velocity_list)
        print('\n--------self.trackingError_list------\n', self.trackingError_list)
        print('\n---------self.reward_without_trackingError------\n', self.reward_without_trackingError)
        #print('\n---------self.reward_with_trackingError--------\n', self.reward_with_trackingError)
        print('\n--------self.segmentReward-------\n', self.segmentReward)
        self.isDone = False
        self.last_state = None
        self.last_action = None
        self.ep_reward.append(0)
        self.step = 0
        self.energy = 0
        self.desired_trajectory = []
        self.actual_trajectory = []
        self.waypoints = []
        self.velocity_list = []
        self.trackingError_list = []
        self.distane_list = []
        self.reward_without_trackingError = []
        self.reward_with_trackingError = []
        self.segmentReward = [0]
        self.waypoints.append([0, 0, self.all_state[3], sqrt(self.all_state[4]**2+self.all_state[5]**2)])
        self.trackingError = 0
        rospy.wait_for_service('/gazebo/set_link_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        #self.gnc.land()
        linkState = LinkState()
        
        self.stopDrone()


        #time.sleep(2)
        # rospy.wait_for_service('/mavros/cmd/land')
        # rospy.wait_for_service('/gazebo/reset_world')
        # reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        # reset_world()
        # self.land_client = rospy.ServiceProxy("/mavros/cmd/land", service_class=CommandTOL)
        # self.land_client()
        # rospy.sleep(5)

        #self.p_.p.terminate()
        #del self.p_
        #self.proc.kill()
        # self.proc.terminate()
        # print('----close terminal----')
        # time.sleep(2)
        # self.proc = subprocess.Popen(['gnome-terminal', '--command', 'roslaunch iq_sim apm.launch'])
        # time.sleep(3)

        # RUN apm.lunch BUT RUNS IN SAME TERMINAL...
        """
        try:
            self.launch.shutdown()
        except:
            pass
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/kobi/catkin_ws/src/iq_sim/launch/apm.launch"])
        self.launch.start()
        """
        


        # DELETE THEN RESPAWN MODEL:

        # KILL apm.launch kindof...

        #os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        #subprocess.call(['gnome-terminal', '--command', 'killall -11 rosmaster'])

        """
        time.sleep(2)

        rospy.wait_for_service('/gazebo/delete_model')
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model("drone_kinect_fixed_rope5")

        time.sleep(2)

        pos = Pose()
        pos.position.x = 0.62
        pos.position.y = 0.26
        pos.position.z = 0.78
        f = open('/home/kobi/model_editor_models/drone_kinect_fixed_rope5/model.sdf', 'r')
        sdf_ = f.read()
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.spawn_model("drone_kinect_fixed_rope5", sdf_, "robotus_name_space", pos, "world")

        time.sleep(3)
        self.proc = subprocess.Popen(['gnome-terminal', '--command', 'roslaunch iq_sim apm.launch'], preexec_fn=os.setsid)
        time.sleep(3)
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/cmd/takeoff')
        self.arm = rospy.ServiceProxy('/mavros/cmd/arming', service_class=CommandBool)
        
        self.listener()
        if not self.current_state_g.armed:
            arm_request = CommandBoolRequest(True)
            response = self.arm(arm_request)
            self.arm()
            while not self.current_state_g.armed:
                rospy.sleep(0.01)
                self.listener()
        self.takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', service_class=CommandTOL)
        takeoff_srv = CommandTOLRequest(0, 0, 0, 0, 2)
        response = self.takeoff(takeoff_srv)
        time.sleep(3)
        """





        # WAS USING THIS... 
        # self.gnc = gnc_api()
        # self.gnc.wait4connect()
        # self.gnc.wait4start()
        # self.gnc.initialize_local_frame()
        # print('---------gnc.current_state before takeoff-----' ,self.gnc.current_state_g)
        # self.gnc.takeoff(2)

        # self.posPub = rospy.Publisher('/mavros/setpoint_position/local', data_class=PoseStamped, queue_size=5)
        # pos = PoseStamped()
        # pos.header.stamp = rospy.get_rostime()
        # pos.pose.position.x = 0.62
        # pos.pose.position.y = 0.26
        # pos.pose.position.z = 2.78
        # self.posPub.publish(pos)

        #rospy.sleep(3)


        
        print('----------position---------', self.gnc.get_current_location())



        # ONLY RESET MODEL STATE, LEAVE LINKS BUT PAUSE SIM BRIEFLY

        self.gnc.set_destination(0,0,0,0)


        # posit = PoseStamped()
        # posit.header.stamp = rospy.get_rostime()
        # posit.pose.position.x = 0.62
        # posit.pose.position.y = 0.26
        # posit.pose.position.z = 2.78
        # posit.pose.orientation.x = 0
        # posit.pose.orientation.y = 0
        # posit.pose.orientation.z = 0
        # posit.pose.orientation.w = 1
        # self.pos_pub.publish(posit)

        # rospy.sleep(1.5)

        modelState = ModelState()
        modelState.model_name = 'drone_kinect_fixed_rope5'
        modelState.pose.position.x = 0.62
        modelState.pose.position.y = -0.26
        #modelState.pose.position.y = 0.26
        modelState.pose.position.z = 0.78 + 2
        modelState.pose.orientation.x = 0
        modelState.pose.orientation.y = 0
        modelState.pose.orientation.z = 0
        modelState.pose.orientation.w = 1
        modelState.twist.linear.x = 0
        modelState.twist.linear.y = 0
        modelState.twist.linear.z = 0
        modelState.twist.angular.x = 0
        modelState.twist.angular.y = 0
        modelState.twist.angular.z = 0
        # link0State = LinkState()
        # link0State.link_name = 'drone_kinect_fixed_rope5::link_0'
        # link0State.pose.position.x = 0.62
        # link0State.pose.position.y = 0.26
        # link0State.pose.position.z = 0.78 + 2
        # link0State.pose.orientation.x = 0
        # link0State.pose.orientation.y = 0
        # link0State.pose.orientation.z = 0
        # link0State.pose.orientation.w = 1
        # link0State.twist.linear.x = 0
        # link0State.twist.linear.y = 0
        # link0State.twist.linear.z = 0
        # link0State.twist.angular.x = 0
        # link0State.twist.angular.y = 0
        # link0State.twist.angular.z = 0
        # link1State = LinkState()
        # link1State.link_name = 'drone_kinect_fixed_rope5::link_1'
        # link1State.pose.position.x = 0.62
        # link1State.pose.position.y = 0.26
        # link1State.pose.position.z = 0.78 + 2 - 0.3
        # # link1State.pose.orientation.x = 0
        # # link1State.pose.orientation.y = 0
        # # link1State.pose.orientation.z = 0
        # # link1State.pose.orientation.w = 1
        # link1State.twist.linear.x = 0
        # link1State.twist.linear.y = 0
        # link1State.twist.linear.z = 0
        # link1State.twist.angular.x = 0
        # link1State.twist.angular.y = 0
        # link1State.twist.angular.z = 0
        # camLink = LinkState()
        # camLink.link_name = 'drone_kinect_fixed_rope5::cam_link'
        # camLink.pose.orientation.x = 0
        # camLink.pose.orientation.y = 0
        # camLink.pose.orientation.z = 0
        # camLink.pose.orientation.w = 1
        # camLink.twist.linear.x = 0
        # camLink.twist.linear.y = 0
        # camLink.twist.linear.z = 0
        # camLink.twist.angular.x = 0
        # camLink.twist.angular.y = 0
        # camLink.twist.angular.z = 0
        # self.setLinkState(link0State)
        #self.setLinkState(link1State)
        self.setModelState(modelState)
        # #self.setLinkState(camLink)
        # self.posPub = rospy.Publisher('/mavros/setpoint_position/local', data_class=PoseStamped, queue_size=5)
        # pos = PoseStamped()
        # pos.header.stamp = rospy.get_rostime()
        # pos.pose.position.x = 0.62
        # pos.pose.position.y = 0.26
        # pos.pose.position.z = 2.78
        # self.posPub.publish(pos)
        # self.unpause()
        # rospy.sleep(3)
        

        #self.gnc.set_destination(0,0,0,0)
        rospy.sleep(10)
        print('----------position reset------------')

        # x = self.gnc.get_current_location().x
        # y = self.gnc.get_current_location().y
        # while not (abs(x-0.62) < 0.2) and not (abs(y-0.26) < 0.2):
        #     x = self.gnc.get_current_location().x
        #     y = self.gnc.get_current_location().y
        #     rospy.sleep(0.01)

        #print('-------reset location------- (x,y) ', x, ' ', y)
        print('---------start reached------------')
        


        #time.sleep(2)
        #rospy.wait_for_service('/gazebo/unpause_physics')
        #self.unpause()
        #self.gnc.initialize_local_frame()
        #self.gnc.set_destination(0,0,-2,0)
        #self.waitStartPos(2)
        pos = self.getLinkState('drone_kinect_fixed_rope5::cam_link', 'ground_plane')
        payload = self.getLinkState('drone_kinect_fixed_rope5::link_1', 'ground_plane')
        tetherRoll, pitch = quatToEuler(payload.link_state.pose.orientation.w, payload.link_state.pose.orientation.x,
                                            payload.link_state.pose.orientation.y, payload.link_state.pose.orientation.z)
        self.all_state= np.array([pos.link_state.pose.position.x, pos.link_state.pose.position.z,
                            tetherRoll, 0.0, 0.0, 0.0, payload.link_state.twist.angular.y, 0.0, 0.01], dtype=np.float32)
        # y, z, tether-roll, drone-roll, drone-vel-y, drone-vel-z, tether-angle-dot, drone-angle-dot, branch-radius        
        self.listener()


    def pauseSim(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        paused_done = False
        counter = 0
        while not paused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    self.pause()
                    paused_done = True
                except rospy.ServiceException as e:
                    counter += 1
            else:
                error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo pause service"
                rospy.logerr(error_message)
                assert False, error_message


    def unpauseSim(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        unpaused_done = False
        counter = 0
        while not unpaused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    self.unpause()
                    unpaused_done = True
                except rospy.ServiceException as e:
                    counter += 1
            else:
                error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo unpause service"
                rospy.logerr(error_message)
                assert False, error_message


    def waitStartPos(self, takeoffHeight):
        print('in waitStartPos')
        location = self.gnc.get_current_location()
        z = location.z
        y = location.y
        x = location.x
        r = rospy.Rate(3)
        while (z<takeoffHeight-0.1 or z>takeoffHeight+0.1) and (x<0.5 or x>0.9) and (y<-0.2 or y>0.2):
            #self.listener()
            r.sleep()
            location = self.gnc.get_current_location()
            print('--------location---------', location)
            z = location.z
            y = location.y
            x = location.x
        self.gnc.initialize_local_frame()


    def makeNewTrajetory(self, start_ep):
        #print('=== in makeNewTrajetory === start_ep: ', start_ep)
        self.listener()
        #self.start_pos = # get model state? but won't same as imu...
        location = self.gnc.get_current_location()
        if start_ep:
            self.start_pos = []
            self.start_pos.append(location.y)
            self.start_pos.append(location.z)
            self.y = 0
            self.z = 0

        #state = np.array([location.x, location.z, self.all_state[3], sqrt(self.all_state[4]**2 + self.all_state[5]**2)]) # x, z, drone-roll, drone-speed
        self.last_state = self.all_state
        if self.mode == 'train':
            action = np.array(self.model.action(self.all_state)) 
            waypoint =self.scaleAction(action)
            self.waypoints.append([self.waypoints[-1][0]+waypoint[0], self.waypoints[-1][1]+waypoint[1],
                                    waypoint[2], waypoint[3]])
            self.waypoints.append([self.waypoints[-1][0]+waypoint[4], self.waypoints[-1][1]+waypoint[5],
                                    waypoint[6], waypoint[7]])


            #self.waypoints.append(waypoint[0:4])
            #self.waypoints.append(waypoint[4:8])
        elif mode == 'test':
            action = np.array(self.model.determinant_action(self.all_state))    # NEED TO ADD DETERMINANT TRAJECTORY GENERATION
            waypoint =self.scaleAction(action)                      # WHERE THE NEXT STATE IS ASSUMED TO BE GIVEN BY PREDICTED STATE > MAKE ALL WAYPOINTS
            self.waypoints.append(waypoint[0:4])
            self.waypoints.append(waypoint[4:8])
        #print('--------------wayPointList: --------------- ', self.waypoints)
        self.approximateTrajectoryFunction()


    def approximateTrajectoryFunction(self):

        #print('--------waypoints------', self.waypoints)
        y = np.array([i[0] for i in self.waypoints])
        z = np.array([i[1] for i in self.waypoints])
        if len(self.waypoints) > 3:
            #print('-------y--------',y)
            #print('-------z--------',z)
            self.polynomial = interp1d(y, z, kind='cubic')
        else:
            self.polynomial = interp1d(y, z, kind='quadratic')
        # print('---self.waypoints---', self.waypoints)
        # print('---min of poly---', y[0])
        # print('---max of poly---', y[-1])
       


        """
        if len(self.waypoints) <= 8:
            a = np.polyfit(y, z, deg=len(self.waypoints)-1)
        else:
            a = np.polyfit(y, z, deg=7)
        self.polynomial = np.poly1d(a)
        """

        self.updateState()

        """
        traj = [[self.all_state[0], self.all_state[1], self.all_state[3], sqrt(self.all_state[4]**2 + self.all_state[5]**2)],
                self.waypoints[-2], self.waypoints[-1]]
        y_diff = (self.waypoints[-1][0] - self.waypoints[0][0])/(len(self.waypoints)*self.segmentLength)
        for j in range(len(traj)-1):
            start_y = self.waypoints[-j-1][0] + y_diff
            for i in range(4):
                traj.insert(-2+j, [self.polynomial(start_y), start_y, None, None])
                start_y += y_diff
        """

        #traj = np.array([])
        traj = []
        for index, i in enumerate(self.waypoints):
            #traj = np.append(traj, i)
            traj.append(i)
            #print('-------traj-------',traj)
            if index <= len(self.waypoints)-2:
                #temp = np.linspace(i[0], self.waypoints[index+1][0], num=4, endpoint=False)
                temp = []
                dif = (self.waypoints[index+1][0] - i[0])/5
                for k in range(1, 5):
                    temp.append(i[0] + (dif*k))

                #print('--------temp-----------',temp)
                for j in temp:
                    traj.append([j, float(self.polynomial(j)), None, None])
                    #traj = np.append(traj,[[j, self.polynomial(j), None, None] for j in temp])

        self.desired_trajectory = traj
        #print('-----approxtraj self.desired_trajectory--------', self.desired_trajectory)


    def trajectorySegment(self): # interpolate speed & roll
        #print('in trajectorySegment')
        #print('---------desired_trajectory--------', self.desired_trajectory)
        startPitch = self.desired_trajectory[0][2]
        endPitch = self.desired_trajectory[-1][2]
        startVel = self.desired_trajectory[0][3]
        endVel = self.desired_trajectory[-1][3]
        return (startPitch, endPitch, startVel, endVel)


    def follow_trajectory_segment(self, startPitch, endPitch, startVel, endVel):

        #print('--------desired trajectory is-------------', self.desired_trajectory)
        r = rospy.Rate(self.segmentLength*6)
        trackingError = 0

        """
        posList = [(i[0],i[1]) for i in self.desired_trajectory[0:self.segmentLength]]
        pitchList = []
        for i in range(self.segmentLength):
            pitchList.append(startPitch + i*(endPitch-startPitch)/self.segmentLength)
        speedList = []
        for i in range(self.segmentLength):
            speedList.append(endVel + i*(endVel-startVel)/self.segmentLength)
        prevZ = self.desired_trajectory[-1][1]
        count = 1
        self.segmentReward = 0
        start = self.getLinkState('drone_kinect_fixed_rope5::cam_link', 'ground_plane')
        startCoords = [start.link_state.pose.position.x, start.link_state.pose.position.y, start.link_state.pose.position.z+0.192] 
        """

        # 0.192 IS THERE BECAUSE cam_link IS THAT MUCH BELOW CENTRE OF DRONE --->                               ^^^^^^^^^^^^^^^^^ 
        #( SAME BELOW )
        #print('------------in follow traj segment---------')

        #start_pos = self.gnc.get_current_location()
        end_y = self.waypoints[-1][0]
        end_z = self.waypoints[-1][1]
        delta_y = 0.02
        count = 0
        self.segmentReward.append(0)
        trackingError = 0
        instant_track_error = 0
        self.listener()
        while not self.y >= end_y and not rospy.is_shutdown():
            #print('----count----', count)
            if instant_track_error>0.1:
                var = 0.1
            else:
                var = instant_track_error

            if self.y + 2*delta_y >= end_y:
                break
            if self.y <= 0:
                self.y = 0

            instant_track_error = abs(self.z-self.polynomial(self.y))
            trackingError += instant_track_error
            self.trackingError_list.append(trackingError)
            print('-----self.y----', self.y)
            try:
                delta_z = self.polynomial(self.y + 2*delta_y) - self.z #self.polynomial(self.y+delta_y) 
            except:
                print('---self.y + delta_y is too big or self.y is too small, calling reset---')
                self.reset()
            print('---delta_y---', delta_y)
            print('---delta_z----', delta_z)
            unit_vec = np.array([2*delta_y/sqrt((2*delta_y)**2 + delta_z**2), delta_z/sqrt((2*delta_y)**2 + delta_z**2)])
            
            angle = self.all_state[3]
            print('-----current angle----', angle)
            print('----instant_track_error----', instant_track_error)
            unit_vec = np.array([cos(angle)*unit_vec[0] - sin(angle)*unit_vec[1], 
                                sin(angle)*unit_vec[0] + cos(angle)*unit_vec[1]])

            print('---unit_vec---', unit_vec)
            speed = startVel + ((self.y - self.start_pos[0])/(end_y - self.start_pos[0]))*(endVel - startVel)
            velocity = unit_vec * speed
            self.velocity_list.append(list(velocity))
            vel = Twist()
            #vel.header.stamp = rospy.Time.now()
            #vel.header.frame_id = "base_link"
            vel.linear.x = 0
            vel.linear.y = velocity[0]
            vel.linear.z = velocity[1]
            self.vel_pub.publish(vel)
            r.sleep()
            pos = self.gnc.get_current_location()
            self.y = pos.y - self.start_pos[0]
            self.z = pos.z - self.start_pos[1]
            self.actual_trajectory.append([self.y,self.z])
            self.observer()
            if self.isDone:
                break
        self.reward_without_trackingError.append(self.segmentReward[-1])
        self.segmentReward[-1] -= self.tracking_const * trackingError
        self.ep_reward[-1] += self.segmentReward[-1]
        self.trackingError += trackingError
        if self.step > 13:
            self.isDone = True
        self.buffer_push(self.isDone)


        """
        while not rospy.is_shutdown() and count < self.segmentLength:
            #print('-----------in while loop in folllow traj------------')
            #print('-----------count---------', count)
            #print('-----------posList----------', posList)
            #print('----------posList[count]-------', posList[count])
            y_diff = posList[count][0] - posList[count-1][0]
            z_diff = posList[count][1] - posList[count-1][1]
            unitVec = np.array([(abs(y_diff)/sqrt(y_diff**2+z_diff**2)) , (abs(z_diff)/sqrt(y_diff**2+z_diff**2))])
            velocity = speedList[count-1] * unitVec
            self.velocity_list.append(list(velocity))

            vel = Twist()
            pose = PoseStamped()
            vel.linear.x = 0
            vel.linear.y = velocity[0]
            vel.linear.z = velocity[1]
            #print('------unit vector-------', unitVec)
            #print('------published vel------', vel)
            self.vel_pub.publish(vel)
            x,y,z,w = eulerToQuat(pitchList[count-1], 0, 0) # unsure if it should be pitch or roll ...
            pose.header.stamp = rospy.get_rostime()

            # NEED TO ALSO SET POSITION OF WAYPOINT IN POSE ...
            # --->

            pose.pose.orientation.x = x
            pose.pose.orientation.y = y
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w
            self.attitude_pub.publish(pose)
            prevZ = posList[count][1]
            r.sleep()
            location = self.getLinkState('drone_kinect_fixed_rope5::cam_link', 'ground_plane')
            self.actual_trajectory.append([location.link_state.pose.position.x, location.link_state.pose.position.z+0.192])
            #print('--------distance to current trajectory waypoint---------------', (abs(location.link_state.pose.position.x - startCoords[0] * (count -1)) + abs(location.link_state.pose.position.z + 0.192 - self.desired_trajectory[count+1][1])))
            trackingError += (abs(location.link_state.pose.position.x - startCoords[0] * (count -1)) + abs(location.link_state.pose.position.z + 0.192 - self.desired_trajectory[count+1][1]))
            reward = self.observer()
            #print('----isDone----', self.isDone)
            if self.isDone:
                break
            count += 1
        self.trackingError = trackingError
        self.buffer_push(self.isDone)
        """

    def stopDrone(self):
        # self.gnc.set_destination(0,0,0,0)
        # rospy.sleep(0.3)
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        self.vel_pub.publish(vel)
        rospy.sleep(2)


    def scaleAction(self, action):
        self.last_action = action
        # self.action_high = np.array([pi, 0.873, 3, pi, 0.873, 3])
        action_ = [action[0]*self.action_high[0], action[1]*self.action_high[1],
                    (action[2]+1)*self.action_high[2]/2, action[3]*self.action_high[3],
                    action[4]*self.action_high[4], (action[5]+1)*self.action_high[5]/2]



        #action = deepcopy(np.array(action)+1)*self.action_high/2.0
        waypntDis = 0.15
        a = [waypntDis*cos(action_[0]), waypntDis*sin(action_[0]), action_[1], action_[2],
            waypntDis*cos(action_[3]), waypntDis*sin(action_[3]), action_[4], action_[5]]
        return a


    def train(self, done):
        if len(self.model.replay_buffer) > self.batch_size:
            self.model.soft_q_update(self.batch_size, done, value_lr  = 0.5e-4, q_lr = 0.5e-4, policy_lr = 0.5e-4)


    def buffer_push(self, done):
        if self.last_state is not None and self.last_action is not None:
            #print('---current segment reward---', self.segmentReward)
            self.model.replay_buffer.push(self.last_state, self.last_action, self.segmentReward[-1], self.all_state, done)


    def energy_standard(self, x = 2.0, y = 2.0):
        self.L = 0.3
        g = 9.81
        target_x = x
        target_y = y
        tree_x = 3.0
        tree_y = 1.0
        r = self.all_state[-1]
        target_angle = atan((tree_x-target_x)/(target_y-tree_y))-asin(r/sqrt((target_x-tree_x)**2+(target_y-tree_y)**2))#np.pi/2
        L1 = self.L - sqrt((target_x-tree_x)**2+(target_y-tree_y)**2-r**2)
        L2 = 0.7*L1
        temp_L2 = 0
        while abs(L2 - temp_L2) > 0.00001:
            temp_L2=L2
            L2 = L1-(atan(r/temp_L2)-target_angle+np.pi)*r;
        target_velocity = sqrt(2*98.1*(sqrt(L2**2+r**2)+L1*cos(target_angle)+r*sin(target_angle)))
        target_thetad = target_velocity/self.L
        energy_threshold=((1-cos(target_angle))*g*self.L+0.5*target_velocity**2)*0.02
        return energy_threshold


    def observer(self):
        #print('----------in def observer------------')
        payloadState = self.getLinkState('drone_kinect_fixed_rope5::link_1', 'ground_plane')
        self.all_state[6] = payloadState.link_state.twist.angular.y
        tetherQuat = payloadState.link_state.pose.orientation # roll is what is needed
        roll, pitch = quatToEuler(tetherQuat.w, tetherQuat.x, tetherQuat.y, tetherQuat.z)
        self.listener()
        velMagn = sqrt(payloadState.link_state.twist.linear.x**2 + payloadState.link_state.twist.linear.z**2)
        self.energy = 0.5*self.payloadMass*velMagn**2 + self.payloadMass*9.81*payloadState.link_state.pose.position.z
        self.updateState()
        # y, z, tether-roll, drone-roll, drone-vel-y, drone-vel-z, tether-angle-dot, drone-angle-dot, branch-radius
        reward = 0
        Distance = self.distance()
        self.distane_list.append(Distance)
        reward -= abs(Distance-(self.all_state[-1]-0.1))*0.3
        reward += self.energy

        
        if self.current_state_g.system_status in range(5,9):
            self.isDone = True
            self.segmentReward[-1] += float(reward)
            #self.reward_without_trackingError.append(reward)
            self.stopDrone()
            return reward
        if Distance <= self.all_state[-1] + 0.05:
            print("Attached")
            reward += 100
            standard = self.energy_standard(x = self.all_state[0], y = self.all_state[1])
            if self.energy >= standard:
                print("sufficient energy for perching %.3f/%.3f" %(self.energy,standard))
                if self.mode == "train":
                    self.model_saver(self.model_path)
                reward += 400
                self.isDone = True
            else :
                print("not sufficient energy %.3f/%.3f" %(self.energy,standard))
                self.isDone = True
        self.segmentReward[-1] += float(reward)
        #self.reward_without_trackingError.append(reward)
        #return reward


    def distance(self):
        
        #tetherState = self.getLinkState('drone_kinect_fixed_rope2::link_0', 'ground_plane')
        #distance = sqrt((branchState.y-tetherState.y)**2 + (branchState.z-tetherState.z)**2)
        g = 9.81
        state = self.all_state
        P1 = np.array([state[0] + 0.3 * np.sin(state[2]),state[1] - 0.3 * np.cos(state[2])])
        P2 = np.array([state[0] + (self.L-2*state[-1]*pi) * np.sin(state[2]),state[1] - (self.L-2*state[-1]*pi) * np.cos(state[2])])
        P0 = np.array([self.branchState.link_state.pose.position.y ,self.branchState.link_state.pose.position.z]) 
        self.branchPos = P0
        #      POSITION RELATIVE TO GROUND_PLANE    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        vec1 = P2-P1
        vec2 = P0-P1
        angle1= np.arccos(np.sum(vec1*vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
        vec3 = P1-P2
        vec4 = P0-P2
        angle2= np.arccos(np.sum(vec3*vec4)/(np.linalg.norm(vec3)*np.linalg.norm(vec4)))
        distance = np.linalg.norm(np.cross(vec1,vec2))/np.linalg.norm(vec1)
        if angle1 >= pi/2 or angle1 <= -pi*3/2:
            distance = np.linalg.norm(vec2)
        elif angle2 >= pi/2 or angle2 <= -pi*3/2 :
            distance = np.linalg.norm(vec4)
            #print('--------distance drone to branch------------', distance)
        return distance


    def done(self):
        self.episode_num += 1
        print('\ndone episode', self.episode_num, 'total_reward is :', self.ep_reward[-1])
        # x = []
        # z = []
        # for i in range(0, len(self.actual_trajectory), 10):
        #     x.append(self.actual_trajectory[i][0])
        #     z.append(self.actual_trajectory[i][1])
        # plt.plot(x,z)
        # plt.show()
        self.reset()


    def updateState(self):
        pos = self.getLinkState('drone_kinect_fixed_rope5::cam_link', 'ground_plane')
        payload = self.getLinkState('drone_kinect_fixed_rope5::link_1', 'ground_plane')
        tetherRoll, pitch = quatToEuler(payload.link_state.pose.orientation.w, payload.link_state.pose.orientation.x, 
                                    payload.link_state.pose.orientation.y, payload.link_state.pose.orientation.z)
        self.all_state= np.array([pos.link_state.pose.position.x, pos.link_state.pose.position.z,
                                    tetherRoll, 0.0, 0.0, 0.0, payload.link_state.twist.angular.y, 0.0, 0.01], dtype=np.float32)
        self.listener()


    def listener(self):
        #rospy.init_node('listener', anonymous=True)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imuCallback)
        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self.velCallack)
        self.mavros_sub = rospy.Subscriber('/mavros/state', data_class=State, queue_size=2, callback=self.mavros_cb)


    def imuCallback(self, data):
        roll, pitch = quatToEuler(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
        self.all_state[3] = roll


    def velCallack(self, data):
        self.all_state[4] = data.twist.linear.y
        self.all_state[5] = data.twist.linear.z
        self.all_state[7] = data.twist.angular.x


    def mavros_cb(self, data):
        self.current_state_g = data


    def model_saver(self,model_path):
        if self.save_command=="on":
            torch.save(self.model.value_net.state_dict(), model_path+"value_21th_aug.dat")
            torch.save(self.model.target_value_net.state_dict(), model_path+"target_21th_aug.dat")
            torch.save(self.model.q_net.state_dict(), model_path+"qvalue_21th_aug.dat")
            torch.save(self.model.policy_net.state_dict(), model_path+"policy_21th_aug.dat")
            print('save model sucessfully')


    def model_loader(self,model_path):
        path = model_path
        print('MODEL_LOADER CALLED')
        if os.path.exists(model_path+"policy.dat") and self.load_command=="on":
            print("PATH EXISTS")
            self.model.value_net.load_state_dict(torch.load(model_path+"value_21th_aug.dat"))
            self.model.value_net.eval()
            self.model.target_value_net.load_state_dict(torch.load(model_path+"target_21th_aug.dat"))
            self.model.target_value_net.eval()
            self.model.q_net.load_state_dict(torch.load(model_path+"qvalue_21th_aug.dat"))
            self.model.q_net.eval()
            self.model.policy_net.load_state_dict(torch.load(model_path+"policy_21th_aug.dat"))
            self.model.policy_net.eval()
            self.model.policy_net_local.load_state_dict(torch.load(model_path+"policy_21th_aug.dat"))
            self.model.policy_net_local.eval()
            print('load model sucessfully')