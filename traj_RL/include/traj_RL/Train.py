
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
                model_path='/home/kobi/catkin_ws/src/traj_RL/trained_models/',
                save_command="on",load_command="on",
                mode='train'):
        
        # states are position (x,y), angle (pitch), velocity (x,y), ang vel (pitch rate), acceleration (x,y)
        print('in Duocopter __init__')
        self.gnc = gnc
        self.current_state_g = State()
        self.listener()
        time.sleep(3)
        self.payloadMass = 0.01
        self.getLinkState = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.getModelState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.branchState = self.getModelState('branch_new_2m', 'world')
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', data_class=Twist, queue_size=5)
        self.attitude_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude', data_class=PoseStamped, queue_size=5)
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', data_class=PoseStamped, queue_size=5)
        self.mavros_sub = rospy.Subscriber('/mavros/state', data_class=State, queue_size=2, callback=self.mavros_cb)
        rospy.wait_for_service('/gazebo/get_link_state')

        pos = self.getLinkState('drone_kinect_fixed_rope5::cam_link', 'ground_plane')
        teth = self.getLinkState('drone_kinect_fixed_rope5::link_0', 'world')
        
        #print('---------tether initial orientation------',teth)

        payload = self.getLinkState('drone_kinect_fixed_rope5::link_1', 'ground_plane')
        tetherRoll, pitch = quatToEuler(payload.link_state.pose.orientation.w, payload.link_state.pose.orientation.x,
                                            payload.link_state.pose.orientation.y, payload.link_state.pose.orientation.z)
        

        posit = self.getModelState('drone_kinect_fixed_rope5', 'world')
        pos_x = self.branchState.pose.position.x - posit.pose.position.x
        pos_z = self.branchState.pose.position.z - posit.pose.position.z
        self.all_state= np.array([pos_x, pos_z,
                            tetherRoll, 0.0, 0.0, 0.0, payload.link_state.twist.angular.y, 0.0, 0.01], dtype=np.float32)
        # x distance to branch, z distance to branch, tether-roll, drone-roll, drone-vel-y, drone-vel-z, tether-angle-dot, drone-angle-dot, branch-radius        
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
        self.step = 0
        self.episode_num = 0
        self.ep_reward = [0]
        self.reward = []
        self.isDone = False
        self.last_state = None
        self.last_action = None
        self.action_low = np.array([-pi/2, -0.873, 0,-pi/2, -0.873, 0])
        self.action_high = np.array([pi/2, 0.873, 2, pi/2, 0.873, 2])
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


    def reset(self, global_start):

        print('in def reset')
        print('\n------self.waypoints--------\n', self.waypoints)
        #print('\n--------self.desired_trajectory----------\n', self.desired_trajectory)
        print('\n---------self.actual_trajectory----------\n', self.actual_trajectory)
        #print('--------self.branchPos------------', self.branchPos)
        
        self.poly = []
        y = np.linspace(self.waypoints[0][0], self.waypoints[-1][0], num=100, endpoint=True)
        self.poly = [[i, float(self.polynomial(i))] for i in y]
        print('\n---------self.poly--------------\n', self.poly)
        print('\n----------self.velocity_list---------\n', self.velocity_list)
        #print('\n--------self.trackingError_list------\n', self.trackingError_list)
        print('\n---------self.reward_without_trackingError------\n', self.reward_without_trackingError)
        print('\n--------self.segmentReward-------\n', self.segmentReward)
        rel_branch_x = self.branchState.pose.position.x - global_start.pose.position.x
        rel_branch_z = self.branchState.pose.position.z - global_start.pose.position.z
        print('----relative branch pos (x,y)------', rel_branch_x, rel_branch_z)
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
        self.global_start = global_start
        rospy.wait_for_service('/gazebo/set_link_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        #self.gnc.land()
        linkState = LinkState()
        
        self.stopDrone()

        print('----------position---------', self.gnc.get_current_location())
        self.gnc.set_destination(0,0,1.35,0)
        modelState = ModelState()
        modelState.model_name = 'drone_kinect_fixed_rope5'
        """
        modelState.pose.position.x = 0.62
        modelState.pose.position.y = 0.5
        #modelState.pose.position.y = -0.26
        #modelState.pose.position.y = 0.26
        modelState.pose.position.z = 0.78 + 4
        """
        modelState.pose.position.x = global_start.pose.position.x
        modelState.pose.position.y = global_start.pose.position.y
        modelState.pose.position.z = global_start.pose.position.z
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


        # ===============================================================================

        #######  --------->>>>> backup_Train.py CONTAINS NEW CODE <<<<<--------------

        # ===============================================================================
        # ===============================================================================

        #######  --------->>>>> backup_Train.py CONTAINS NEW CODE <<<<<--------------

        # ===============================================================================




        self.setModelState(modelState)
        rospy.sleep(8)

        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        self.vel_pub.publish(vel)
        rospy.sleep(2)

        #loc = self.gnc.get_current_location()
        #self.gnc.set_destination(loc.x, loc.y, loc.z, 0)



        print('----------position reset------------')
        print('---------start reached------------')
        pos = self.getLinkState('drone_kinect_fixed_rope5::cam_link', 'ground_plane')
        payload = self.getLinkState('drone_kinect_fixed_rope5::link_1', 'ground_plane')
        tetherRoll, pitch = quatToEuler(payload.link_state.pose.orientation.w, payload.link_state.pose.orientation.x,
                                            payload.link_state.pose.orientation.y, payload.link_state.pose.orientation.z)
        
        posit = self.getModelState('drone_kinect_fixed_rope5', 'world')
        pos_x = self.branchState.pose.position.x - posit.pose.position.x
        pos_z = self.branchState.pose.position.z - posit.pose.position.z
        self.all_state= np.array([pos_x, pos_z,
                            tetherRoll, 0.0, 0.0, 0.0, payload.link_state.twist.angular.y, 0.0, 0.01], dtype=np.float32)
        # x distance to drone, z distance to drone, tether-roll, drone-roll, drone-vel-y, drone-vel-z, tether-angle-dot, drone-angle-dot, branch-radius        
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
        drone_start = self.getModelState('drone_kinect_fixed_rope5', 'world')
        self.global_start = drone_start
        return drone_start


    def makeNewTrajetory(self, start_ep):
        print('=== in makeNewTrajetory === start_ep: ', start_ep)
        self.listener()
        #self.start_pos = # get model state? but won't same as imu...
        location = self.gnc.get_current_location()
        if start_ep:
            self.start_pos = []
            self.start_pos.append(location.y)
            self.start_pos.append(location.z)
            self.y = 0
            self.z = 0

        self.last_state = self.all_state
        if self.mode == 'train':
            action = np.array(self.model.action(self.all_state)) 
            waypoint =self.scaleAction(action)
            self.waypoints.append([self.waypoints[-1][0]+waypoint[0], self.waypoints[-1][1]+waypoint[1],
                                    waypoint[2], waypoint[3]])
            self.waypoints.append([self.waypoints[-1][0]+waypoint[4], self.waypoints[-1][1]+waypoint[5],
                                    waypoint[6], waypoint[7]])

        elif mode == 'test':
            action = np.array(self.model.determinant_action(self.all_state))    # NEED TO ADD DETERMINANT TRAJECTORY GENERATION
            waypoint =self.scaleAction(action)                      # WHERE THE NEXT STATE IS ASSUMED TO BE GIVEN BY PREDICTED STATE > MAKE ALL WAYPOINTS
            self.waypoints.append(waypoint[0:4])
            self.waypoints.append(waypoint[4:8])
        self.approximateTrajectoryFunction()


    def approximateTrajectoryFunction(self):

        print('---in def approximateTrajectoryFunction----')
        y = np.array([i[0] for i in self.waypoints])
        z = np.array([i[1] for i in self.waypoints])
        if len(self.waypoints) > 3:
            #print('-------y--------',y)
            #print('-------z--------',z)
            self.polynomial = interp1d(y, z, kind='cubic')
        else:
            self.polynomial = interp1d(y, z, kind='quadratic')

        self.updateState()

        traj = []
        for index, i in enumerate(self.waypoints):
            traj.append(i)
            #print('-------traj-------',traj)
            if index <= len(self.waypoints)-2:
                temp = []
                dif = (self.waypoints[index+1][0] - i[0])/5
                for k in range(1, 5):
                    temp.append(i[0] + (dif*k))

                for j in temp:
                    traj.append([j, float(self.polynomial(j)), None, None])

        self.desired_trajectory = traj


    def trajectorySegment(self): # interpolate speed & roll
        print('----in trajectorySegment----')
        pnts_per_waypnt = 5
        wypnts_per_actions = 2
        startPitch = self.desired_trajectory[-pnts_per_waypnt*wypnts_per_actions-1][2]
        endPitch = self.desired_trajectory[-1][2]
        startVel = self.desired_trajectory[-pnts_per_waypnt*wypnts_per_actions-1][3]
        endVel = self.desired_trajectory[-1][3]
        return (startPitch, endPitch, startVel, endVel)


    def follow_trajectory_segment(self, startPitch, endPitch, startVel, endVel):

        print('----in follow_trajectory_segment-----')
        r = rospy.Rate(self.segmentLength*6)
        trackingError = 0


        # 0.192 IS THERE BECAUSE cam_link IS THAT MUCH BELOW CENTRE OF DRONE --->                               ^^^^^^^^^^^^^^^^^ 
        end_y = self.waypoints[-1][0]
        end_z = self.waypoints[-1][1]
        #delta_y = 0.02
        delta_y = 0.04
        count = 0
        self.segmentReward.append(0)
        trackingError = 0
        instant_track_error = 0
        self.listener()
        segment_start_y = self.y
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
            #print('-----self.y----', self.y)
            try:
                delta_z = self.polynomial(self.y + 2*delta_y) - self.z #self.polynomial(self.y+delta_y) 
            except:
                print('---self.y + delta_y is too big or self.y is too small, calling reset---')
                self.reset()
            #print('---delta_y---', delta_y)
            #print('---delta_z----', delta_z)
            unit_vec = np.array([2*delta_y/sqrt((2*delta_y)**2 + delta_z**2), delta_z/sqrt((2*delta_y)**2 + delta_z**2)])
            angle = self.all_state[3]
            #unit_vec = np.array([cos(angle)*unit_vec[0] - sin(angle)*unit_vec[1], 
            #                    sin(angle)*unit_vec[0] + cos(angle)*unit_vec[1]])

            #print('--- unit_vec ----', unit_vec)
            #print('--- startVel ---- ', startVel)
            #print('---- endVel -----', endVel)

            speed = startVel + ((self.y - segment_start_y)/ (end_y - segment_start_y))*(endVel - startVel)
            #speed = startVel + ((self.y - self.start_pos[0])/(end_y - self.start_pos[0]))*(endVel - startVel)
            if speed < 0.1:
                speed = 0.1
            #print(' ---- endy_y ----', end_y)
            #print(' ----- self.y ----', self.y)
            #print(' ---- segment_start_y --- ', segment_start_y)
            #print('---- interpolated speed ----', speed)

            velocity = unit_vec * speed

            self.velocity_list.append(list(velocity))
            vel = Twist()
            vel.linear.x = 0
            vel.linear.y = velocity[0]
            vel.linear.z = velocity[1]
            self.vel_pub.publish(vel)
            r.sleep()
            pos = self.gnc.get_current_location()
            branch_y = self.branchState.pose.position.x
            branch_z = self.branchState.pose.position.z


        # ===============================================================================

        #######  --------->>>>> backup_Train.py CONTAINS NEW CODE <<<<<--------------

        # ===============================================================================


        # ===============================================================================

        #######  --------->>>>> backup_Train.py CONTAINS NEW CODE <<<<<--------------

        # ===============================================================================

        # ===============================================================================

        #######  --------->>>>> backup_Train.py CONTAINS NEW CODE <<<<<--------------

        # ===============================================================================
        # ===============================================================================

        #######  --------->>>>> backup_Train.py CONTAINS NEW CODE <<<<<--------------

        # ===============================================================================

        # ===============================================================================

        #######  --------->>>>> backup_Train.py CONTAINS NEW CODE <<<<<--------------

        # ===============================================================================
        # ===============================================================================

        #######  --------->>>>> backup_Train.py CONTAINS NEW CODE <<<<<--------------

        # ===============================================================================

        # ===============================================================================

        #######  --------->>>>> backup_Train.py CONTAINS NEW CODE <<<<<--------------

        # ===============================================================================
        # ===============================================================================

        #######  --------->>>>> backup_Train.py CONTAINS NEW CODE <<<<<--------------

        # ===============================================================================


            self.y = pos.y - self.start_pos[0]
            self.z = pos.z - self.start_pos[1]
            self.actual_trajectory.append([self.y,self.z])
            self.observer()
            if self.z < -2.5 or self.z > 2.5:
                self.isDone = True
                break

            #print('------self.all_state----', list(self.all_state))
            #print('----branch location (x,z) ----  ', self.branchState.pose.position.x, self.branchState.pose.position.z)
            #print('----self.pos-----', pos)
            if self.isDone:
                break
        self.reward_without_trackingError.append(self.segmentReward[-1])
        self.segmentReward[-1] -= self.tracking_const * trackingError
        self.ep_reward[-1] += self.segmentReward[-1]
        self.trackingError += trackingError
        if self.step > 13:
            self.isDone = True
        self.buffer_push(self.isDone)



    def stopDrone(self):
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        self.vel_pub.publish(vel)
        rospy.sleep(2)


    def scaleAction(self, action):
        # self.action_high = np.array([pi/2, 0.873, 2, pi/2, 0.873, 2])
        self.last_action = action
        action_ = [action[0]*self.action_high[0], action[1]*self.action_high[1],
                    (action[2]+1)*self.action_high[2]/2, action[3]*self.action_high[3],
                    action[4]*self.action_high[4], (action[5]+1)*self.action_high[5]/2]

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


    def energy_standard(self, x=None, y=None):
        L = 0.3
        g = 9.81
        tree_x = self.branchState.pose.position.x
        tree_y = self.branchState.pose.position.z
        rospy.wait_for_service('/gazebo/get_model_state')
        pos = self.getModelState('drone_kinect_fixed_rope5', 'world')
        target_x = pos.pose.position.x
        target_y = pos.pose.position.z
        """
        target_x = x
        target_y = y
        tree_x = 3.0
        tree_y = 1.0
        """

        r = self.all_state[-1]
        try:
            target_angle = atan((tree_x-target_x)/(target_y-tree_y))-asin(r/sqrt((target_x-tree_x)**2+(target_y-tree_y)**2))#np.pi/2
        except:
            print('=== TARGET ANGLE FAILED, THESE ARE THE VARIABLES: ===')  
            print(' tree_x: ', tree_x, '   tree_y: ', tree_y, '   target_x: ', target_x, '   target_y: ', target_y, '   r: ', r)
            print(1/0)  # <-- TO BREAK THE PROGRAM, SEE ABOVE
        L1 = L - sqrt((target_x-tree_x)**2+(target_y-tree_y)**2-r**2)
        L2 = 0.7*L1
        temp_L2 = 0
        while abs(L2 - temp_L2) > 0.00001:
            temp_L2=L2
            L2 = L1-(atan(r/temp_L2)-target_angle+np.pi)*r;

        # changed 98.1 --> 9.81
        target_velocity = sqrt(2*9.81*(sqrt(L2**2+r**2)+L1*cos(target_angle)+r*sin(target_angle)))
        print('---target_velocity---', target_velocity)
        target_thetad = target_velocity/L
        energy_threshold=((1-cos(target_angle))*g*L+0.5*target_velocity**2)*0.02
        return energy_threshold


    def observer(self):
        #print('----------in def observer------------')

        start_height = 4 # of drone, for change in grav. pot. energy
        payloadState = self.getLinkState('drone_kinect_fixed_rope5::link_1', 'ground_plane')
        self.all_state[6] = payloadState.link_state.twist.angular.y
        tetherQuat = payloadState.link_state.pose.orientation # roll is what is needed
        roll, pitch = quatToEuler(tetherQuat.w, tetherQuat.x, tetherQuat.y, tetherQuat.z)
        self.listener()
        velMagn = sqrt(payloadState.link_state.twist.linear.x**2 + payloadState.link_state.twist.linear.z**2)
        self.energy = 0.5*self.payloadMass*velMagn**2  + self.payloadMass*9.81*(payloadState.link_state.pose.position.z- self.global_start.pose.position.z)   #(self.start_pos[1]-0.3))
        self.updateState()
        # y, z, tether-roll, drone-roll, drone-vel-y, drone-vel-z, tether-angle-dot, drone-angle-dot, branch-radius
        reward = 0
        Distance = self.distance()
        #print('\n-----Distance (from mid-tether to branch)-----', Distance)
        self.distane_list.append(Distance)
        reward -= abs(Distance-(self.all_state[-1]-0.1))*0.3
        reward += self.energy # REMOVED G.P.H. ENERGY FROM REWARD, ONLY BALL K.E. MAKES IT UP
        
        if self.current_state_g.system_status in range(5,9):
            self.isDone = True
            self.segmentReward[-1] += float(reward)
            #self.reward_without_trackingError.append(reward)
            self.stopDrone()
            return reward
        if Distance <= self.all_state[-1] + 0.1:
            print('----current pos----',self.actual_trajectory[-1])
            i = -1
            while self.actual_trajectory[i] == self.actual_trajectory[-1]:
                i -= 1
            print('----prev pos----', self.actual_trajectory[i])
            print('----branch pos-----', self.branchState, '\n-----self.start_pos------', self.start_pos)
            if True:
            #check_crash(self.actual_trajectory[-2], self.actual_trajectory[-1], self.branchState, self.start_pos)==False:
                print("Attached")
                print('--VELOCITY--', velMagn)
                reward += 100
                standard = self.energy_standard()
                if self.energy >= standard:
                    print("sufficient energy for perching %.3f/%.3f" %(self.energy,standard))
                    if self.mode == "train":
                        self.model_saver(self.model_path)
                    reward += 400
                    self.isDone = True
                else :
                    print("not sufficient energy %.3f/%.3f" %(self.energy,standard))
                    self.isDone = True
            else:
                print('---> CRASH ')
                self.isDone = True
        #print('----instant reward without tracking----', reward)
        self.segmentReward[-1] += float(reward)
        #self.reward_without_trackingError.append(reward)
        #return reward


    def distance(self):
        rospy.wait_for_service('/gazebo/get_link_state')
        tether = self.getLinkState('drone_kinect_fixed_rope5::link_0', 'world')
        distance = sqrt((tether.link_state.pose.position.x-self.branchState.pose.position.x)**2 + (tether.link_state.pose.position.z-self.branchState.pose.position.z)**2)
        return distance


    def done(self):
        self.episode_num += 1
        print('\ndone episode', self.episode_num, 'total_reward is :', self.ep_reward[-1])
        #self.reset()


    def updateState(self):
        pos = self.getLinkState('drone_kinect_fixed_rope5::cam_link', 'ground_plane')
        payload = self.getLinkState('drone_kinect_fixed_rope5::link_1', 'ground_plane')
        tetherRoll, pitch = quatToEuler(payload.link_state.pose.orientation.w, payload.link_state.pose.orientation.x, 
                                    payload.link_state.pose.orientation.y, payload.link_state.pose.orientation.z)
    
        posit = self.getModelState('drone_kinect_fixed_rope5', 'world')
        pos_x = self.branchState.pose.position.x - posit.pose.position.x
        pos_z = self.branchState.pose.position.z - posit.pose.position.z
        self.all_state= np.array([pos_x, pos_z,
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
            torch.save(self.model.value_net.state_dict(), model_path+"value_13th_sep.dat")
            torch.save(self.model.target_value_net.state_dict(), model_path+"target_13th_sep.dat")
            torch.save(self.model.q_net.state_dict(), model_path+"qvalue_13th_sep.dat")
            torch.save(self.model.policy_net.state_dict(), model_path+"policy_13th_sep.dat")
            print('save model sucessfully')


    def model_loader(self,model_path):
        path = model_path
        print('MODEL_LOADER CALLED')
        if os.path.exists(model_path+"policy.dat") and self.load_command=="on":
            print("PATH EXISTS")
            self.model.value_net.load_state_dict(torch.load(model_path+"value_13th_sep.dat"))
            self.model.value_net.eval()
            self.model.target_value_net.load_state_dict(torch.load(model_path+"target_13th_sep.dat"))
            self.model.target_value_net.eval()
            self.model.q_net.load_state_dict(torch.load(model_path+"qvalue_13th_sep.dat"))
            self.model.q_net.eval()
            self.model.policy_net.load_state_dict(torch.load(model_path+"policy_13th_sep.dat"))
            self.model.policy_net.eval()
            self.model.policy_net_local.load_state_dict(torch.load(model_path+"policy_13th_sep.dat"))
            self.model.policy_net_local.eval()
            print('load model sucessfully')