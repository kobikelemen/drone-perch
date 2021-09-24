
#!/usr/bin/env python

from math import pi, atan, atan2, cos, sin, acos, asin, tan, sqrt
from copy import deepcopy
import roslaunch
import rospy
from std_srvs.srv import Empty
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

from .Model import *
from .utils import *
from gazebo_msgs.srv import GetLinkState, DeleteModel, SpawnModel
from gazebo_msgs.srv import GetModelState, SetLinkState, SetModelState
from gazebo_msgs.msg import ModelState, LinkState
from mavros_msgs.srv import CommandTOL, CommandTOLRequest, CommandBool
from mavros_msgs.msg import State, AttitudeTarget
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
    def __init__(self,gnc, action_dim=6,state_dim=4,
                initial_state=None,
                model_path='/home/kobi/catkin_ws/src/traj_RL/trained_models/',
                save_command="on",load_command="off",
                mode='train'):
        
        # states are position (x,y), angle (pitch), velocity (x,y), ang vel (pitch rate), acceleration (x,y)
        print('in Duocopter __init__')
        self.gnc = gnc
        self.current_state_g = State()
        
        time.sleep(3)
        self.payloadMass = 0.01
        self.getLinkState = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.getModelState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.branchState = self.getModelState('branch_new_2m', 'world')
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', data_class=Twist, queue_size=5)
        self.att_vel_pub = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', data_class=TwistStamped, queue_size=3)
        self.attitude_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude', data_class=PoseStamped, queue_size=5)
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', data_class=PoseStamped, queue_size=5)
        self.att_pub_raw = rospy.Publisher('/mavros/setpoint_raw//target_attitude', data_class=AttitudeTarget, queue_size=3)
        self.mavros_sub = rospy.Subscriber('/mavros/state', data_class=State, queue_size=2, callback=self.mavros_cb)
        rospy.wait_for_service('/gazebo/get_link_state')

        pos = self.getLinkState('drone_kinect_fixed_rope5::cam_link', 'ground_plane')
        teth = self.getLinkState('drone_kinect_fixed_rope5::link_0', 'world')
        payload = self.getLinkState('drone_kinect_fixed_rope5::link_1', 'ground_plane')
        tetherRoll, pitch = quatToEuler(payload.link_state.pose.orientation.w, payload.link_state.pose.orientation.x,
                                            payload.link_state.pose.orientation.y, payload.link_state.pose.orientation.z)
        

        posit = self.getModelState('drone_kinect_fixed_rope5', 'world')
        pos_x = self.branchState.pose.position.x - posit.pose.position.x
        pos_z = self.branchState.pose.position.z - posit.pose.position.z
        self.all_state= np.array([pos_x, pos_z,
                            tetherRoll, 0.0, 0.0, 0.0, payload.link_state.twist.angular.y, 
                            0.0, 0.01], dtype=np.float32)
        # x distance to branch, z distance to branch, tether-roll, drone-pitch, drone-vel-y, drone-vel-z, tether-angle-dot, drone-angle-dot, branch-radius        
        
        # >>>> STARTING FROM INDEX 0 <<<< REMOVIG INDEX:3 AND INDEX:
        # DRONE PITCH IS ALWAYS 0... I THINK 

        self.listener()

        self.energy_threshold = self.energy_standard()
        self.energy = 0
        
        self.waypoints_list = []
        self.actual_trajectory = []
        self.velocity_list = []
        self.reward_without_trackingError = []
        self.reward_with_trackingError = []
        self.tracking_error_list = []
        self.trackingError = 0
        self.segmentReward = [0]
        self.segmentLength = 5
        self.tracking_const = 100
        self.energy_const = 10
        self.mode = mode
        self.step = 0
        self.episode_num = 0
        self.ep_reward = [0]
        self.reward = []
        self.isDone = False
        self.last_state = None
        self.last_action = None
        #self.action_low = np.array([-pi/2, 0,-pi/2, 0, -pi/2, 0])
        self.action_high = np.array([1.5, 2, 1.5, 2, 1.5, 2])
        # 1.5 radians instead of pi/2 to avoid vertical
        self.batch_size = 50
        self.save_command=save_command
        self.load_command=load_command
        self.model_path = model_path
        self.model = SAC(action_dim=len(self.action_high),state_dim=len(self.all_state),hidden_dim=256,buffer_size=1000000)
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
        print('\n------self.waypoints_list--------')
        temp = []
        for i in self.waypoints_list:
            temp2 = []
            for j in i:
                temp2 += j
            temp.append(temp2)
        print(temp)
        #print('\n--------self.desired_trajectory----------\n', self.desired_trajectory)
        print('\n---------self.actual_trajectory----------\n', self.actual_trajectory)
        #print('--------self.branchPos------------', self.branchPos)
        print('\n---------self.poly_list--------------\n')
        self.poly_list = []
        for i in range(len(self.waypoints_list)):
            self.approximateTrajectoryFunction(self.waypoints_list[i])
            y = np.linspace(self.waypoints_list[i][0][0], self.waypoints_list[i][-1][0], num=20, endpoint=True)
            self.poly_list.append([[j, float(self.polynomial(j))] for j in y])
        print(self.poly_list)

        print('\n----------self.velocity_list---------\n', self.velocity_list)
        print('\n--------self.tracking_error_list------\n', self.tracking_error_list)
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
        self.reward_without_trackingError = []
        self.reward_with_trackingError = []
        self.waypoints_list = []
        self.tracking_error_list = []
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
        self.gnc.set_destination(0,0,1.05,0)
        modelState = ModelState()
        modelState.model_name = 'drone_kinect_fixed_rope5'
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


        self.setModelState(modelState)
        rospy.sleep(8)

        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        self.vel_pub.publish(vel)
        rospy.sleep(2)

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
        # x distance to drone, z distance to drone, tether-roll, drone-pitch, drone-vel-y, drone-vel-z, tether-angle-dot, drone-angle-dot, branch-radius        
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
        location = self.gnc.get_current_location()
        waypoints = []
        if start_ep:
            s = self.getModelState('drone_kinect_fixed_rope5', 'world')
            self.start_pos = [s.pose.position.x, s.pose.position.z]
            #self.start_pos = [location.y, location.z]
            self.y = 0
            self.z = 0
        self.last_state = self.all_state
        if self.mode == 'train':
            action = np.array(self.model.action(self.all_state)) 
            print(' ============================')
            print(' ============================')
            print(' ---- raw action ---- ', action)
            print(' ============================')
            print(' ============================')

            waypoint = self.scaleAction(action)
            print(' ----- SCALED ACTION ----- ', waypoint)
            #self.updateState()
            s = self.getModelState('drone_kinect_fixed_rope5', 'world')
            waypoints.append([self.y, self.z, sqrt(s.twist.linear.x**2 + s.twist.linear.z**2)])
            
            waypoints.append([
                waypoints[-1][0]+waypoint[0], 
                waypoints[-1][1]+waypoint[1], 
                waypoint[2]
                ])
            
            waypoints.append([
                waypoints[-1][0]+waypoint[3], 
                waypoints[-1][1]+waypoint[4], 
                waypoint[5]
                ])

            waypoints.append([
                waypoints[-1][0]+waypoint[6], 
                waypoints[-1][1]+waypoint[7], 
                waypoint[8]
                ])

            self.waypoints_list.append(waypoints)


        elif mode == 'test': # THIS NEEDS FIXING -->
            action = np.array(self.model.determinant_action(self.all_state))    # NEED TO ADD DETERMINANT TRAJECTORY GENERATION
            waypoint =self.scaleAction(action)                      # WHERE THE NEXT STATE IS ASSUMED TO BE GIVEN BY PREDICTED STATE > MAKE ALL WAYPOINTS
            self.waypoints.append(waypoint[0:3])
            self.waypoints.append(waypoint[3:6])
        return waypoints
   


    def approximateTrajectoryFunction(self, waypoints):

        print('---in def approximateTrajectoryFunction----')
        y = np.array([i[0] for i in waypoints])
        z = np.array([i[1] for i in waypoints])

        self.polynomial = interp1d(y, z, kind='cubic')

    def follow_trajectory_segment(self, startVel, endVel, waypoints, global_start):

        print('----in follow_trajectory_segment-----') 
        print('--- waypoints ----', waypoints)
        r = rospy.Rate(self.segmentLength*5)
        trackingError = 0

        # 0.192 IS THERE BECAUSE cam_link IS THAT MUCH BELOW CENTRE OF DRONE --->                               ^^^^^^^^^^^^^^^^^ 

        end_y = waypoints[-1][0]
        end_z = waypoints[-1][1]
        delta_y = 0.01
        count = 0
        self.segmentReward.append(0)
        trackingError = 0
        instant_track_error = 0
        self.listener()

        segment_start_y = self.y
        print('--- self.y before while loop --- ', self.y)
        print(' ---- end_y -----', end_y)
        first = True
        while not self.y + delta_y >= end_y and not rospy.is_shutdown():
            print('--- in while loop ---')
            # if self.y + delta_y >= end_y:
            #     break
            if self.y <= 0:
                self.y = 0

            try:
                #print(' --- in try ---- ')
                #print(' ---- end_y ----', end_y)
                #print(' --- self.y + delta_y ---- ', self.y + delta_y)
                delta_z = self.polynomial(self.y + delta_y) - self.z #self.polynomial(self.y+delta_y) 
            except:
                print('---self.y + delta_y is too big, calling reset---')
                self.reset(global_start)
                return 0

            #print('---delta_y---', delta_y)
            #print('---delta_z----', delta_z)
            unit_vec = np.array([2*delta_y/sqrt((2*delta_y)**2 + delta_z**2), delta_z/sqrt((2*delta_y)**2 + delta_z**2)])
            angle = self.all_state[3]
            #unit_vec = np.array([cos(angle)*unit_vec[0] - sin(angle)*unit_vec[1], 
            #                    sin(angle)*unit_vec[0] + cos(angle)*unit_vec[1]])


            speed = startVel + ((self.y - segment_start_y)/ (end_y - segment_start_y))*(endVel - startVel)
            if speed < 0.1:
                speed = 0.1
            #print(' ----- self.y ----', self.y)
            st = self.getModelState('drone_kinect_fixed_rope5', 'world')
            #print(' ----- model_velocity (x,y) -----', st.twist.linear.x, st.twist.linear.z)
            velocity = unit_vec * speed
            
            self.velocity_list.append(list(velocity))
            vel = Twist()
            vel.linear.x = 0
            vel.linear.y = velocity[0]
            vel.linear.z = velocity[1]
            vel.angular.x = 1
            self.vel_pub.publish(vel)
            r.sleep()
            





            """
            vel = Twist()
            att_raw = AttitudeTarget()
            att_raw.header.stamp = rospy.Time.now()
            att_raw.header.frame_id = 'world'
            att_raw.body_rate.y = 1
            self.att_pub_raw.publish(att_raw)
            #vel.header.stamp = rospy.Time.now()
            #vel.header.frame_id = 'world'
            #vel.linear.z = 0.15
            vel.linear.y = 0.3
            vel.linear.z = 0.3
            rospy.sleep(1)
            self.vel_pub.publish(vel)
            self.att_pub_raw.publish(att_raw)
            rospy.sleep(4)
            """

            #print(' --- all_state ---- ', self.all_state)
            pos = self.getModelState('drone_kinect_fixed_rope5', 'world')
            branch_y = self.branchState.pose.position.x
            branch_z = self.branchState.pose.position.z

            self.y = pos.pose.position.x - self.start_pos[0]
            self.z = pos.pose.position.z - self.start_pos[1]
            self.actual_trajectory.append([self.y,self.z])
            print(' ----- all_state ----- ', self.all_state)
            if first:
                first_pos = [self.y, self.z]
                first = False
            self.observer()
            if self.z < -2.5 or self.z > 2.5:
                self.isDone = True
                break
            if self.isDone:
                break

        if len(self.actual_trajectory) > 0:
            if first:
                first_pos = [self.y, self.z]
            #print(' ---- self.actual_trajectory ---- ', self.actual_trajectory)
            seg_error = self.calc_error(self.polynomial, self.actual_trajectory, waypoints, first_pos)
        
            #print(' ---- self.y AFTER while loop --- ', self.y)
            #print(' ---- isDone ----', self.isDone)

            self.ep_reward[-1] += seg_error * self.tracking_const
            self.trackingError += seg_error
            if self.step > 20:
                self.isDone = True
            self.buffer_push(self.isDone)
        else:
            print('-- len(actual_trajectory) -- ', len(self.actual_trajectory))
            print(' len(self.actual_trajectory) IS ZERO, EXITING ... ')
            #assert 1==2


    def calc_error(self, poly, actual_trajectory, waypoints, first_pos):


        #print(' ---- waypoints ------ ', waypoints)
        tracking_error = []
        start_y = waypoints[0][0]
        j = 0
        test = 1000
        start_index = None
        for index, i in enumerate(actual_trajectory):
            if i[0] == first_pos[0]:
                start_index = index
        assert start_index != None
        
        start_y_acc = actual_trajectory[start_index][0]
        start_z_acc = actual_trajectory[start_index][1]
        start = start_index

        for wypnt in waypoints:
            check_y = start_y_acc + wypnt[0]
            check_z = start_z_acc + wypnt[1]
            if check_y < 0:
                check_y = 0
            elif check_y > actual_trajectory[-1][0]:
                check_y = actual_trajectory[-1][0]

            for i in range(start, len(actual_trajectory)-1):


                #print(' ---- actual_trajectory[i][0] ---- ', actual_trajectory[i][0])
                #print(' ---- actual_trajectory[i+1][0] ---- ', actual_trajectory[i+1][0])
                if check_y >= actual_trajectory[i][0] and check_y <= actual_trajectory[i+1][0]:
                    #print(' --- in if check_y... ----')
                    start = i
                    interp_frac = (abs(actual_trajectory[start][0]-check_y)/abs(actual_trajectory[start+1][0]-actual_trajectory[start][0]))
                    #print(' ---- interp_y ---- ', actual_trajectory[start][0] + interp_frac * abs(actual_trajectory[start+1][0]-actual_trajectory[start][0]))
                    z_interp = actual_trajectory[start][1] + interp_frac * abs(actual_trajectory[start+1][1]-actual_trajectory[start][1])
                    tracking_error.append(abs(z_interp - wypnt[1]))
                    #print(' --- tracking_error recent --- ', tracking_error[-1])
                    break 
        #print(' --- self.tracking_error_list ---- ', self.tracking_error_list)
        self.tracking_error_list += tracking_error
        return sum(tracking_error)






    def stopDrone(self):
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        self.vel_pub.publish(vel)
        rospy.sleep(2)


    def scaleAction(self, action):
        #np.array([pi/2, 2, pi/2, 2, pi/2, 2])
        self.last_action = action
        action_ = [
                    action[0]*self.action_high[0], (action[1]+1)*self.action_high[1]/2,
                     action[2]*self.action_high[2], (action[3]+1)*self.action_high[3]/2,
                     action[4]*self.action_high[4], (action[5]+1)*self.action_high[5]/2
                    ] 

        waypntDis = 0.15/3  # <--three waypoints per action ...?
        a = [
        waypntDis*cos(action_[0]), waypntDis*sin(action_[0]), action_[1],
        waypntDis*cos(action_[2]), waypntDis*sin(action_[2]), action_[3],
        waypntDis*cos(action_[4]), waypntDis*sin(action_[4]), action_[5],
        ]
        #print(' ---- action in scaleAction ---- ', a)
        return a


    def train(self, done):
        if len(self.model.replay_buffer) > self.batch_size:
            self.model.soft_q_update(self.batch_size, done, value_lr = 0.5e-4, q_lr = 0.5e-4, policy_lr = 0.5e-4)


    def buffer_push(self, done):
        if self.last_state is not None and self.last_action is not None:
            self.model.replay_buffer.push(self.last_state, self.last_action, self.segmentReward[-1], self.all_state, done)


    def dist_reward(self, all_state):
        reward = -0.3 *((all_state[0]-0.2)**2 + 2*(all_state[1]+0.2)**2)
        return reward 


    def energy_standard(self, x=None, y=None):
        L = 0.3
        g = 9.81
        tree_x = self.branchState.pose.position.x
        tree_y = self.branchState.pose.position.z
        rospy.wait_for_service('/gazebo/get_model_state')
        pos = self.getModelState('drone_kinect_fixed_rope5', 'world')
        target_x = pos.pose.position.x
        target_y = pos.pose.position.z
        r = self.all_state[-1]
        try:
            target_angle = atan((tree_x-target_x)/(target_y-tree_y))-asin(r/sqrt((target_x-tree_x)**2+(target_y-tree_y)**2))#np.pi/2
        except:
            print('=== TARGET ANGLE FAILED, THESE ARE THE VARIABLES: ===')  
            print(' tree_x: ', tree_x, '   tree_y: ', tree_y, '   target_x: ', target_x, '   target_y: ', target_y, '   r: ', r)
            assert 1==2
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
        reward -= abs(Distance-(self.all_state[-1]-0.1))*0.3
        print(' ======= energy x energy_const======= ', self.energy*self.energy_const)

        reward += self.energy * self.energy_const # REMOVED G.P.H. ENERGY FROM REWARD, ONLY BALL K.E. MAKES IT UP
        
        """
        if self.current_state_g.system_status in range(5,9):
            self.isDone = True
            self.segmentReward[-1] += float(reward)
            #self.reward_without_trackingError.append(reward)
            self.stopDrone()
            return reward
        """

        if Distance <= self.all_state[-1] + 0.1:
            #print('----current pos----',self.actual_trajectory[-1])
            i = -1
            while self.actual_trajectory[i] == self.actual_trajectory[-1]:
                i -= 1
            print("Attached")
            print('--VELOCITY--', velMagn)
            reward += 100
            standard = self.energy_standard()
            if self.energy >= standard:
                print("sufficient energy for perching %.3f/%.3f" %(self.energy,standard))
                    
                reward += 400
                self.isDone = True
            else :
                print("not sufficient energy %.3f/%.3f" %(self.energy,standard))
                self.isDone = True
            if self.mode == "train":
                self.model_saver(self.model_path)
        reward = float(reward) + self.dist_reward(self.all_state)
        self.segmentReward[-1] += reward
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
        #print('--- in listener---')
        rospy.Subscriber('/mavros/imu/data', Imu, self.imuCallback)
        s = self.getModelState('drone_kinect_fixed_rope5', 'world')
        self.all_state[4] = s.twist.linear.x
        self.all_state[5] = s.twist.linear.z
        self.all_state[7] = s.twist.angular.y
        rospy.Subscriber('/mavros/state', data_class=State, queue_size=2, callback=self.mavros_cb)


    def imuCallback(self, data):
        roll, pitch = quatToEuler(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
        self.all_state[3] = pitch

    def mavros_cb(self, data):
        self.current_state_g = data


    def model_saver(self,model_path):
        if self.save_command=="on":
            torch.save(self.model.value_net.state_dict(), model_path+"value_14th_sep.dat")
            torch.save(self.model.target_value_net.state_dict(), model_path+"target_14th_sep.dat")
            torch.save(self.model.q_net.state_dict(), model_path+"qvalue_14th_sep.dat")
            torch.save(self.model.policy_net.state_dict(), model_path+"policy_14th_sep.dat")
            print('save model sucessfully')


    def model_loader(self,model_path):
        path = model_path
        print('MODEL_LOADER CALLED')
        if os.path.exists(model_path+"policy.dat") and self.load_command=="on":
            print("PATH EXISTS")
            self.model.value_net.load_state_dict(torch.load(model_path+"value_14th_sep.dat"))
            self.model.value_net.eval()
            self.model.target_value_net.load_state_dict(torch.load(model_path+"target_14th_sep.dat"))
            self.model.target_value_net.eval()
            self.model.q_net.load_state_dict(torch.load(model_path+"qvalue_14th_sep.dat"))
            self.model.q_net.eval()
            self.model.policy_net.load_state_dict(torch.load(model_path+"policy_14th_sep.dat"))
            self.model.policy_net.eval()
            self.model.policy_net_local.load_state_dict(torch.load(model_path+"policy_14th_sep.dat"))
            self.model.policy_net_local.eval()
            print('load model sucessfully')