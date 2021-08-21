#!/usr/bin/env python

from math import pi, atan2, cos, sin, arcos, arsin, tan, sqrt
import rospy
from std_srvs.init_node import empty
import Trajectory
import py_gnc_functions 
import numpy as np
from mavros_msgs import Twist
from mavros_msgs import PoseStamped
from mavros_msgs import TwistedStamped
from sensor_msgs import Imu
from SAC.Model import SAC
from SAC.utils.ReplayBuffer import ReplayBuffer
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetModelState

class Duocopter():
	def __init__(self,action_dim =2,state_dim=9,
                initial_state=None,
                model_path='',
                save_command="on",load_command="on",
                dt=1.0/500, mode='test'):
        
		# states are position (x,y), angle (pitch), velocity (x,y), ang vel (pitch rate), acceleration (x,y)
        self.all_state= np.array([-2.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

        self.energy_threshold = self.energy_standard()
        self.energy = 0
        
        # trajectory consists of position (x,y), pitch (radians), velocity_magnitude
       	self.desired_trajectory = [] # z coords, distance of (y_last - y_first)/100 between each 
       	self.actual_trajectory = []
        self.trackingError = 0
        self.segmentReward = 0
        self.getLinkState = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        rospy.wait_for_service('/gazebo/get_link_state')
        self.branchState = self.getLinkState('branch_new_2m::link_0', 'ground_plane')
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', data_class=Twist, queue_size=5)
        self.attitude_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude', data_class=PoseStamped, queue_size=5)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', data_class=Imu, self.imuCallback)
        self.vel_sub = rospy.Subscriber('mavros/local_position/velocity', data_class=TwistedStamped, self.velCallack)

        self.state_list = []
        self.mode = mode
        self.dt = dt
        self.time_elapsed = 0

        self.steps = 0
        self.episode_num = 0
        self.total_reward = 0
        self.reward = []

        self.last_state = None
        self.last_action = None


        # actions space is position, angle, velocity ..?
        # the NN can choose waypoints say 0.5m apart. I guess this can be represented as an angle 
        # relative to previous waypoint?
        # angle is in range -50deg to +50 deg (in radians)..? (pitch angle)
        # velocity 0-3 m/s ..?

        self.action_low = np.array([0, -0.873, 0])
        self.action_high = np.array([2*pi, 0.873, 3])
        
        self.batch_size=256
        self.save_command=save_command
        self.load_command=load_command
        self.model_path = model_path
        self.model = SAC(action_dim =len(self.action_high),state_dim=len(self.all_state),hidden_dim=256*2,buffer_size=1000000)

        self.model_loader(self.model_path)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self._max_retry = 20

        self.reset()
        

	def reset(self):        
		self.all_state=np.array([start_x, start_y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
	    self.last_state = None
	    self.last_action = None
	    self.total_reward = 0
	    self.steps = 0
	    self.energy = 0
	    self.desired_trajectory = []
	    self.actual_trajectory = []
        self.trackingError = 0
	    rospy.wait_for_service('/gazebo/reset_world')
	    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
  		reset_world()


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
		while z < takeoffHeight-0.1 or z > takeoffHeight+0.1:
			rospy.spinOnce()
			r.sleep()
			z = gnc.get_current_location().z


	def trajectorySegment(self, step): # i.e. step
		y = step*10  # y is 100 points, but only 10 steps
		startPitch = self.desired_trajectory[y][2]
		endPitch = self.desired_trajectory[y+10][2]
		startVel = self.desired_trajectory[y][3]
		endVel = self.desired_trajectory[y+10][3]
		return (startPitch endPitch, startVel, endVel)


	def makeNewTrajetory(self, step):

        rospy.spinOnce()
        location = gnc.get_current_location()
		state = [location.y, location.z, self.all_state[3], sqrt(self.all_state[4]**2 + self.all_state[5]**2)] # y, z, drone-roll, drone-speed
		self.last_state = state
        wayPointList = [state]
		for s in range(9-step):
			# angleNextPos, pitch, speed 
            if self.mode == 'train':
                action = self.model.action(state)  # feeding same thing can be fixed by time parametrising .. i think
			elif self.mode == 'test':
                action = self.model.determinant_action(state)
            state = [(wayPointList[-1] + 0.5*cos(action[0])), (wayPointList[-1] + 0.5*sin(action[0])), action[1], action[2]]
			wayPointList.append(state)
        self.last_action = wayPointList
		self.approximateTrajectoryFunction(wayPointList)


	def approximateTrajectoryFunction(self, waypoints):
		self.yDiff = waypoints[-1][0] - waypoints[0][0]
		y = pylab.array([yDiff/100 * i for i in range(100)])  # y is forward, not time parametrizd yet
		z = pylab.array([i[1] for i in waypoints])  # z is up
		a = pylab.polyfit(time, z, deg=7)
		esty = a[0]*(y**7) + a[1]*(y**6) + a[2]*(y**5) + a[3]*(y**4) + a[4]*(y**3) + a[5]*(y**2) +a[6]*(y) + a[7] 
		self.desired_trajectory = esty


    def train(self, done):
        if len(self.model.replay_buffer) > self.batch_size:
            self.count += 1
            self.model.soft_q_update(self.batch_size, done, value_lr  = 0.5e-4, q_lr = 0.5e-4, policy_lr = 0.5e-4)


    def buffer_push(self, done):
        if self.last_state is not None and self.last_action is not None:
            self.model.replay_buffer.push(self.last_state, self.last_action, self.segmentReward, self.all_state, done)


    def energy_standard(self, x = 2.0, y = 2.0):
        (mass, L, g, m2) = self.params
        target_x = x
        target_y = y
        tree_x = 3.0
        tree_y = 1.0
        r = self.all_state[-1]
        target_angle = arctan((tree_x-target_x)/(target_y-tree_y))-arcsin(r/sqrt((target_x-tree_x)**2+(target_y-tree_y)**2))#np.pi/2
        L1 = L - sqrt((target_x-tree_x)**2+(target_y-tree_y)**2-r**2)
        L2 = 0.7*L1
        temp_L2 = 0
        while abs(L2 - temp_L2) > 0.00001:
            temp_L2=L2
            L2 = L1-(arctan(r/temp_L2)-target_angle+np.pi)*r;
        target_velocity = sqrt(2*98.1*(sqrt(L2**2+r**2)+L1*cos(target_angle)+r*sin(target_angle)))
        target_thetad = target_velocity/L
        energy_threshold=((1-cos(target_angle))*g*L+0.5*target_velocity**2)*0.02
        return energy_threshold


    # def reward_function(self):
    #     reward = - 0.3*((self.all_state[0]-2.0)**2 + 2*(self.all_state[1]-2.2)**2)
    #     return reward


    def observer(self):

        # NEED ENERGY OF PAYLOAD
        # - drone pitch (check)     }
        # - angle of tether         } THESE TWO GIVE ANGLE OF PAYLOAD

        # ^^ NOT NEEDED IF HEIGHT OF PAYLOAD IS KNOWN ANOTHER WAY !!!

        # - velocity of payload
        # - drone velocity   }< MAYBE NOT NEEDED ... 
        # use 1/2*m*v^2 + mgh of PAYLOAD for energy... > multiply by 0.02 to scale...
        # h found using height of drone minus (i think) 1-cos(phi) times tether length...

        
        payloadState = self.getLinkState('drone_kinect_fixed_rope2::link_1', 'ground_plane')

        # ^^^^^^^^^^^        ======================================================
        # NEED TO CHANGE SECOND PARAMETER IN getPAyloadState I.E. REFFERENCE MODEL TO SOMETHING THAT STARTS DIRECTLY
        # UNDER TO GET ACCURATE CHANGE IN Y (OR TAKE IT AWAY FROM STARTING VALUE) 
        #          ============================================================================
        
        self.all_state[6] = payloadState.link_state.twist.angular.x
        tetherQuat = payloadState.link_state.pose.orientation # roll is what is needed
        roll, pitch = self.quatToEuler(tetherQuat.w, tetherQuat.x, tetherQuat.y, tetherQuat.z)
        payloadVel = payloadState.link_state.twist.linear
        #(mass, L, g, m2) = self.params
        rospy.spinOnce()
        velMagn = sqrt(payloadVel.y**2 + payloadState.z**2)
        self.energy = 0.5*self.payloadMass*velMagn**2 + self.payloadMass*9.81*payloadState.link_state.pose.position.z
        self.all_state = [payloadState.link_state.pose.position.y,
                        payloadState.link_state.pose.position.z, 
                        roll, , ,self.branchRadius]
        # ^^^^^  y, z, tether-roll, drone-roll, drone-vel-y, drone-vel-z, tether-angle-dot, drone-angle-dot, branch-radius

        done = 0
        #reward = self.reward_function()
        Distance = self.distance()

        reward -= abs(Distance-(self.all_state[-1]-0.1))*0.3
        reward += self.energy
        if Distance <= self.all_state[-1] + 0.05:
            print("Attached")
            reward += 100
            standard = self.energy_standard(x = self.all_state[0], y = self.all_state[1])
            if self.energy >= standard:
                print("sufficient energy for perching %.3f/%.3f" %(self.energy,standard))
                if self.mode == "train":
                    self.model_saver(self.model_path)
                reward += 400
                done = 1
            else :
                print("not sufficient energy %.3f/%.3f" %(self.energy,standard))
                done = 1
        
        if self.steps >= 649 :
            done = 1
        self.segmentReward += reward
        self.total_reward += reward

        return reward, done


    def distance(self):
        
        #tetherState = self.getLinkState('drone_kinect_fixed_rope2::link_0', 'ground_plane')
        #distance = sqrt((branchState.y-tetherState.y)**2 + (branchState.z-tetherState.z)**2)

        state = self.all_state
        P1 = np.array([state[0] + 0.3 * np.sin(state[2]),state[1] - 0.3 * np.cos(state[2])])
        P2 = np.array([state[0] + (L-2*state[-1]*pi) * np.sin(state[2]),state[1] - (L-2*state[-1]*pi) * np.cos(state[2])])
        P0 = np.array([self.branchState.link_state.pose.position.y ,self.branchState.link_state.pose.position.z]) 
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
        return distance


    def done(self):
        self.episode_num += 1
        self.reward.append(self.total_reward)
        print('\ndone episode', self.episode_num, 'total_reward is :', self.total_reward)
        self.reset()


    def eulerToQuat(self, roll, pith, yaw):
        cy = cos(yaw*0.5)
        sy = sin(yaw*0.5)
        cp = cos(pitch*0.5)
        sp = sin(pitch*0.5)
        cr = cos(roll*0.5)
        sr = sin(roll*0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy     ### NOT SURE SHOULD PUBLISH QUAT AS X,Y,Z,W OR W,X,Y,Z ???
        return x, y, z, w                   ## DIFFERENT ON WIKIPEDIA COMPARED TO MAVROS ...


    def quatToEuler(self, w, x, y, z):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
        #t3 = +2.0 * (w * z + x * y)
        #t4 = +1.0 - 2.0 * (y * y + z * z)
        #yaw_z = atan2(t3, t4)
        return roll_x, pitch_y # in radians


    def follow_trajectory_segment(self, startPitch, endPitch, startVel, endVel, desiredTrajectory, yDiff, step):

        trackingError = 0
        posList = [i for i in desiredTrajectory[step:step+10]]
        pitchList = [startPitch + i*(endPitch-startPitch)/10 for i range(10)]
        speedList = [endVel + i*(endVel-startVel)/10 for i in range(10)]
        prevZ = desiredTrajectory[0] 
        count = 1
        self.segmentReward = 0
        startLocation = gnc.get_current_location()
        while rospy.ok() and count < 10:
            deltaZ = desiredTrajectory[count] - prevZ
            unitVec = np.array(yDiff/(yDiff**2 + deltaZ**2), deltaZ/(yDiff**2 + deltaZ**2))
            velocity = speedList[count-1] * unitVec
            vel = Twist()
            pose = PoseStamped()
            vel.linear.x = 0
            vel.linear.y = velocity[0]
            vel.linear.z = velocity[1]
            self.vel_pub.publish(velocity)
            x,y,z,w = eulerToQuat(0, pitchList[count-1], 0)
            pose.header.stamp = rospy.get_rostime()
            pose.pose.orientation.x = x
            pose.pose.orientation.y = y
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w
            self.attitude_pub.publish(pose)
            prevZ = desiredTrajectory[count]
            r.sleep()
            location = gnc.get_current_location()
            trackingError += (abs(location.y - startLocation.y * (count -1)) + abs(location.z - desiredTrajectory[count+1]))
            isDone = self.observer()
            count += 1
        self.trackingError = trackingError
        self.buffer_push(isDone)


    def imuCallback(self, data):
        roll, pitch = quatToEuler(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
        self.all_state[2] = roll


    def velCallack(self, data):
        self.all_state[4] = data.twist.linear.y
        self.all_state[5] = data.twist.linear.z
        self.all_state[7] = data.twist.angular.x

