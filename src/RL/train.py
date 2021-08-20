from math import pi
import rospy
from std_srvs.init_node import empty



class Duocopter():
	def __init__(self,action_dim =2,state_dim=9,
                initial_state=None,
                model_path='',
                save_command="on",load_command="on",
                dt=1.0/500, mode='test_ideal'):
        
		# states are position (x,y), angle (pitch), velocity (x,y), ang vel (pitch rate), acceleration (x,y)
        self.all_state= np.array([-2.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

        self.energy_threshold = self.energy_standard()
        self.energy = 0
        
        # trajectory consists of position (x,y), pitch (radians), velocity (x,y), time
       	self.desired_trajectory = []
       	self.actual_trajectory = []

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


	def waitStartPos(self):

	def trajectorySegment(self, time): # i.e. step

		startPitch = self.desired_trajectory[time][1]
		endPitch = self.desired_trajectory[time+1][1]
		startVel = self.desired_trajectory[time][2]
		endVel = self.desired_trajectory[time+1][2]
		


	def makeNewTrajetory(self):


	def getTrackingError(self):



	def approximateTrajectoryFunction(self, waypoints):

		time = pylab.array([i[0] for i in waypoints])
		y = pylab.array([i[1] for i in waypoints])
		a = pylab.polyfit(time, y, deg=7)
		esty = a[0]*(x**7) + a[1]*(x**6) + a[2]*(x**5) + a[3]*(x**4) + a[4]*(x**3) + a[5]*(x**2) +a[6]*(x) + a[7]
		self.desired_trajectory = esty

	def rl_action(self):
        state = self.all_state
        self.last_state = deepcopy(state)
        #state=self.noise_state(state)
        action = self.model.action(state)
        return action

    def rl_determinant_action(self):
        state = self.all_state
        self.last_state = deepcopy(state)
        #state=self.noise_state(state)

        action = self.model.determinant_action(state)

        return action

    def train(self, done):
        if len(self.model.replay_buffer) > self.batch_size:
            self.count += 1
            self.model.soft_q_update(self.batch_size, done, value_lr  = 0.5e-4, q_lr = 0.5e-4, policy_lr = 0.5e-4)



    def buffer_push(self,reward,done):
        if self.last_state is not None and self.last_action is not None:
            self.model.replay_buffer.push(self.last_state, self.last_action, reward, self.all_state, done)

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



    def reward_function(self):
        
        ## I CHANGED THE 'STATE' REWARD FUNCTION WEIGHT FROM 0.3 -> 0.2 !!!!!!!!!!!!
        ## (to reduce weight of target since increasing range of starting points ... not necasarily good target)
        ## (start of line below)

        reward = - 0.2*((self.all_state[0]-2.0)**2 + 2*(self.all_state[1]-2.2)**2) - 5 * getTrackingError()
        return reward



    def observer(self):
        (mass, L, g, m2) = self.params
        done = 0
        reward = self.reward_function()
        Distance = self.distance()

        phid = self.all_state[6] # phi represents beta in the thesis
        phi = self.all_state[2]
        vx = self.all_state[4]
        vz = self.all_state[5]
        v = sqrt((vx+phid*L*cos(phi))**2+(vz+phid*L*sin(phi))**2)

        self.energy = ((1-cos(self.all_state[2]))*g*L+0.5*(v)**2)*0.02

        reward -= abs(Distance-(self.all_state[-1]-0.1))*0.3
        reward += self.energy
        if Distance <= self.all_state[-1]:
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
        self.total_reward += reward

        return reward, done

    def distance(self):
        (mass, L, g, m2) = self.params
        state = self.all_state
        P1 = np.array([state[0] + 0.3 * np.sin(state[2]),state[1] - 0.3 * np.cos(state[2])])
        P2 = np.array([state[0] + (L-2*state[-1]*pi) * np.sin(state[2]),state[1] - (L-2*state[-1]*pi) * np.cos(state[2])])
        P0 = np.array([3 ,1])
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




if __name__ == 'main':

	rospy.init_node('train')

	# 1 . initialise model
	# 2.

	for ep in no_episodes:


		#                 ===============================================
		## IMPORTANT QUESTION -> HAVE MANY STEPS IN EACH EPISODE WHERE STATE & REWARD IS GIVEN TO AGENT?
		## ... SO ALLOW AGENT TO CHANGE TRAJECTORY DURING EPISODE?
		## OR GENERATE TRAJECTORY AND COMPLETE IT THEN GIVE REWARD SIGNAL TO AGENT AT END OF EP..?
		#                 ===============================================

		## I THINK IT MAKES MORE SENSE TO HAVE STEPS IN EACH EPISOE:
		# TIME=0: PRODUCES FULL TRAJECTORY USING EACH FUTURE ESTIMATED STATE
		# TIME=1: CHECKS DISTANCE TO WHERE IT SHOULD BE AT CURRENT TIME & UPDATES NN GIVEN REWARDS
		# THEN PRODUCS NEW TRAJECTORY FROM WHEREVER IT IS NOW
		# ETC...



		# 1. reset simulation (random starting point?)
		# 2. takeoff drone
		# 3. feed drone & branch position to RL which returns series of 
		# waypoints with angle at each (and velocity -> endpoint would need velocity)
		# 4. function approximation using waypoints
		# 5. compute optimal time trajectory

		for s in no_steps:   # step corresponds to distance between waypoints

			# 6. follow trajectory until next waypoint
			# 7. compute reward at current position:
			#     - penalise total tracking error
			#     - track final position & velocity of ball for other perching rewards
			# 8. update weights of NN 
			# 9. produce new set of waypoints and trajectory when at time corresponding with next waypoint


		
