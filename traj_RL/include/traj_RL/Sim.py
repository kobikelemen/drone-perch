


class Sim():

	def __init__(self):


        self.gnc = gnc
        self.current_state_g = State()
        self.listener()
        self.payloadMass = 0.01
        self.getLinkState = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.getModelState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.mavros_sub = rospy.Subscriber('/mavros/state', data_class=State, queue_size=2, callback=self.mavros_cb)
        rospy.wait_for_service('/gazebo/get_link_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.branchState = self.getModelState('branch_new_2m', 'world')
        pos = self.getLinkState('drone_kinect_fixed_rope5::cam_link', 'ground_plane')
        teth = self.getLinkState('drone_kinect_fixed_rope5::link_0', 'world')
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
        self.distance_list = []
        self.mode = mode
        self.step = 0
        self.episode_num = 0
        self.ep_reward = [0]
        self.reward = []
        self.isDone = False
        self.last_state = None
        self.last_action = None
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
        self._max_retry = 20

	def reset(self, waypoints, actual_trajectory, velocity_list, 
                trackingError_list, reward_without_trackingError, segmentReward, polynomial):

        print('in def reset')
        print('\n------self.waypoints--------\n', waypoints)
        print('\n---------self.actual_trajectory----------\n', actual_trajectory)
        
        y = np.linspace(waypoints[0][0], waypoints[-1][0], num=100, endpoint=True)
        poly = [[i, float(polynomial(i))] for i in y]
        print('\n---------self.poly--------------\n', poly)
        print('\n----------self.velocity_list---------\n', velocity_list)
        print('\n--------self.trackingError_list------\n', trackingError_list)
        print('\n---------self.reward_without_trackingError------\n', reward_without_trackingError)
        print('\n--------self.segmentReward-------\n', segmentReward)
        self.isDone = False
        self.last_state = None
        self.last_action = None
        self.ep_reward.append(0)
        self.step = 0
        self.energy = 0
        self.distance_list = []
        rospy.wait_for_service('/gazebo/set_model_state')
        linkState = LinkState()
        self.stopDrone()
        print('----------position---------', self.gnc.get_current_location())
        self.gnc.set_destination(0,0,0,0)
        modelState = ModelState()
        modelState.model_name = 'drone_kinect_fixed_rope5'
        modelState.pose.position.x = 0.62
        modelState.pose.position.y = -0.26
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
        self.setModelState(modelState)
        rospy.sleep(10)
        print('----------position reset------------')
        print('---------start reached------------')
        pos = self.getLinkState('drone_kinect_fixed_rope5::cam_link', 'ground_plane')
        payload = self.getLinkState('drone_kinect_fixed_rope5::link_1', 'ground_plane')
        tetherRoll, pitch = quatToEuler(payload.link_state.pose.orientation.w, payload.link_state.pose.orientation.x,
                                            payload.link_state.pose.orientation.y, payload.link_state.pose.orientation.z)
        self.all_state= np.array([pos.link_state.pose.position.x, pos.link_state.pose.position.z,
                            tetherRoll, 0.0, 0.0, 0.0, payload.link_state.twist.angular.y, 0.0, 0.01], dtype=np.float32)
        # y, z, tether-roll, drone-roll, drone-vel-y, drone-vel-z, tether-angle-dot, drone-angle-dot, branch-radius        
        self.listener()


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
        self.distance_list.append(Distance)
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

	def distance(self):
        rospy.wait_for_service('/gazebo/get_link_state')
        tether = self.getLinkState('drone_kinect_fixed_rope5::link_0', 'world')
        distance = sqrt((tether.link_state.pose.position.x-self.branchState.pose.position.x)**2 + (tether.link_state.pose.position.z-self.branchState.pose.position.z)**2)
        return distance

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





