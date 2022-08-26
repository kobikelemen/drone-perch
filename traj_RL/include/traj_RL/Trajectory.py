from SAC.Model import *
from SAC.utils import *


class Trajectory(Sim):
	def __init__(self):
		Sim.__init__(self)

		self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', data_class=Twist, queue_size=5)
        self.attitude_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude', data_class=PoseStamped, queue_size=5)
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', data_class=PoseStamped, queue_size=5)
        self.desired_trajectory = []
        self.actual_trajectory = []
        self.waypoints = []
        self.velocity_list = []
        self.trackingError_list = []
        self.reward_without_trackingError = []
        self.reward_with_trackingError = []
        self.waypoints.append([0, 0, self.all_state[3], sqrt(self.all_state[4]**2+self.all_state[5]**2)])
        self.trackingError = 0
        self.segmentReward = [0]
        self.segmentLength = 5
        self.tracking_const = 3
        self.action_low = np.array([-pi/2, -0.873, 0,-pi/2, -0.873, 0])
        self.action_high = np.array([pi/2, 0.873, 3, pi/2, 0.873, 3])

	def waitStartPos(self, takeoffHeight):
        location = self.gnc.get_current_location()
        z = location.z
        y = location.y
        x = location.x
        r = rospy.Rate(3)
        while (z<takeoffHeight-0.1 or z>takeoffHeight+0.1) and (x<0.5 or x>0.9) and (y<-0.2 or y>0.2):
            r.sleep()
            location = self.gnc.get_current_location()
            z = location.z
            y = location.y
            x = location.x
        self.gnc.initialize_local_frame()


    def makeNewTrajetory(self, start_ep):
        self.listener()
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
            action = np.array(self.model.determinant_action(self.all_state))
            waypoint =self.scaleAction(action)                      
            self.waypoints.append(waypoint[0:4])
            self.waypoints.append(waypoint[4:8])
        self.approximateTrajectoryFunction()


    def approximateTrajectoryFunction(self):

        y = np.array([i[0] for i in self.waypoints])
        z = np.array([i[1] for i in self.waypoints])
        if len(self.waypoints) > 3:
            self.polynomial = interp1d(y, z, kind='cubic')
        else:
            self.polynomial = interp1d(y, z, kind='quadratic')
        self.updateState()
        traj = []
        for index, i in enumerate(self.waypoints):
            traj.append(i)
            if index <= len(self.waypoints)-2:
                temp = []
                dif = (self.waypoints[index+1][0] - i[0])/5
                for k in range(1, 5):
                    temp.append(i[0] + (dif*k))

                for j in temp:
                    traj.append([j, float(self.polynomial(j)), None, None])
        self.desired_trajectory = traj


    def trajectorySegment(self):
        startPitch = self.desired_trajectory[0][2]
        endPitch = self.desired_trajectory[-1][2]
        startVel = self.desired_trajectory[0][3]
        endVel = self.desired_trajectory[-1][3]
        return (startPitch, endPitch, startVel, endVel)


    def follow_trajectory_segment(self, startPitch, endPitch, startVel, endVel):

        r = rospy.Rate(self.segmentLength*6)
        trackingError = 0
        end_y = self.waypoints[-1][0]
        end_z = self.waypoints[-1][1]
        delta_y = 0.02
        count = 0
        self.segmentReward.append(0)
        trackingError = 0
        instant_track_error = 0
        self.listener()
        while not self.y >= end_y and not rospy.is_shutdown():
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

            try:
                delta_z = self.polynomial(self.y + 2*delta_y) - self.z
            except:
                self.reset()
            unit_vec = np.array([2*delta_y/sqrt((2*delta_y)**2 + delta_z**2), delta_z/sqrt((2*delta_y)**2 + delta_z**2)])
            angle = self.all_state[3]
            unit_vec = np.array([cos(angle)*unit_vec[0] - sin(angle)*unit_vec[1], 
                                sin(angle)*unit_vec[0] + cos(angle)*unit_vec[1]])

            speed = startVel + ((self.y - self.start_pos[0])/(end_y - self.start_pos[0]))*(endVel - startVel)
            velocity = unit_vec * speed
            self.velocity_list.append(list(velocity))
            vel = Twist()
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

    def stopDrone(self):
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        self.vel_pub.publish(vel)
        rospy.sleep(2)


    def scaleAction(self, action):
        self.last_action = action
        action_ = [action[0]*self.action_high[0], action[1]*self.action_high[1],
                    (action[2]+1)*self.action_high[2]/2, action[3]*self.action_high[3],
                    action[4]*self.action_high[4], (action[5]+1)*self.action_high[5]/2]

        waypntDis = 0.15
        a = [waypntDis*cos(action_[0]), waypntDis*sin(action_[0]), action_[1], action_[2],
            waypntDis*cos(action_[3]), waypntDis*sin(action_[3]), action_[4], action_[5]]
        return a