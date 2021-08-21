#!/usr/bin/env python

from Train import Duocopter


## ========================IMPORTANT===============================
#      >>>>>>>> MAKE USE OF TOPIC mavros_msgs/Trajectory <<<<<<<<<<
## ================================================================


if __name__ == 'main':

	no_steps = 10
	gnc = py_gnc_functions.gnc_api()
	rospy.init_node('train')
	gnc.wait4connect()
	gnc.wait4start()
	droneStateSub = rospy.Subscriber()
	Quad = Duocopter() # ....
	takeoffHeight = 2
	r = rospy.Rate(100)
	# 1 . initialise model
	# 2.
	
	for ep in no_episodes:

		gnc.takeoff(takeoffHeight)
		Quad.waitStartPos(takeoffHeight)
		Quad.makeNewTrajetory(0)


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

			trajSegment = Quad.trajectorySegment(s)
			Quad.follow_trajectory_segment(trajSegment[0], trajectorySegment[1], trajectorySegment[2], trajectorySegment[3], desired_trajectory, Quad.yDiff, s)
			#Quad.reward()
			Quad.segmentReward += Quad.trackingError
			Quad.buffer_push()
			# 6. follow trajectory until next waypoint
			# 7. compute reward at current position:
			#     - penalise total tracking error
			#     - track final position & velocity of ball for other perching rewards
			# 8. add (s, a, s', r) to replay buffr
			# 9. produce new set of waypoints and trajectory when at time corresponding with next waypoint
            if Quad.mode == 'train':
            	Quad.train()
            Quad.makeNewTrajetory(s)
        # 10. update weights of NN 
		Quad.reset()

		