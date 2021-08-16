import numpy as np
from numpy import linalg, arctan, sqrt, arccos, arcsin, tan 
import os
from os import path
import torch
from math import cos, sin, pi, atan2
from copy import deepcopy
from pyquaternion import Quaternion

from vpython import box, sphere, color, vector, rate, canvas, cylinder, arrow, label

from SAC.Model import SAC
import random
class Quadrotor():
	def __init__(self,dt=3.0/1000.0,max_steps=300,batch_size=256*2,model_path = '/home/kobi/catkin_ws/src/drone_perch/src/RL/Weights/',
				mass=1.0,load_mass=0.02,tether_length=3.0,g= 98.1):
		self.steps=0
		self.max_steps=max_steps
		self.parameters = (mass , load_mass, tether_length,g)
		self.dt = dt
		self.tether_length = tether_length
		self.energy = 0.0
		self.load_command = "on"

		self.state = np.array([-2.5,0.0,1.0,
								0.0,0.0,0.0,
								0.0,0.0,
								0.0,0.0,0.0,
								0.0,0.0,0.0,
								0.0,0.0,
								0.2])


		#self.action_low = np.array([0.0, 0.0, 0.0, 0.0])
		self.action_high = np.array([55.0, 55.0])
		
		

		# Model inputs
		self.last_state=None
		self.last_action=None

		self.total_reward=0
		self.episode_num=0
		self.batch_size=batch_size


		self.model_path = model_path

		self.model=SAC(action_dim = 2,state_dim=9,hidden_dim=256*2,buffer_size=1000000)
		# remove old reward data
		self.model_loader(self.model_path)
		self.reset()
		self.render()



	def model_loader(self,model_path):
    	#path = model_path
		if os.path.exists(model_path+"policy_new.dat") and self.load_command=="on":
			self.model.value_net.load_state_dict(torch.load(model_path+"target_value_new.dat"))
			self.model.value_net.eval()
			self.model.target_value_net.load_state_dict(torch.load(model_path+"target_value_new.dat"))
			self.model.target_value_net.eval()
			self.model.q_net.load_state_dict(torch.load(model_path+"qvalue_new.dat"))
			self.model.q_net.eval()
			self.model.policy_net.load_state_dict(torch.load(model_path+"policy_new.dat"))
			self.model.policy_net.eval()


			self.model.policy_net_local.load_state_dict(torch.load(model_path+"policy_local_new.dat"))
			self.model.policy_net_local.eval()
			print('load model sucessfully')
		else:
			print('Saved model not found, initilize the model')






	# ADDED THIS MYSELF
	def model_saver(self,model_path):
		if self.save_command=="on":
			torch.save(self.model.value_net.state_dict(), model_path+"value_new_quad.dat")
			torch.save(self.model.target_value_net.state_dict(), model_path+"target_value_new_quad.dat")
			torch.save(self.model.q_net.state_dict(), model_path+"qvalue_new_quad.dat")
			torch.save(self.model.policy_net.state_dict(), model_path+"policy_new_quad.dat")
			print('save model sucessfully')





	def reset(self):

		
		
		self.state = np.array([-2.5,0.0,1.0,
								0.0,0.0,0.0,
								0.0,0.0+1e-8,
								0.0,0.0,0.0,
								0.0,0.0,0.0,
								0.0,0.0,
								0.2])

		self.last_state = None
		self.last_action=None
		self.total_reward=0
		self.steps=0

		return np.array(self.state)

	def step(self, action):
		(m1 , m2, L, g) = self.parameters 
		self.steps+=1


		
		# resacle the action to fit the real model

		F = (action+1.0)*55/2.0 #self.action_high/2.0



		#F = self.noise_action(F)
		F_fl = F[0]
		F_bl = F[1]
		F_fr = F_fl
		F_br = F_bl


		thrust = F_fl+F_fr+F_bl+F_br
		roll = (F_fr+F_br-F_fl-F_bl)*3
		pitch = -(F_br-F_fr+F_bl-F_fl)*3
		yaw = (F_fl+F_br-F_fr-F_bl)*1.5*0.1


		


		state = self.state

		pos = np.array([state[0], state[1], state[2]]).flatten()               # location in space 
		att = np.array([state[3], state[4], state[5]]).flatten()     # attitude of drone
		
		cable_pos = np.array([state[6], state[7]]).flatten()

		vel = np.array([state[8], state[9], state[10]]).flatten()               # velocity
		att_dot = np.array([state[11], state[12], state[13]]).flatten()
		
		cable_dot = np.array([state[14], state[15]]).flatten()
		radius = np.array([state[16]]).flatten()

		theta = cable_pos[0]
		thetad = cable_dot[0]
		phi = cable_pos[1]
		phid = cable_dot[1]



		qua_att = self.att_converter(att)
		att_quaternion = Quaternion(qua_att)                                   # use quaternion to represent attitude
		thrust_vec = thrust*att_quaternion.rotation_matrix.dot(np.array([0.0, 0.0, 1.0])) # thrust vector
		Fx = thrust_vec[0]
		Fy = thrust_vec[1]
		Fz = thrust_vec[2]



		dynamics = np.array([
			[-m1-m2, 0, 0,  L*m2*sin(theta)*sin(phi),-L*m2*cos(theta)*cos(phi)],
			[0, -m1-m2, 0,  L*m2*cos(theta)*sin(phi),L*m2*sin(theta)*cos(phi)],
			[-2*L*sin(phi)*cos(phi)*thetad*phid, -cos(theta)*sin(phi), -m1-m2, L*sin(phi)**2, -L*m2*sin(phi)],
			[0,-cos(phi), sin(theta)*sin(phi), L*cos(theta)*sin(phi)*cos(phi), L*sin(theta)],
			[cos(phi), 0, cos(theta)*sin(phi), -L*sin(theta)*sin(phi)*cos(phi), L*cos(theta)]
			])

		force = np.array([
			[-Fx + m2*L*(-thetad**2*cos(theta)*sin(phi)-2*thetad*phid*sin(theta)*cos(phi)-phid**2*cos(theta)*sin(phi))],
			[-Fy + m2*L*(thetad**2*sin(theta)*sin(phi)-2*thetad*phid*cos(theta)*cos(phi)+phid**2*sin(theta)*sin(phi))],
			[(m1+m2)*g-Fz+m2*L*(phid**2*cos(phi))-2*L*sin(phi)*cos(phi)*thetad*phid],
			[-g*sin(theta)*sin(phi)+L*thetad**2*sin(theta)*sin(phi)*cos(phi)-2*L*thetad*phid*cos(theta)*cos(phi)**2],
			[-(g*cos(theta)*sin(phi)+L*thetad**2*cos(theta)*sin(phi)*cos(phi)+2*L*thetad*phid*sin(theta)*cos(phi)**2)]
			])

		inv=np.linalg.inv(dynamics)

		acceleration = inv.dot(force).flatten()




		pos += vel * self.dt + 0.5 * acceleration[0:3] * self.dt**2
		vel += acceleration[0:3] * self.dt

		att += np.array([roll,pitch,yaw]) * self.dt**2 + att_dot * self.dt
		att_dot += np.array([roll,pitch,yaw]) * self.dt


		#angle_acc = np.array([0.0,acceleration[3]])
		angle_acc = np.array([acceleration[3],acceleration[4]])
		cable_pos += cable_dot * self.dt + 0.5 * angle_acc * self.dt**2
		cable_dot += angle_acc * self.dt


		self.state = np.array([pos[0], pos[1], pos[2], 
							att[0], att[1], att[2],
							cable_pos[0],cable_pos[1], 
							vel[0], vel[1], vel[2], 
							att_dot[0], att_dot[1], att_dot[2],
							cable_pos[0], cable_dot[1],
							radius[0]])
		State = np.array([pos[0], pos[2], cable_pos[1],-att[1],vel[0], vel[2], cable_dot[1], -att_dot[1],radius[0]])
		
		# THIS LINE WAS CHANGED >>> (ref_generater > observer)
		#self.ref_generator(State)
		#self.observer(State)
		
		reward, done = self.observer(State)

		self.model.replay_buffer.push(self.last_state, self.last_action, reward, State, done)

		if len(self.model.replay_buffer) > self.batch_size:
			self.model.soft_q_update(self.batch_size)

		if done==1 :
			self.episode_num+=1
			print('done episode',self.episode_num,'total_reward is :',self.total_reward)
			self.reset()

		return state, reward, done

	def att_converter(self,att):
		roll = att[0]
		pitch = att[1]
		yaw = att[2]
		cy = cos(yaw * 0.5);
		sy = sin(yaw * 0.5);
		cp = cos(pitch * 0.5);
		sp = sin(pitch * 0.5);
		cr = cos(roll * 0.5);
		sr = sin(roll * 0.5);
		qua_att = [cr * cp * cy + sr * sp * sy, 
					sr * cp * cy - cr * sp * sy, 
					cr * sp * cy + sr * cp * sy, 
					cr * cp * sy - sr * sp * cy]
		return qua_att

    # To calculate the energy threshold for successful perching
	def energy_standard(self, x = 2.0, z = 2.0):
		(m1, m2, L, g) = self.parameters
		target_x = x
		target_y = z
		tree_x = 3.0
		tree_y = 1.0
		tree_diameter = self.state[-1]

		target_angle = arctan((tree_x-target_x)/(target_y-tree_y))-arcsin(tree_diameter/sqrt(tree_diameter**2+(target_x-tree_x)**2+(target_y-tree_y)**2))#np.pi/2
		L1 = L - sqrt((target_x-tree_x)**2+(target_y-tree_y)**2-tree_diameter*tree_diameter)
		L2 = 0.7*L1
		

		temp_L2 = 0.1
		while abs(L2 - temp_L2) > 0.00001:
			temp_L2=L2
			L2 = L1-(arctan(tree_diameter*2/(2*temp_L2))-target_angle+np.pi)*tree_diameter*2/2;
		target_velocity = sqrt(2*g*(sqrt(L2**2+tree_diameter**2)+L1*cos(target_angle)+tree_diameter*sin(target_angle)))
		target_thetad = target_velocity/L
		energy_threshold=((1-cos(target_angle))*g*L+0.5*target_velocity**2)*0.02
		return energy_threshold

	# calculate reward
	def reward_function(self):
		state = self.state
		ref_pos = np.array([state[0]-2.1, state[1], state[2]-2.1,state[9]]).flatten()               # location in space
		ref_angle = np.array([state[3],state[5],state[6],state[11],state[13],state[14]]).flatten()

		reward = -0.3*np.sum(ref_pos**2) - 0.3*np.sum(ref_angle**2)

		return reward

	# Determine the rewards and status of the simulation
	def observer(self,State):
		(m1, m2, L, g) = self.parameters
		done = 0
		reward = self.reward_function()
		Distance = self.distance()
 
		phi = self.state[7]
		phid = self.state[15]
		vx = self.state[8]
		vz = self.state[10]
		v = sqrt((vx+phid*L*cos(phi))**2+(vz+phid*L*sin(phi))**2)
		self.energy = ((1-cos(phi))*g*L+0.5*(v)**2)*0.02

		reward -= abs(Distance-(self.state[-1]-0.1))*0.3 
		reward += self.energy

		if Distance <= self.state[-1]:
			print("Attached")
			standard = self.energy_standard(x = self.state[0], z = self.state[2])
			if self.energy >= standard:
				print("sufficient energy for perching %.3f/%.3f" %(self.energy,standard))
				#self.model_saver(self.model_path)
				reward += 400
				done = 1
				self.stop = True
			else:
				print("not sufficient energy %.3f/%.3f" %(self.energy,standard))
				done = 1


		if self.steps >= 499 :
			done = 1


		self.total_reward += reward

		return reward, done

	# Distance between cable and target 
	def distance(self):
		(m1, m2, L, g) = self.parameters
		state = self.state
		pos = np.array([state[0], state[1], state[2]]).flatten()               # location in space
		theta = state[6]
		phi = state[7]
		P1 = pos + 0.5* np.array([sin(phi)*cos(theta),-sin(phi)*sin(theta),-cos(phi)])
		P2 = pos + (L-2*state[-1]*pi)* np.array([sin(phi)*cos(theta),-sin(phi)*sin(theta),-cos(phi)])

		P0 = np.array([3 ,0 ,1])
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


	def rl_action(self):
		state = self.state
		
		pos = np.array([state[0], state[1], state[2]]).flatten()               # location in space 
		att = np.array([state[3], state[4], state[5]]).flatten()     # attitude of drone
		
		cable_pos = np.array([state[6], state[7]]).flatten()

		vel = np.array([state[8], state[9], state[10]]).flatten()               # velocity
		att_dot = np.array([state[11], state[12], state[13]]).flatten()
		
		cable_dot = np.array([state[14], state[15]]).flatten()
		radius = np.array([state[16]]).flatten()
		State = np.array([pos[0], pos[2], cable_pos[1],-att[1],vel[0], vel[2], cable_dot[1], -att_dot[1],radius[0]])
		
		
		self.last_state = deepcopy(State)



		action = self.model.action(State)
		self.last_action = deepcopy(action)


		
		return action
	def rl_determinant_action(self):
		state = self.state

		pos = np.array([state[0], state[1], state[2]]).flatten()               # location in space 
		att = np.array([state[3], state[4], state[5]]).flatten()     # attitude of drone
		
		cable_pos = np.array([state[6], state[7]]).flatten()

		vel = np.array([state[8], state[9], state[10]]).flatten()               # velocity
		att_dot = np.array([state[11], state[12], state[13]]).flatten()
		
		cable_dot = np.array([state[14], state[15]]).flatten()
		radius = np.array([state[16]]).flatten()
		State = np.array([pos[0], pos[2], cable_pos[1],-att[1],vel[0], vel[2], cable_dot[1], -att_dot[1],radius[0]])
		self.last_state = deepcopy(State)
		

		action = self.model.determinant_action(State)
		self.last_action = deepcopy(action)
		return action




	def render(self, close=False):
		(m1 , m2, L, g) = self.parameters 

		state = self.state
		ref_pos = np.array([0.0,0.0,0.0])

		pos = np.array([state[0], state[1], state[2]]).flatten()               # location in space 
		att = np.array([state[3], state[4], state[5]]).flatten()     # attitude of drone
		cable_pos = np.array([state[6], state[7]]).flatten()
		vel = np.array([state[8], state[9], state[10]]).flatten()               # velocity
		att_dot = np.array([state[11], state[12], state[13]]).flatten()
		cable_dot = np.array([state[14], state[15]]).flatten()
		radius = np.array([state[16]]).flatten()

		theta = cable_pos[0]
		phi = cable_pos[1]


		load_pos = pos + L* np.array([sin(phi)*cos(theta),-sin(phi)*sin(theta),-cos(phi)])

		perch_hubhead = pos + 0.7* np.array([sin(phi)*cos(theta),-sin(phi)*sin(theta),-cos(phi)]) 
		perch_hubtail = pos + (L-2*radius*pi)* np.array([sin(phi)*cos(theta),-sin(phi)*sin(theta),-cos(phi)])
		perch_hubtail = perch_hubtail - perch_hubhead


		qua_att = self.att_converter(att)
		current_quat = Quaternion(qua_att)
		x_axis = current_quat.rotation_matrix.dot(np.array([(2**0.5)/2, (2**0.5)/2, 0.0])) # transfrom the local x_axis to globle coordinate
		y_axis = current_quat.rotation_matrix.dot(np.array([(2**0.5)/2, -(2**0.5)/2, 0.0]))
		z_axis = current_quat.rotation_matrix.dot(np.array([0.0, 0.0, 1.0]))

		tether_vec = load_pos - pos
		time =self.steps*self.dt
# initialize the quadcopter cmodel 

		rate(100)

		return True

	def close(self):
		if self.viewer:
			self.viewer = None

if __name__ == '__main__':
	robotenv=Quadrotor()
	

	count = 0
	while count<100000:
		count += 1 

		action=robotenv.rl_determinant_action() # CHANGE THIS TO rl_action() TO TRAIN I THINK
		state, reward, done=robotenv.step(action)
		robotenv.render()

