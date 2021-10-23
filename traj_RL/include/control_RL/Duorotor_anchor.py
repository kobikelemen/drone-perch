import numpy as np
from numpy import arctan, arcsin, arccos, tan, cos, sin, pi, sqrt
from copy import deepcopy
import os
import matplotlib
import torch
import random
from SAC.Model import SAC
from time import time
import matplotlib.pyplot as plt
from Anim import RobotAnimation





class State():
    def __init__(self, pos, vel, ang_vel, ang):
        self.pos = pos
        self.vel = vel
        self.ang_vel = ang_vel
        self.ang = ang
    def set_pos(self, pos):
        self.pos = pos
    def set_vel(self, vel):
        self.vel = vel
    def set_ang_vel(self, ang_vel):
        self.ang_vel = ang_vel


class Duorotor():
    def __init__(self,action_dim =2,state_dim=9,
                initial_state=None,
                model_path='/home/kobi/altered_rl_perch/imitation_learning_models/',
                save_command="on",load_command="off",
                mass=1.0, m2=0.02 , L=3.0, g=98.1, dt=1.0/500, mode='test_ideal'):
        self.all_state= np.array([-2.5, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20], dtype=np.float32)
        self.params = (mass, L, g, m2) # ^ pitch here equals roll in other scripts since use differene coordinates
        self.energy_threshold = self.energy_standard()
        self.energy = 0
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
        self.action_low = np.array([0, 0])
        self.action_high = np.array([100, 100])
        self.batch_size=256
        self.save_command=save_command
        self.load_command=load_command
        self.model_path = model_path
        self.model = SAC(action_dim =len(self.action_high),state_dim=len(self.all_state),hidden_dim=256*2,buffer_size=1000000)
        self.count = 0
        self.model_loader(self.model_path)
        self.reset()
        self.render()

        
    def reset(self):
        (mass, L, g, m2) = self.params
        #start_x = (random.random()*8.5) - 10
        #start_y = (random.random()*4) - 1
        start_x = -2.5
        start_y = 1
        radius = (random.random()*0.3) + 0.15
        self.all_state=np.array([start_x, start_y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2], dtype=np.float32)
        self.last_state = None
        self.last_action = None
        self.total_reward = 0
        self.steps = 0
        self.energy = 0


    def step(self, action, mode):
        self.steps += 1
        self.time_elapsed += self.dt

        Thrust,Omega_dot=self.actor(action)
        self.state_update(Thrust,Omega_dot)

        reward, done = self.observer()

        self.buffer_push(reward,done)

        if mode == "train":
            self.train(done)

        # [x, z, x_dot, z_dot, theta, beta, theta_dot, beta_dot, radius]
        

        if done == 1:
            x_poses = [robot.state_list[i].pos[0]/10 for i in range(0, len(robot.state_list), 45)]
            z_poses = [robot.state_list[i].pos[1]/10 for i in range(0, len(robot.state_list), 45)]
            x_ang = [robot.state_list[i].ang[0] for i in range(0, len(robot.state_list), 45)]
            z_ang = [robot.state_list[i].ang[1] for i in range(0, len(robot.state_list), 45)]
            vel_x = [robot.state_list[i].vel[0]/10 for i in range(0, len(robot.state_list), 45)]
            vel_z = [robot.state_list[i].vel[1]/10 for i in range(0, len(robot.state_list), 45)]
            ang_vel_x = [robot.state_list[i].ang_vel[0]/10 for i in range(0, len(robot.state_list), 45)]
            ang_vel_z = [robot.state_list[i].ang_vel[1]/10 for i in range(0, len(robot.state_list), 45)]
            self.done()
        
        return self.all_state, reward, done

    def state_update(self,Thrust,Omega_dot):
        (mass, L, g, m2) = self.params

        state=self.all_state[0:4]
        velocity=self.all_state[4:8]
        acceleration = np.array([0.0,0.0,0.0,0.0])
        theta = state[2]
        theta_dot = velocity[2]
        thi = state[3]
        self.theta = thi

        matrix = np.array([[cos(theta), sin(theta), L],
            [mass-m2, 0, -m2*L*cos(theta)],
            [0, mass+m2, m2*L*sin(theta)]])

        vector = np.array([-sin(theta)*g,
            (-Thrust*sin(thi))-m2*L*sin(theta)*theta_dot**2,
            Thrust*cos(thi)-(mass+m2)*g-m2*L*cos(theta)*theta_dot**2])
        inv=np.linalg.inv(matrix)
        acceleration[0:3] =  inv.dot(vector) 
        acceleration[3] = Omega_dot

        state += velocity * self.dt + 0.5 * acceleration * self.dt * self.dt
        velocity += acceleration * self.dt

        if self.mode == "test_ideal":
            pos = [state[0], state[1]]
            ang = [state[2], state[3]]
            vel = [velocity[0], velocity[1]]
            ang_vel = [velocity[2], velocity[3]]
            s = State(pos, vel, ang_vel, ang)
            self.state_list.append(s)

        full_state=np.concatenate((state, velocity), axis=-1)
        #print(" velocty: ", self.all_state[4], " ", self.all_state[5])
        self.all_state=np.concatenate((full_state,np.array([self.all_state[-1]])), axis=-1)


    def actor(self,action):
        Actual_action=deepcopy(action+1)*self.action_high/2.0
        Actual_action=self.noise_action(Actual_action)
        self.last_action = action
        I = 1.0*1.0*(2.0)**2/12.0
        self.thrust1=Actual_action[0]
        self.thrust2=Actual_action[1]
        self.thrust=Actual_action[0]+Actual_action[1]
        Omega_dot = (Actual_action[1]-Actual_action[0])/I
        return self.thrust,Omega_dot
         
    def noise_state(self,action,mu=0.0,sigma=1): 
        y =  np.array([(sigma * np.random.randn() + mu)/10 for i in range(len(action))])
        action=action+y
        return action

    def noise_action(self,action,mu=0.0,sigma=1): 
        y =  np.array([(sigma * np.random.randn() + mu)/10 for i in range(len(action))])
        action+= y*self.action_high
        return action

    def rl_action(self):
        print(' =======')
        print(' IN RL ACTION')
        print(' =======')
        state = self.all_state
        self.last_state = deepcopy(state)
        action = self.model.action(state)
        return action

    def rl_determinant_action(self):
        state = self.all_state
        self.last_state = deepcopy(state)
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
        reward = - 0.2*((self.all_state[0]-2.0)**2 + 2*(self.all_state[1]-2.2)**2)#
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
 

    def model_saver(self,model_path):
        if self.save_command=="on":
            torch.save(self.model.value_net.state_dict(), model_path+"value_ignore.dat")
            torch.save(self.model.target_value_net.state_dict(), model_path+"target_ignore.dat")
            torch.save(self.model.q_net.state_dict(), model_path+"qvalue_ignore.dat")
            torch.save(self.model.policy_net.state_dict(), model_path+"policy_ignore.dat")
            print('save model sucessfully')

    def model_loader(self,model_path):
        path = model_path
        print('MODEL_LOADER CALLED')
        if os.path.exists(model_path+"policy.dat") and self.load_command=="on":
            print("PATH EXISTS")
            self.model.value_net.load_state_dict(torch.load(model_path+"value_12th_aug.dat"))
            self.model.value_net.eval()
            self.model.target_value_net.load_state_dict(torch.load(model_path+"target_12th_aug.dat"))
            self.model.target_value_net.eval()
            self.model.q_net.load_state_dict(torch.load(model_path+"qvalue_12th_aug.dat"))
            self.model.q_net.eval()
            self.model.policy_net.load_state_dict(torch.load(model_path+"policy_12th_aug.dat"))
            self.model.policy_net.eval()
            self.model.policy_net_local.load_state_dict(torch.load(model_path+"policy_12th_aug.dat"))
            self.model.policy_net_local.eval()
            print('load model sucessfully')


    def render(self, length=0.3):
        x = self.all_state[0]
        y = self.all_state[1]
        theta = self.all_state[3]
        (mass, L, g, m2) = self.params
        L = 2.0
        # rotor render
        self.x_rotorR = (L / 2.0) * cos(theta) + x
        self.y_rotorR = (L / 2.0) * sin(theta) + y
        self.x_rotorL = -(L / 2.0) * cos(theta) + x
        self.y_rotorL = -(L / 2.0) * sin(theta) + y
        # motor render
        m_x_R = (self.x_rotorR, self.x_rotorR - length * sin(theta))
        m_y_R = (self.y_rotorR, self.y_rotorR + length * cos(theta))
        m_x_L = (self.x_rotorL, self.x_rotorL - length * sin(theta))
        m_y_L = (self.y_rotorL, self.y_rotorL + length * cos(theta))
        # propeller render
        x_RR = self.x_rotorR - length * sin(theta) + length * cos(theta)
        x_LR = self.x_rotorL - length * sin(theta) + length * cos(theta)
        y_RR = self.y_rotorR + length * cos(theta) + length * sin(theta)
        y_LR = self.y_rotorL + length * cos(theta) + length * sin(theta)
        x_RL = self.x_rotorR - length * sin(theta) - length * cos(theta)
        x_LL = self.x_rotorL - length * sin(theta) - length * cos(theta)
        y_RL = self.y_rotorR + length * cos(theta) - length * sin(theta)
        y_LL = self.y_rotorL + length * cos(theta) - length * sin(theta)
        p_x_R = (x_RR, x_RL)
        p_y_R = (y_RR, y_RL)
        p_x_L = (x_LR, x_LL)
        p_y_L = (y_LR, y_LL)
        L = 3.0
        return m_x_R, m_y_R, m_x_L, m_y_L, p_x_R, p_y_R, p_x_L, p_y_L



if __name__ == '__main__':
    mode = 'train'
    robot=Duorotor(save_command="on",load_command="off", mode=mode)
    fig=plt.figure()
    ax=fig.add_subplot(111, aspect='equal', autoscale_on=False,xlim=(-10, 5), ylim=(-7, 5))
    anim = RobotAnimation(robot,mode=mode,fig=fig,ax=ax) # USE model="test_ideal" TO TEST MODEL !!!
    plt.show()
    

