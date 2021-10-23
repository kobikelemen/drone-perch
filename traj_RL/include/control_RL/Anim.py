
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from numpy import arctan, arcsin, arccos, tan, cos, sin, pi, sqrt



class RobotAnimation(animation.FuncAnimation):
    def __init__(self, robot,mode="train", fig=None, ax=None, T=10, dt=1.0/500):

        self.robot = robot
        self.mode=mode
        self.fig = fig
        self.ax = ax 
        self.shape_initializer()
        ts = time()
        self.animate(0)
        te = time()
        interval = 1000 * dt - (te - ts)
        super(RobotAnimation, self).__init__(self.fig, self.animate, frames=int(T / dt), interval=interval, blit=True, init_func=self.init, repeat=True)

        
    def init(self):
        sxr,syr,sxl,syl,Rsxr,Rsyr,Rsxl,Rsyl=self.robot.render()
        self.line_rotorR.set_data(Rsxr,Rsyr)
        self.line_rotorL.set_data(Rsxl,Rsyl)
        self.line_supportR.set_data(sxr,syr)
        self.line_supportL.set_data(sxl,syl)
        self.cable.set_data((self.robot.all_state[0],self.robot.all_state[6]),(self.robot.all_state[1],self.robot.all_state[7]))
        self.perching_hub.set_data((self.robot.all_state[0],self.robot.all_state[6]),(self.robot.all_state[1],self.robot.all_state[7]))
        self.line_body.set_data((self.robot.x_rotorR,self.robot.x_rotorL),(self.robot.y_rotorR,self.robot.y_rotorL))
        self.force.set_UVC(self.robot.thrust*cos(self.robot.theta+pi/2),self.robot.thrust*sin(self.robot.theta+pi/2))
        self.force.set_offsets(((self.robot.x_rotorR+self.robot.x_rotorL)/2,(self.robot.y_rotorR+self.robot.y_rotorL)/2))
        self.energy_text.set_text('')
        self.action_text.set_text('')
        self.cable2.set_data(0.0,0.0)
        return self.line_body,self.line_rotorR,self.line_rotorL,self.line_supportR,self.line_supportL, self.time_text, self.energy_text,self.action_text,self.force,self.cable,self.perching_hub,self.cable2#,self.circ,self.cable

    def animate(self, i):
        if self.mode=="test_ideal":
          action=self.robot.rl_determinant_action()
        else:
          action=self.robot.rl_action()
        state, reward, done=self.robot.step(action,mode=self.mode)
        sxr,syr,sxl,syl,Rsxr,Rsyr,Rsxl,Rsyl=self.robot.render()
        (mass, L, g, m2)=self.robot.params
        self.line_rotorR.set_data(Rsxr,Rsyr)
        self.line_rotorL.set_data(Rsxl,Rsyl)
        self.line_supportR.set_data(sxr,syr)
        self.line_supportL.set_data(sxl,syl)
        self.cable.set_data([self.robot.all_state[0], self.robot.all_state[0]+L * sin(self.robot.all_state[2])],[self.robot.all_state[1], self.robot.all_state[1]-L * cos(self.robot.all_state[2])])
        self.perching_hub.set_data([self.robot.all_state[0] + 0.7 * np.sin(self.robot.all_state[2]),self.robot.all_state[0] + (L-2*self.robot.all_state[-1]*pi) * np.sin(self.robot.all_state[2])],
            [self.robot.all_state[1] - 0.7 * np.cos(self.robot.all_state[2]),self.robot.all_state[1] - (L-2*self.robot.all_state[-1]*pi) * np.cos(self.robot.all_state[2])])
        self.line_body.set_data((self.robot.x_rotorR,self.robot.x_rotorL),(self.robot.y_rotorR,self.robot.y_rotorL))
        self.force.set_UVC(5*self.robot.thrust*cos(self.robot.theta+pi/2),5*self.robot.thrust*sin(self.robot.theta+pi/2))
        self.force.set_offsets(((self.robot.x_rotorR+self.robot.x_rotorL)/2,(self.robot.y_rotorR+self.robot.y_rotorL)/2))
        self.time_text.set_text('time = %.1f' % self.robot.time_elapsed)
        self.energy_text.set_text('energy = %.3f' % (self.robot.energy))
        self.action_text.set_text('thrust = %.2f , %.2f ' % (self.robot.thrust1,self.robot.thrust2))
        self.cable2.set_data(0.0,0.0)
        if self.robot.done == 1:
            self.perching_hub.set_data(0.0,0.0)
            self.cable2.set_data([self.robot.all_state[0]-self.robot.all_state[-1]*2, self.robot.all_state[0]-self.robot.all_state[-1]*2],
                [1, 1-(3.0-L-self.robot.all_state[-1]*pi)])
            anim.event_source.stop()
        return self.line_body,self.line_rotorR,self.line_rotorL,self.line_supportR,self.line_supportL, self.time_text, self.energy_text,self.action_text,self.force,self.cable, self.perching_hub,self.cable2#,self.circ,self.cable

    def shape_initializer(self):
        self.ax.grid()
        self.line_body = plt.Line2D((0,0), (0,0), lw=3., 
                  ls='-', marker='.', 
                  markersize=1, 
                  markerfacecolor='r', 
                  markeredgecolor='r', 
                  alpha=0.5)
        self.line_rotorR = plt.Line2D((0,0), (0,0), lw=3., 
                  ls='-', marker='.', 
                  markersize=0.5, 
                  markerfacecolor='r', 
                  markeredgecolor='r', 
                  alpha=0.5)
        self.line_rotorL= plt.Line2D((0,0), (0,0), lw=3., 
                  ls='-', marker='.', 
                  markersize=0.5, 
                  markerfacecolor='r', 
                  markeredgecolor='r', 
                  alpha=0.5)
        self.line_supportR= plt.Line2D((0,0), (0,0), lw=3., 
                  ls='-', marker='.', 
                  markersize=0.5, 
                  markerfacecolor='r', 
                  markeredgecolor='r', 
                  alpha=0.5)
        self.line_supportL = plt.Line2D((0,0), (0,0), lw=3., 
                  ls='-', marker='.', 
                  markersize=0.5, 
                  markerfacecolor='r', 
                  markeredgecolor='r', 
                  alpha=0.5)
        self.cable = plt.Line2D((0,0), (0,0), lw=1.5, 
                  ls='-', marker='o', color='black',
                  markersize=3, 
                  markerfacecolor='r', 
                  markeredgecolor='r', 
                  alpha=1)
        self.perching_hub = plt.Line2D((0,0), (0,0), lw=1.5, 
                  ls='-', marker='o', color='g',
                  markersize=2, 
                  markerfacecolor='g', 
                  markeredgecolor='g', 
                  alpha=1)
        self.cable2 = plt.Line2D((0,0), (0,0), lw=1.5, 
                  ls='-', marker='o', color='black',
                  markersize=3, 
                  markerfacecolor='r', 
                  markeredgecolor='r', 
                  alpha=1)
        #self.circ = plt.Circle((0.5, 0.5), 0.1)
        self.circle=plt.Circle((3, 1), self.robot.all_state[-1], color='black')
        self.target=plt.Circle((2, 2), 0.05, color='black')
        self.ax.add_artist(self.circle)
        self.ax.add_artist(self.target)
        
        self.force= self.ax.quiver(0, 0, 0, 0, units='xy', scale=30, color='blue')
        self.time_text = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes)
        self.energy_text = self.ax.text(0.02, 0.90, '', transform=self.ax.transAxes)
        self.action_text = self.ax.text(0.02, 0.85, '', transform=self.ax.transAxes)
        plt.gca().add_line(self.line_body)
        plt.gca().add_line(self.line_rotorR)
        plt.gca().add_line(self.line_rotorL)
        plt.gca().add_line(self.line_supportR)
        plt.gca().add_line(self.line_supportL)
        plt.gca().add_line(self.cable)
        plt.gca().add_line(self.perching_hub)
        plt.gca().add_line(self.cable2)