#!/usr/bin/env python

import sys
sys.path.append('/home/kobi/.local/lib/python3.8/site-packages/torch')
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from .Policy import *
from .Q import *
from .Value import *
from .ReplayBuffer import *

class SAC():
    def __init__(self,action_dim =None,state_dim=None,hidden_dim=30,buffer_size=1000000,loss_criteria=nn.MSELoss()):

        self.action_dim = action_dim
        self.state_dim  = state_dim
        self.hidden_dim = hidden_dim
        self.value_net = ValueNetwork(state_dim, hidden_dim)
        self.target_value_net = ValueNetwork(state_dim, hidden_dim)

        for target_param, param in zip(self.target_value_net.parameters(), self.value_net.parameters()):
            target_param.data.copy_(param.data)

        self.q_net = QNetwork(state_dim, action_dim, hidden_dim)
        self.policy_net = PolicyNetwork(state_dim, action_dim, hidden_dim)
        self.policy_net_local = PolicyNetwork(state_dim, action_dim, hidden_dim)
        self.replay_buffer = ReplayBuffer(buffer_size)
        self.loss_criteria=loss_criteria
        self.step = 0

    def action(self,state):
        self.policy_net=self.policy_net.eval()
        action,determinant_action =self.policy_net.get_action(state)
        self.q_net=self.q_net.train()
        self.policy_net=self.policy_net.train()
        self.value_net=self.value_net.train()
        self.target_value_net=self.target_value_net.train()
        return action
    def determinant_action(self,state):
        self.policy_net=self.policy_net.eval()
        action,determinant_action =self.policy_net.get_action(state)
        self.q_net=self.q_net.train()
        self.policy_net=self.policy_net.train()
        self.value_net=self.value_net.train()
        self.target_value_net=self.target_value_net.train()
        return determinant_action




    def soft_q_update(self,batch_size, done_duo,
               gamma=0.99,
               mean_lambda=1e-3,
               sd_lambda=1e-3,
               z_lambda=1e-3,
               tau=1e-2,
               value_lr  = 3e-4,
               q_lr = 3e-4,
               policy_lr = 3e-4
              ):


        state, action, reward, next_state, done = self.replay_buffer.sample(batch_size)
        state      = torch.FloatTensor(state)
        next_state = torch.FloatTensor(next_state)
        action     = torch.FloatTensor(action)
        reward     = torch.FloatTensor(reward).unsqueeze(1)
        done       = torch.FloatTensor(np.float32(done)).unsqueeze(1)
        expected_q = self.q_net(state, action)
        new_action, log_prob, z, mean, log_sd = self.policy_net_local.evaluate(state)
        expected_value = self.value_net(state)
        target_value = self.target_value_net(next_state)
            
        next_q = reward + (1 - done * gamma * target_value)
        new_expected_q = self.q_net(state, new_action)
        next_value = new_expected_q - log_prob
        q_loss = self.loss_criteria(expected_q, next_q.detach())
        v_loss = self.loss_criteria(expected_value, next_value.detach())
    
        policy_optimizer = optim.Adam(self.policy_net_local.parameters(), lr=policy_lr)
        policy_loss=(log_prob-new_expected_q).mean()   # PROBLEM IS WITH new_epected_q !!!!!!!!!!
        policy_optimizer.zero_grad()
        policy_loss.backward()
        policy_optimizer.step()
            
        mean_loss = mean_lambda * mean.pow(2).mean()
        sd_loss  = sd_lambda  * log_sd.pow(2).mean()
        z_loss    = z_lambda    * z**2

        v_optimizer  = optim.Adam(self.value_net.parameters(), lr=value_lr)
        q_optimizer = optim.Adam(self.q_net.parameters(), lr=q_lr)
        q_optimizer.zero_grad()
        q_loss.backward()
        q_optimizer.step()
        v_optimizer.zero_grad()
        v_loss.backward()
        v_optimizer.step()

        for target_param, param in zip(self.target_value_net.parameters(), self.value_net.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

        for target_param, param in zip(self.policy_net.parameters(), self.policy_net_local.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)