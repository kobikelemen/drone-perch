#!/usr/bin/env python

import sys
sys.path.append('/home/kobi/.local/lib/python3.8/site-packages/torch')

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal
import numpy 
class PolicyNetwork(nn.Module):
    def __init__(self, num_inputs, num_actions, hidden_size, init_w=3e-3, log_sd_min=-20, log_sd_max=2):
        super(PolicyNetwork, self).__init__()
        self.log_sd_min = log_sd_min
        self.log_sd_max = log_sd_max
        self.layer1 = nn.Linear(num_inputs, hidden_size)
        self.layer2 = nn.Linear(hidden_size, hidden_size)
        self.layer3 = nn.Linear(hidden_size, hidden_size)
        self.bn2 = nn.BatchNorm1d(hidden_size)
        self.mean_layer = nn.Linear(hidden_size, num_actions)
        self.mean_layer.weight.data.uniform_(-init_w, init_w)
        self.mean_layer.bias.data.uniform_(-init_w, init_w)        
        self.log_sd_layer = nn.Linear(hidden_size, num_actions)
        self.log_sd_layer.weight.data.uniform_(-init_w, init_w)
        self.log_sd_layer.bias.data.uniform_(-init_w, init_w)
        
    def forward(self, state):

        state = state
        x = F.relu(self.layer1(state))
        x = F.relu(self.layer2(x))
        x = F.relu(self.bn2(self.layer3(x)))
        mean    = self.mean_layer(x)
        log_sd = self.log_sd_layer(x)
        log_sd = torch.clamp(log_sd, self.log_sd_min, self.log_sd_max)
        
        return mean, log_sd


    def evaluate(self, state, epsilon=1e-6):
        mean, log_sd = self.forward(state)
        sd = log_sd.exp()
        normal = Normal(0,1)
        z=normal.sample()  
        action = torch.tanh(z*sd+mean)
        log_prob = Normal(mean,sd).log_prob(z*sd+mean) - torch.log(1 - action.pow(2) + epsilon)
        log_prob = log_prob.sum(1, keepdim=True)

        return action, log_prob, z, mean, log_sd
        
    
    def get_action(self, state):
        
        state = torch.FloatTensor(state).unsqueeze(0)
        mean, log_sd = self.forward(state)
        sd = log_sd.exp()        
        normal = Normal(0,1)
        z=normal.sample()        
        action = torch.tanh(z*sd+mean)
        determinant_action=torch.tanh(mean).detach().numpy()
        action  = action.detach().numpy()
        return action[0] , determinant_action[0]

