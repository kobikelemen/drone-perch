import torch
import torch.nn as nn
import torch.optim as optim


import numpy as np

from SAC.Policy import PolicyNetwork
from SAC.Q import QNetwork
from SAC.Value import ValueNetwork
from SAC.utils.ReplayBuffer import ReplayBuffer

class SAC():
    def __init__(self,action_dim =None,state_dim=None,hidden_dim=256,buffer_size=1000000,loss_criteria=nn.MSELoss()):

        # define model structure
        self.action_dim = action_dim
        self.state_dim  = state_dim
        self.hidden_dim = hidden_dim


        # initialize local and target state value net
        self.value_net        = ValueNetwork(state_dim, hidden_dim)
        self.target_value_net = ValueNetwork(state_dim, hidden_dim)

        # unify target value net
        for target_param, param in zip(self.target_value_net.parameters(), self.value_net.parameters()):
            target_param.data.copy_(param.data)

        # initialize the action state value net and the local and target policy model
        self.q_net = QNetwork(state_dim, action_dim, hidden_dim)
        self.policy_net = PolicyNetwork(state_dim, action_dim, hidden_dim)
        self.policy_net_local = PolicyNetwork(state_dim, action_dim, hidden_dim)

        self.value_net.to('cuda')
        self.target_value_net.to('cuda')
        self.q_net.to('cuda')
        self.policy_net.to('cuda')
        self.policy_net_local.to('cuda')


        # initialize replay buffer
        self.replay_buffer = ReplayBuffer(buffer_size)

        # define loss function
        self.loss_criteria=loss_criteria

        self.step = 0

    def action(self,state):
        # compute action from policy model
        self.policy_net=self.policy_net.eval()
        action,determinant_action =self.policy_net.get_action(state)
        self.q_net=self.q_net.train()
        self.policy_net=self.policy_net.train()
        self.value_net=self.value_net.train()
        self.target_value_net=self.target_value_net.train()
        return action
    def determinant_action(self,state):
        # compute action without stochasticity from policy model
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


        # I added one parameter to soft_q_update .......


        # collect samples from buffer
        state, action, reward, next_state, done = self.replay_buffer.sample(batch_size)
        

        state      = torch.FloatTensor(state)
        next_state = torch.FloatTensor(next_state)
        action     = torch.FloatTensor(action)
        reward     = torch.FloatTensor(reward).unsqueeze(1)
        done       = torch.FloatTensor(np.float32(done)).unsqueeze(1)

        # print('state  ', state)
        # print('next_state  ', next_state)
        # print('action  ', action)
            # print('reward   ', reward)
            # print('done   ', done)

            # compute the action value Q
        expected_q = self.q_net(state, action)

            # compute next action and log(policy distribution)
            #new_action, log_prob, z, mean, log_sd = self.policy_net_local.evaluate(state)
        new_action, log_prob, z, mean, log_sd = self.policy_net_local.evaluate(state)
            #print(' new_action, log_prob, z, mean, log_sd',new_action, log_prob, z, mean, log_sd)
            # the state value for current and next state
        expected_value   = self.value_net(state)
        target_value = self.target_value_net(next_state)
            
            
            # compute action value Q at next state after the current action
        next_q = reward.cuda() + (1 - done.cuda()) * gamma * target_value.cuda()
            # compute expected action value Q at next state after the current action
        new_expected_q = self.q_net(state, new_action)
            #print( 'new_expected_q', new_expected_q)
            # compute state value for next state

        next_value = new_expected_q - log_prob

            #print('new_expected_q  ', new_expected_q.size())
            # main loss functions

        q_loss = self.loss_criteria(expected_q, next_q.detach())
        #print(' q_loss: ', q_loss)

        v_loss = self.loss_criteria(expected_value, next_value.detach())
        #print(' v_loss: ', v_loss)
            #print('log_prob    ', log_prob.size())
        
        
        
        policy_optimizer = optim.Adam(self.policy_net_local.parameters(), lr=policy_lr)

        policy_loss=(log_prob-new_expected_q).mean()   # PROBLEM IS WITH new_epected_q !!!!!!!!!!
        #print(' policy_loss: ', policy_loss)
        policy_optimizer.zero_grad()
        policy_loss.backward()
        policy_optimizer.step()


            
            # additional loss functions
        mean_loss = mean_lambda * mean.pow(2).mean()
        sd_loss  = sd_lambda  * log_sd.pow(2).mean()
        z_loss    = z_lambda    * z**2

            #print('policy_loss before   ', policy_loss)

            #policy_loss = policy_loss + mean_loss + sd_loss + z_loss

            #print("policy loss after   ", policy_loss)

            # Define Optimisation method used adam
        v_optimizer  = optim.Adam(self.value_net.parameters(), lr=value_lr)
        q_optimizer = optim.Adam(self.q_net.parameters(), lr=q_lr)
            
            

            # for name, param in self.policy_net_local.named_parameters():
            #     print('policy_net_local: ',param.device, ' ', name)
            # for name, param in self.policy_net.named_parameters():
            #     print('policy_net: ',param.device, ' ', name)   
            # for name, param in self.q_net.named_parameters():
            #     print('q_net: ',param.device, ' ', name)
            # for name, param in self.value_net.named_parameters():
            #     print('value_net: ',param.device, ' ', name)
            



            # Back-propagation
        q_optimizer.zero_grad()
            #print('q_loss   ', q_loss)
        q_loss.backward()
        q_optimizer.step()
            #print('q_loss after     ', v_loss)

            #print('v_loss  ', v_loss)
        v_optimizer.zero_grad()
        v_loss.backward()
        v_optimizer.step()
            #print('v_loss  after   ', v_loss)

        # if done_duo == 1:
        #     print("policy loss: ", policy_loss)
        #     print("q loss: ", q_loss)
        #     print("v loss: ", v_loss)

            # update the target network
        for target_param, param in zip(self.target_value_net.parameters(), self.value_net.parameters()):
            target_param.data.copy_(
                target_param.data * (1.0 - tau) + param.data * tau
            )

        for target_param, param in zip(self.policy_net.parameters(), self.policy_net_local.parameters()):
            target_param.data.copy_(
                target_param.data * (1.0 - tau) + param.data * tau
            )