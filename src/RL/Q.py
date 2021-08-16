import torch
import torch.nn as nn
import torch.nn.functional as F

class QNetwork(nn.Module):
    def __init__(self, num_inputs, num_actions, hidden_size, init_w=3e-3):
        super(QNetwork, self).__init__()

        #torch.autograd.set_detect_anomaly(True)
        # Define Neural Network layers
        print("QNet")
        self.layer1 = nn.Linear(num_inputs + num_actions, hidden_size)
        self.layer2 = nn.Linear(hidden_size, hidden_size)
        self.layer3 = nn.Linear(hidden_size, 1)
        # initialize the weights and bias of model
        self.layer3.weight.data.uniform_(-init_w, init_w)
        self.layer3.bias.data.uniform_(-init_w, init_w)
    def forward(self, state, action):
        # create input tensor 
        state = state.cuda()
        action = action.cuda()

        x = torch.cat([state, action], 1)
        # add activation functions and connect layers
        x = F.relu(self.layer1(x))
        x = F.relu(self.layer2(x))
        x = self.layer3(x)
        return x