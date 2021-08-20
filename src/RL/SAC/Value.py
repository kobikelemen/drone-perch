import torch
import torch.nn as nn
import torch.nn.functional as F
class ValueNetwork(nn.Module):
    def __init__(self, state_dim, hidden_dim, init_w=3e-3):
        super(ValueNetwork, self).__init__()


        # Define Neural Network layers
        self.layer1 = nn.Linear(state_dim, hidden_dim)
        self.layer2 = nn.Linear(hidden_dim, hidden_dim)
        self.layer3 = nn.Linear(hidden_dim, 1)

        # imitialize the weights and bias of model
        self.layer3.weight.data.uniform_(-init_w, init_w)
        self.layer3.bias.data.uniform_(-init_w, init_w)
        
    def forward(self, state):
        state = state.cuda()
        # add activation functions and connect layers
        x = F.relu(self.layer1(state))
        x = F.relu(self.layer2(x))
        x = self.layer3(x)
        return x