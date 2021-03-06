#!/usr/bin/env python

import numpy as np
import random

class ReplayBuffer:
    def __init__(self, capacity):
        self.batch = 0
        self.capacity = capacity
        self.buffer = []
        self.position = 0
    
    def push(self, state, action, reward, next_state, done):
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)
        self.buffer[self.position] = (state, action, reward, next_state, done)
        self.position = (self.position + 1) % self.capacity
    
    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state, done = map(np.stack, zip(*batch))

        return state, action, reward, next_state, done

    def ordered_sample(self, batch_size):
        self.batch = 0
        batch = [self.buffer[self.batch+i] for i in range(batch_size)]
        self.batch += batch_size
        if self.batch >= len(self.buffer):
            self.batch = 0
        state, action, reward, next_state, done = map(np.stack, zip(*batch))

        return state, action, reward, next_state, done

    def __len__(self):
        return len(self.buffer)