import socket
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import random
from collections import deque

# ==========================================
# 1. Hyperparameters (From the Paper)
# ==========================================
BATCH_SIZE = 54
GAMMA = 0.9
LR = 0.001
MEMORY_SIZE = 2000
EPSILON_START = 1.0
EPSILON_END = 0.01
EPSILON_DECAY = 0.995
TAU = 0.005 # Soft update parameter for Target Network

# ==========================================
# 2. Neural Network Architecture
# ==========================================
class DQN(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(DQN, self).__init__()
        # Paper specifies: Two hidden layers with 50 and 30 dimensions
        self.fc1 = nn.Linear(state_dim, 50)
        self.fc2 = nn.Linear(50, 30)
        self.fc3 = nn.Linear(30, action_dim)

    def forward(self, x):
        # Paper specifies: Leaky Relu activation function
        x = F.leaky_relu(self.fc1(x))
        x = F.leaky_relu(self.fc2(x))
        return self.fc3(x)

# ==========================================
# 3. The Reinforcement Learning Agent
# ==========================================
class UoraAgent:
    def __init__(self, state_dim=3, action_dim=3):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.epsilon = EPSILON_START
        
        self.policy_net = DQN(state_dim, action_dim)
        self.target_net = DQN(state_dim, action_dim)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=LR)
        self.memory = deque(maxlen=MEMORY_SIZE)

    def select_action(self, state):
        # Epsilon-Greedy action selection
        if random.random() < self.epsilon:
            return random.randrange(self.action_dim)
        else:
            with torch.no_grad():
                state_tensor = torch.FloatTensor(state).unsqueeze(0)
                q_values = self.policy_net(state_tensor)
                return q_values.argmax().item()

    def store_transition(self, state, action, reward, next_state):
        self.memory.append((state, action, reward, next_state))

    def train_step(self):
        if len(self.memory) < BATCH_SIZE:
            return

        # Sample from replay buffer
        batch = random.sample(self.memory, BATCH_SIZE)
        state_batch = torch.FloatTensor(np.array([t[0] for t in batch]))
        action_batch = torch.LongTensor([[t[1]] for t in batch])
        reward_batch = torch.FloatTensor([t[2] for t in batch])
        next_state_batch = torch.FloatTensor(np.array([t[3] for t in batch]))

        # Compute current Q values
        current_q_values = self.policy_net(state_batch).gather(1, action_batch)

        # Compute next Q values from target network
        with torch.no_grad():
            max_next_q_values = self.target_net(next_state_batch).max(1)[0]
            expected_q_values = reward_batch + (GAMMA * max_next_q_values)

        # Compute Huber loss
        loss = F.smooth_l1_loss(current_q_values.squeeze(), expected_q_values)

        # Optimize the model
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        # Soft update the target network
        for target_param, policy_param in zip(self.target_net.parameters(), self.policy_net.parameters()):
            target_param.data.copy_(TAU * policy_param.data + (1.0 - TAU) * target_param.data)

        # Decay Epsilon
        if self.epsilon > EPSILON_END:
            self.epsilon *= EPSILON_DECAY
    def save_model(self, filename="uora_dqn.pt"):
            torch.save(self.policy_net.state_dict(), filename)
            print(f"[AI] Model weights successfully saved to {filename}")
# ==========================================
# 4. UDP Server Loop
# ==========================================
def parse_state(state_str):
    # Parses "TX=X_RX=Y_CW=Z" into a list of floats: [X, Y, Z]
    parts = state_str.split('_')
    tx = float(parts[0].split('=')[1])
    rx = float(parts[1].split('=')[1])
    cw = float(parts[2].split('=')[1])
    return np.array([tx, rx, cw])

def start_ai_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 9999))
    
    agent = UoraAgent(state_dim=3, action_dim=3)
    
    print("[*] Deep Q-Network Agent Online (Listening on 9999)...")
    
    prev_state = None
    prev_action = None

    while True:
        data, addr = sock.recvfrom(1024) 
        state_str = data.decode('utf-8')
        
        # Skip the initial test ping
        if "TEST" in state_str:
            sock.sendto(b"1", addr)
            continue
            
        current_state = parse_state(state_str)
        current_rx = current_state[1]
        
        # Calculate Reward: Maximize Throughput (RX)
        # If RX increases, AI gets positive reward.
        reward = current_rx 

        # Store transition in Replay Buffer and Train
        if prev_state is not None:
            agent.store_transition(prev_state, prev_action, reward, current_state)
            agent.train_step()

        # Select next action
        action = agent.select_action(current_state)
        
        # Send action to ns-3
        sock.sendto(str(action).encode('utf-8'), addr)
        
        # Update tracking variables
        prev_state = current_state
        prev_action = action
        
        if agent.epsilon <= 0.01 or "simulation_ended_condition": 
             agent.save_model()
        
        print(f"State: TX={int(current_state[0])} RX={int(current_state[1])} CW={int(current_state[2])} | Action: {action} | Epsilon: {agent.epsilon:.3f}")

if __name__ == "__main__":
    start_ai_server()