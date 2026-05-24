import socket
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import random
from collections import deque
from torch.utils.tensorboard import SummaryWriter

device = torch.device("cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu")
print(f"[*] Agent initialized on device: {device}")

# Hyperparameters (From the Paper)
BATCH_SIZE = 54
GAMMA = 0.9
LR = 0.001
MEMORY_SIZE = 2000
EPSILON_START = 1.0
EPSILON_END = 0.01
EPSILON_DECAY = 0.995
TAU = 0.005 

class DQN(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(DQN, self).__init__()
        # Two hidden layers with 50 and 30 dimensions
        self.fc1 = nn.Linear(state_dim, 50)
        self.fc2 = nn.Linear(50, 30)
        self.fc3 = nn.Linear(30, action_dim)

    def forward(self, x):
        # Leaky Relu activation function
        x = F.leaky_relu(self.fc1(x))
        x = F.leaky_relu(self.fc2(x))
        return self.fc3(x)

# The Reinforcement Learning Agent
class UoraAgent:
    def __init__(self, state_dim=3, action_dim=3):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.epsilon = EPSILON_START
        
        # Move networks to detected device
        self.policy_net = DQN(state_dim, action_dim).to(device)
        self.target_net = DQN(state_dim, action_dim).to(device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=LR)
        self.memory = deque(maxlen=MEMORY_SIZE)

    def select_action(self, state):
        # Epsilon-Greedy action selection
        if random.random() < self.epsilon:
            return random.randrange(self.action_dim)
        else:
            with torch.no_grad():
                # Push tensor to device before inference
                state_tensor = torch.FloatTensor(state).unsqueeze(0).to(device)
                q_values = self.policy_net(state_tensor)
                return q_values.argmax().item()

    def store_transition(self, state, action, reward, next_state):
        self.memory.append((state, action, reward, next_state))

    def train_step(self):
        if len(self.memory) < BATCH_SIZE:
            return None # Not enough memory to train yet

        # Sample from replay buffer
        batch = random.sample(self.memory, BATCH_SIZE)
        
        # Push all batches to the designated device
        state_batch = torch.FloatTensor(np.array([t[0] for t in batch])).to(device)
        action_batch = torch.LongTensor([[t[1]] for t in batch]).to(device)
        reward_batch = torch.FloatTensor([t[2] for t in batch]).to(device)
        next_state_batch = torch.FloatTensor(np.array([t[3] for t in batch])).to(device)

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
            
        return loss.item() 

    def save_model(self, filename="uora_dqn.pt"):
        torch.save(self.policy_net.state_dict(), filename)
        print(f"\n[Agent] Model weights successfully saved to {filename}")


def parse_state(state_str):
    # Parses "TX=X_RX=Y_CW=Z" into a list of floats
    parts = state_str.split('_')
    tx = float(parts[0].split('=')[1])
    rx = float(parts[1].split('=')[1])
    cw = float(parts[2].split('=')[1])
    return np.array([tx, rx, cw])

def start_agent_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 9999))
    
    agent = UoraAgent(state_dim=3, action_dim=3)
    writer = SummaryWriter('runs/uora_experiment')
    
    print("[*] Deep Q-Network Agent Online (Listening on 9999)...")
    
    prev_state = None
    prev_action = None
    global_step = 0

    try:
        while True:
            data, addr = sock.recvfrom(1024) 
            state_str = data.decode('utf-8')
            

                
            current_state = parse_state(state_str)
            current_tx = current_state[0]
            current_rx = current_state[1]
            
            # We scale the RX down by 100 so the neural network deals with 
            # numbers like 7.5 instead of 750 (prevents exploding gradients)
            reward = current_rx / 100.0 

            # Store transition and Train
            loss_value = None
            if prev_state is not None:
                agent.store_transition(prev_state, prev_action, reward, current_state)
                loss_value = agent.train_step()

            # Select next action
            action = agent.select_action(current_state)
            
            # Send action to ns-3
            sock.sendto(str(action).encode('utf-8'), addr)
            
            global_step += 1
            writer.add_scalar('Metrics/Demand_TX', current_tx, global_step)
            writer.add_scalar('Metrics/Throughput_RX', current_rx, global_step)
            writer.add_scalar('Metrics/Contention_Window', current_state[2], global_step)
            writer.add_scalar('Agent/Reward', reward, global_step)
            writer.add_scalar('Agent/Epsilon', agent.epsilon, global_step)
            if loss_value is not None:
                writer.add_scalar('Agent/Loss', loss_value, global_step)
            
            # Auto-save every 500 steps
            if global_step % 500 == 0:
                agent.save_model()
            
            # Update tracking variables
            prev_state = current_state
            prev_action = action
            
            print(f"Step {global_step} | TX={int(current_state[0])} RX={int(current_state[1])} CW={int(current_state[2])} | Action: {action} | Epsilon: {agent.epsilon:.3f}")

    except KeyboardInterrupt:
        
        print("\n[*] Training interrupted by user. Saving and shutting down...")
        agent.save_model("uora_dqn_interrupted.pt")
        writer.close()
        sock.close()

if __name__ == "__main__":
    start_agent_server()