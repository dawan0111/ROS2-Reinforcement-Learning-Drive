import asyncio
import os
import time

import rclpy
from rclpy.action import ActionClient
# from reinforcement_learning_drive_interface.action import Drive
from reinforcement_learning_drive_interface.srv import MultiDrive  # 서비스 메시지 임포트
from geometry_msgs.msg import Twist
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import collections
import random
import math

def get_yaw_from_pose(pose):
    q = pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return yaw

# Hyperparameters
learning_rate = 0.01
gamma = 0.98
buffer_limit = 50000
batch_size = 128

class ReplayBuffer():
    def __init__(self):
        self.buffer = collections.deque(maxlen=buffer_limit)
    
    def put(self, transition):
        self.buffer.append(transition)
    
    def sample(self, n):
        mini_batch = random.sample(self.buffer, n)
        s_lst, a_lst, r_lst, s_prime_lst, done_mask_lst = [], [], [], [], []
        
        for transition in mini_batch:
            s, a, r, s_prime, done_mask = transition
            s_lst.append(s)
            a_lst.append([a])
            r_lst.append([r])
            s_prime_lst.append(s_prime)
            done_mask_lst.append([done_mask])

        return torch.tensor(s_lst, dtype=torch.float), torch.tensor(a_lst), \
               torch.tensor(r_lst), torch.tensor(s_prime_lst, dtype=torch.float), \
               torch.tensor(done_mask_lst)
    
    def size(self):
        return len(self.buffer)

class Qnet(nn.Module):
    def __init__(self, input_size):
        super(Qnet, self).__init__()
        self.fc1 = nn.Linear(input_size, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, 64)
        self.fc4 = nn.Linear(64, 32)
        self.fc5 = nn.Linear(32, 10)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        return self.fc5(x)

    def sample_action(self, obs, epsilon):
        q_values = self.forward(obs)
        if random.random() < epsilon:
            action_index = random.randint(0, 9)
        else:
            action_index = q_values.argmax().item()
        return action_index


def train(q, q_target, memory, optimizer):
    for i in range(10):
        s, a, r, s_prime, done_mask = memory.sample(batch_size)

        q_out = q(s)
        q_a = q_out.gather(1, a)  # `a`는 이제 int64 타입의 인덱스 텐서
        max_q_prime = q_target(s_prime).max(1)[0].unsqueeze(1)
        target = r + gamma * max_q_prime * done_mask
        loss = F.smooth_l1_loss(q_a, target)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

def load_model(n_epi, input_size):
    model_path = os.path.join(os.path.expanduser('~'), f"models/gazebo_model_episode_{n_epi}.pt")
    
    # 모델 인스턴스 생성
    q = Qnet(input_size=input_size)
    
    # 모델 파라미터 로드
    q.load_state_dict(torch.load(model_path, weights_only=True))
    # q.eval()
    
    return q

class DriveServiceClient:
    def __init__(self):
        self.node = rclpy.create_node('drive_service_client')
        self.client = self.node.create_client(MultiDrive, 'multi_drive_service')
    
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

    async def send_goal(self, velocity_commands):
        request = MultiDrive.Request()

        for velocity in velocity_commands:
            target_twist = Twist()
            target_twist.linear.x = velocity[0]
            target_twist.angular.z = velocity[1]
            request.target_velocity.append(target_twist)

        future = self.client.call_async(request)
        
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result() is not None:
            result = future.result()
            return result
        else:
            self.node.get_logger().error('Service call failed')
            return None

async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.005)
        await asyncio.sleep(0.001)

async def run(args, loop):
    rclpy.init()

    client = DriveServiceClient()
    spin_task = loop.create_task(spinning(client.node))
    score = 0.0
    print_interval = 20
    actor_lens = 9

    velocity_commands = []
    for _ in range(actor_lens):
        velocity_commands.append([0.0, 0.0])
    results = await loop.create_task(client.send_goal(velocity_commands))
    input_states = [list(state.scan_data) for state in results.state]
    names = [state.name for state in results.state]

    input_size = len(input_states[0])
    print(f"input_size: {input_size}")
    q = load_model(100, input_size)
    q_target = load_model(100, input_size)
    q_target.load_state_dict(q.state_dict())
    memory = ReplayBuffer()

    optimizer = optim.Adam(q.parameters(), lr=learning_rate)
    mode = "TRAIN"
    epsilon = 0.0
    for n_epi in range(500):
        if mode == "TRAIN":
            epsilon = max(0.1, 0.23 - 0.002 * (n_epi / 10))
        done = False
        done_dict = dict()

        while not done:
            velocity_commands = []
            actions = []
            for i, input_state in enumerate(input_states):
                state_tensor = torch.tensor(input_state, dtype=torch.float)
                action_values = q.sample_action(state_tensor, epsilon)

                linear_velocity = 0.5
                angular_velocity = (-0.5 + (action_values * 0.1)) * 1.25

                if names[i] in done_dict:
                    linear_velocity = 0.0
                    angular_velocity = 0.0

                velocity_commands.append([linear_velocity, angular_velocity])
                actions.append(action_values)

            results = await loop.create_task(client.send_goal(velocity_commands))
            state_prime = [list(state.scan_data) for state in results.state]
        
            for i, state in enumerate(results.state):
                if state.name in done_dict:
                    continue
                now_input_state = list(state.scan_data)
                reward = state.score
                actor_done = state.done
                done_mask = 0.0 if actor_done else 1.0
                
                if actor_done:
                    done_dict[state.name] = True

                memory.put((input_states[i], actions[i], reward, now_input_state, done_mask))
                score += reward

            input_states = state_prime

            if len(done_dict) == actor_lens:
                print(f"Done!!")
                break

        velocity_commands = []
        for _ in range(actor_lens):
            velocity_commands.append([0.0, 0.0])
        results = await loop.create_task(client.send_goal(velocity_commands))
        input_states = [list(state.scan_data) for state in results.state]

        time.sleep(2)

        if memory.size() > 500:
            print("Train!!")
            train(q, q_target, memory, optimizer)
        
        if n_epi % print_interval == 0 and n_epi != 0:
            q_target.load_state_dict(q.state_dict())
            print(
                f"n_episode: {n_epi}, score: {score / print_interval:.1f}, n_buffer: {memory.size()}, eps: {epsilon * 100:.1f}%"
            )
            score = 0.0
            model_path = os.path.join(os.path.expanduser('~'), f"models/gazebo_model_episode_{n_epi}.pt")
            model_path2 = os.path.join(os.path.expanduser('~'), f"models/gazebo_model_episode_{n_epi}_target.pt")
            torch.save(q.state_dict(), model_path)
            torch.save(q_target.state_dict(), model_path2)

    spin_task.cancel()
    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass
    rclpy.shutdown()


def main(args=None):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args, loop=loop))

if __name__ == '__main__':
    main()
