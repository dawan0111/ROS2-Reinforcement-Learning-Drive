import asyncio
import os
import time

import rclpy
from rclpy.action import ActionClient
# from reinforcement_learning_drive_interface.action import Drive
from reinforcement_learning_drive_interface.srv import Drive  # 서비스 메시지 임포트
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import collections
import random
import math

def get_yaw_from_pose(pose):
    """Pose 메시지에서 yaw(방향) 값을 쿼터니언 수식을 통해 직접 계산합니다."""
    q = pose.orientation
    # 쿼터니언에서 yaw를 계산
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return yaw

# Hyperparameters
learning_rate = 0.005
gamma = 0.98
buffer_limit = 50000
batch_size = 32

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
        self.fc5 = nn.Linear(32, 10)  # 10개의 출력으로 변경하여 각도를 범위 내의 스텝 인덱스로 표현

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        return self.fc5(x)  # 각도의 선택을 위한 10개 노드 출력 (스텝 인덱스)

    def sample_action(self, obs, epsilon):
        q_values = self.forward(obs)
        if random.random() < epsilon:
            # -0.5 ~ 0.5 범위에서 10개의 스텝 중 하나 선택
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


# class DriveActionClient:
#     def __init__(self):
#         self.node = rclpy.create_node('drive_action_client')
#         self._action_client = ActionClient(self.node, Drive, 'drive_action')

#     async def send_goal(self, linear_velocity, angular_velocity):
#         goal_msg = Drive.Goal()
#         goal_msg.target_velocity.linear.x = linear_velocity
#         goal_msg.target_velocity.angular.z = angular_velocity

#         self._action_client.wait_for_server()
        
#         goal_handle = await self._action_client.send_goal_async(goal_msg)

#         if not goal_handle.accepted:
#             self.node.get_logger().info('Goal was rejected')
#             return None
#         # self.node.get_logger().info('Goal accepted, waiting for result')

#         res = await goal_handle.get_result_async()
#         result = res.result
#         status = res.status
#         return result, status  # `result.result`을 통해 실제 결과 데이터 접근


class DriveServiceClient:
    def __init__(self):
        # ROS 2 노드 생성
        self.node = rclpy.create_node('drive_service_client')
        # 서비스 클라이언트 생성
        self.client = self.node.create_client(Drive, 'drive_service')
        
        # 서비스가 활성화될 때까지 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

    async def send_goal(self, linear_velocity, angular_velocity):
        # 요청 메시지 생성
        request = Drive.Request()
        request.target_velocity.linear.x = linear_velocity
        request.target_velocity.angular.z = angular_velocity

        # 비동기 방식으로 서비스 요청
        future = self.client.call_async(request)
        
        # 결과를 기다림
        rclpy.spin_until_future_complete(self.node, future)
        
        # 서비스 응답 확인
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

    # Action Client setup
    client = DriveServiceClient()
    spin_task = loop.create_task(spinning(client.node))

    # RL setup
    input_size = 18  # 예시: scan_data (18차원) + goal_distance + goal_angle
    q = Qnet(input_size=input_size)
    q_target = Qnet(input_size=input_size)
    q_target.load_state_dict(q.state_dict())
    memory = ReplayBuffer()

    optimizer = optim.Adam(q.parameters(), lr=learning_rate)
    score = 0.0
    print_interval = 20

    
    # print("!!")

    result = await loop.create_task(client.send_goal(0.0, 0.0))
    state = list(result.state.scan_data)
    score = result.state.score                

    for n_epi in range(2000):
        epsilon = max(0.1, 0.25 - 0.002 * (n_epi / 10))
        done = False
        while not done:
            
            state_tensor = torch.tensor(state, dtype=torch.float)
            action_values = q.sample_action(state_tensor, epsilon)

            # 소수 첫째 자리로 스텝 계산
            linear_velocity = 2.0
            angular_velocity = (-0.5 + (action_values * 0.1)) * 2

            # Action 실행
            # print("?????")
            result = await loop.create_task(client.send_goal(linear_velocity, angular_velocity))
            result = result.state
            time.sleep(0.001)
            # print(len(list(result.scan_data)))
            
            # 새로운 상태 업데이트
            state_prime = list(result.scan_data)
            reward = result.score  # 피드백에서 스코어를 보상으로 사용
            done = result.done

            done_mask = 0.0 if done else 1.0
            memory.put((state, action_values, reward, state_prime, done_mask))

            state = state_prime
            score += reward

            if done:
                break

        result = await loop.create_task(client.send_goal(0.0, 0.0))
        state = list(result.state.scan_data)

        if memory.size() > 500:
            train(q, q_target, memory, optimizer)
        
        print(n_epi, n_epi % print_interval)
        if n_epi % print_interval == 0 and n_epi != 0:
            q_target.load_state_dict(q.state_dict())
            print(
                f"n_episode: {n_epi}, score: {score / print_interval:.1f}, n_buffer: {memory.size()}, eps: {epsilon * 100:.1f}%"
            )
            score = 0.0
            model_path = os.path.join(os.path.expanduser('~'), f"models/model_episode_{n_epi}.pt")
            torch.save(q.state_dict(), model_path)

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
