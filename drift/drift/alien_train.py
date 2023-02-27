####################################
# Copywrite from caipeide drift_drl
####################################


import argparse
import os
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal
from torch.autograd import grad
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler
from tensorboardX import SummaryWriter


from enum import Enum, auto
import time
import math
import numpy as np
from geometry_msgs.msg import Vector3
from drift_interfaces.msg import Control, Config
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from collections import deque
import pandas as pd


import sys
# import environment
import random
# import pygame
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.patches as patches
from drift_interfaces.msg import Control, Config
from math import sin, cos, tan




#****************************************
# SACAgent.py
#****************************************

#CPU or GPU
device = 'cuda' if torch.cuda.is_available() else 'cpu'
#device = 'cpu'


parser = argparse.ArgumentParser()

parser.add_argument('--tau',  default=0.005, type=float) # target smoothing coefficient
parser.add_argument('--target_update_interval', default=1, type=int)
parser.add_argument('--gradient_steps', default=1, type=int)

parser.add_argument('--learning_rate', default=3e-4, type=int)
parser.add_argument('--gamma', default=0.99, type=int) # discount gamma
parser.add_argument('--capacity', default=400000, type=int) # replay buffer size
parser.add_argument('--iteration', default=100000, type=int) #  num of  games
parser.add_argument('--batch_size', default=512, type=int) # mini batch size
parser.add_argument('--seed', default=1, type=int)

# optional parameters
parser.add_argument('--num_hidden_layers', default=2, type=int)
parser.add_argument('--num_hidden_units_per_layer', default=256, type=int)
parser.add_argument('--sample_frequency', default=256, type=int)
parser.add_argument('--activation', default='Relu', type=str)
parser.add_argument('--render', default=False, type=bool) # show UI or not
parser.add_argument('--log_interval', default=2000, type=int) #
parser.add_argument('--load', default=True, type=bool) # load model

args = parser.parse_args()

min_Val = torch.tensor(1e-7).float().to(device)

class Config:
    def __init__(self, theta, x, y):
        self.theta = theta
        self.x = x
        self.y = y

    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def get_theta(self):
        return self.theta
    
class Control:
    def __init__(self, throttle, steer, brake, hand_brake, reverse, manual_gear_shift, gear):
        self.throttle = throttle
        self.steer = steer
        self.brake = brake
        self.hand_brake = hand_brake
        self.reverse = reverse
        self.manual_gear_shift = manual_gear_shift
        self.gear = gear

    def get_throttle(self):
        return self.throttle
    
    def get_steer(self):
        return self.steer
    
    def get_brake(self):
        return self.brake

class Replay_buffer():
    def __init__(self, capacity,state_dim,action_dim):
        self.capacity = capacity
        self.state_pool = torch.zeros(self.capacity, state_dim).float().to(device)
        self.action_pool = torch.zeros(self.capacity, action_dim).float().to(device)
        self.reward_pool = torch.zeros(self.capacity, 1).float().to(device)
        self.next_state_pool = torch.zeros(self.capacity, state_dim).float().to(device)
        self.done_pool = torch.zeros(self.capacity, 1).float().to(device)
        self.num_transition = 0

    def push(self, s, a, r, s_, d):
        index = self.num_transition % self.capacity
        s = torch.tensor(s).float().to(device)
        a = torch.tensor(a).float().to(device)
        r = torch.tensor(r).float().to(device)
        s_ = torch.tensor(s_).float().to(device)
        d = torch.tensor(d).float().to(device)
        for pool, ele in zip([self.state_pool, self.action_pool, self.reward_pool, self.next_state_pool, self.done_pool],
                           [s, a, r, s_, d]):
            pool[index] = ele
        self.num_transition += 1

    def sample(self, batch_size):
        index = np.random.choice(range(self.capacity), batch_size, replace=False)
        bn_s, bn_a, bn_r, bn_s_, bn_d = self.state_pool[index], self.action_pool[index], self.reward_pool[index],\
                                        self.next_state_pool[index], self.done_pool[index]

        return bn_s, bn_a, bn_r, bn_s_, bn_d

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim ,min_log_std=-20, max_log_std=2):##max and min left to modify
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(state_dim, 512)
        self.fc2 = nn.Linear(512, 256)
        self.mu_head = nn.Linear(256, action_dim)
        self.log_std_head = nn.Linear(256, action_dim)
        self.min_log_std = min_log_std
        self.max_log_std = max_log_std

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        mu = self.mu_head(x)
        log_std_head = self.log_std_head(x)

        log_std_head = torch.clamp(log_std_head, self.min_log_std, self.max_log_std) ##give a resitriction on the chosen action
        return mu, log_std_head


class Critic(nn.Module):
    def __init__(self, state_dim):
        super(Critic, self).__init__()
        self.fc1 = nn.Linear(state_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, 1)

    def forward(self, x):
        
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x


class Q(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Q, self).__init__()

        self.state_dim = state_dim
        self.action_dim = action_dim
        self.fc1 = nn.Linear(state_dim + action_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, 1)

    def forward(self, s, a):
        s = s.reshape(-1, self.state_dim)
        a = a.reshape(-1, self.action_dim)
        x = torch.cat((s, a), -1) # combination s and a
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x


class SACAgent():
    def __init__(self, state_dim = 45, action_dim=21):
        super(SACAgent, self).__init__()

        self.policy_net = Actor(state_dim=state_dim, action_dim = action_dim).to(device)
        self.value_net = Critic(state_dim).to(device)
        self.Target_value_net = Critic(state_dim).to(device)
        
        self.Q_net1 = Q(state_dim, action_dim).to(device)
        self.Q_net2 = Q(state_dim, action_dim).to(device)

        self.policy_optimizer = optim.Adam(self.policy_net.parameters(), lr=args.learning_rate)
        self.value_optimizer = optim.Adam(self.value_net.parameters(), lr=args.learning_rate)
        self.Q1_optimizer = optim.Adam(self.Q_net1.parameters(), lr=args.learning_rate)
        self.Q2_optimizer = optim.Adam(self.Q_net2.parameters(), lr=args.learning_rate)

        self.replay_buffer = Replay_buffer(args.capacity,state_dim,action_dim)
        self.num_transition = 0
        self.num_training = 0
        self.writer = SummaryWriter('./exp-SAC_dual_Q_network')

        self.value_criterion = nn.MSELoss()
        self.Q1_criterion = nn.MSELoss()
        self.Q2_criterion = nn.MSELoss()

        for target_param, param in zip(self.Target_value_net.parameters(), self.value_net.parameters()):
            target_param.data.copy_(param.data)

        self.steer_range = (-0.8,0.8)
        self.throttle_range = (0.6,1.0)

    def select_action(self, state):
        state = torch.FloatTensor(state).to(device)
        mu, log_sigma = self.policy_net(state)
        sigma = torch.exp(log_sigma)
        
        dist = Normal(mu, sigma)
        z = dist.sample()

        steer = float(torch.tanh(z[0,0]).detach().cpu().numpy())
        throttle = float(torch.tanh(z[0,1]).detach().cpu().numpy())
        
        steer = (steer + 1)/2 * (self.steer_range[1] - self.steer_range[0]) + self.steer_range[0]
        throttle = (throttle + 1)/2 * (self.throttle_range[1] - self.throttle_range[0]) + self.throttle_range[0]
        

        return np.array([steer, throttle])


    def test(self, state):
        state = torch.FloatTensor(state).to(device)
        mu, log_sigma = self.policy_net(state)
       
        action = mu

        steer = float(torch.tanh(action[0,0]).detach().cpu().numpy())
        throttle = float(torch.tanh(action[0,1]).detach().cpu().numpy())

        steer = (steer + 1)/2 * (self.steer_range[1] - self.steer_range[0]) + self.steer_range[0]
        throttle = (throttle + 1)/2 * (self.throttle_range[1] - self.throttle_range[0]) + self.throttle_range[0]


        return np.array([steer, throttle])

    def evaluate(self, state):
        batch = state.size()[0]
        batch_mu, batch_log_sigma = self.policy_net(state)
        batch_sigma = torch.exp(batch_log_sigma)
        dist = Normal(batch_mu, batch_sigma)
        noise = Normal(0, 1)

        z = noise.sample()

        action = torch.tanh(batch_mu + batch_sigma * z.to(device))
        
        log_prob = dist.log_prob(batch_mu + batch_sigma * z.to(device)) - torch.log(1 - action.pow(2) + min_Val)

        log_prob_0 = log_prob[:,0].reshape(batch,1)
        log_prob_1 = log_prob[:,1].reshape(batch,1)
        log_prob = log_prob_0 + log_prob_1

        return action, log_prob, z, batch_mu, batch_log_sigma


    def update(self):
        if self.num_training % 500 == 0:
            print("**************************Train Start************************")
            print("Training ... \t{} times ".format(self.num_training))

        for _ in range(args.gradient_steps):
            bn_s, bn_a, bn_r, bn_s_, bn_d = self.replay_buffer.sample(args.batch_size)
            
            target_value = self.Target_value_net(bn_s_)
            next_q_value = bn_r + (1 - bn_d) * args.gamma * target_value

            excepted_value = self.value_net(bn_s)
            excepted_Q1 = self.Q_net1(bn_s, bn_a)
            excepted_Q2 = self.Q_net2(bn_s, bn_a)
            sample_action, log_prob, z, batch_mu, batch_log_sigma = self.evaluate(bn_s)
            excepted_new_Q = torch.min(self.Q_net1(bn_s, sample_action), self.Q_net2(bn_s, sample_action))
            next_value = excepted_new_Q - log_prob
            
            V_loss = self.value_criterion(excepted_value, next_value.detach()).mean()  # J_V

            # Dual Q net
            Q1_loss = self.Q1_criterion(excepted_Q1, next_q_value.detach()).mean() # J_Q
            Q2_loss = self.Q2_criterion(excepted_Q2, next_q_value.detach()).mean()

            pi_loss = (log_prob - excepted_new_Q).mean() # according to original paper

            self.writer.add_scalar('Loss/V_loss', V_loss, global_step=self.num_training)
            self.writer.add_scalar('Loss/Q1_loss', Q1_loss, global_step=self.num_training)
            self.writer.add_scalar('Loss/Q2_loss', Q2_loss, global_step=self.num_training)
            self.writer.add_scalar('Loss/policy_loss', pi_loss, global_step=self.num_training)

            # mini batch gradient descent
            self.value_optimizer.zero_grad()
            V_loss.backward(retain_graph=True)
            nn.utils.clip_grad_norm_(self.value_net.parameters(), 0.5)
            self.value_optimizer.step()

            self.Q1_optimizer.zero_grad()
            Q1_loss.backward(retain_graph = True)
            nn.utils.clip_grad_norm_(self.Q_net1.parameters(), 0.5)
            self.Q1_optimizer.step()

            self.Q2_optimizer.zero_grad()
            Q2_loss.backward(retain_graph = True)
            nn.utils.clip_grad_norm_(self.Q_net2.parameters(), 0.5)
            self.Q2_optimizer.step()

            self.policy_optimizer.zero_grad()
            pi_loss.backward(retain_graph = True)
            nn.utils.clip_grad_norm_(self.policy_net.parameters(), 0.5)
            self.policy_optimizer.step()

            # update target v net update
            for target_param, param in zip(self.Target_value_net.parameters(), self.value_net.parameters()):
                target_param.data.copy_(target_param * (1 - args.tau) + param * args.tau)

            self.num_training += 1

    def save(self,epoch, capacity):
        os.makedirs('./SAC_model_' +str(capacity) , exist_ok=True)
        torch.save(self.policy_net.state_dict(), './SAC_model_' +str(capacity)+ '/policy_net_' + str(epoch) + '.pth')
        torch.save(self.value_net.state_dict(), './SAC_model_'  +str(capacity)+ '/value_net_'+ str(epoch) +'.pth')
        torch.save(self.Q_net1.state_dict(), './SAC_model_' +str(capacity)+'/Q_net1_' + str(epoch) + '.pth')
        torch.save(self.Q_net2.state_dict(), './SAC_model_' +str(capacity)+'/Q_net2_' + str(epoch) + '.pth')
        print("====================================")
        print("Model has been saved...")
        print("====================================")

    def load(self, epoch, capacity):
        dir = './SAC_model_' + str(capacity) + '/'
        self.policy_net.load_state_dict(torch.load( dir + 'policy_net_' + str(epoch) + '.pth'))
        self.value_net.load_state_dict(torch.load( dir + 'value_net_'+ str(epoch) + '.pth'))
        self.Q_net1.load_state_dict(torch.load( dir + 'Q_net1_' + str(epoch) + '.pth'))
        self.Q_net2.load_state_dict(torch.load( dir + 'Q_net2_' + str(epoch) + '.pth'))
        print("====================================")
        print("model has been loaded...")
        print("====================================")






#****************************************
# Environment.py
#****************************************
step_T_bound = (0.6,1)		# Boundary of throttle values
step_S_bound = (-0.8,0.8)	# Boundary of the steering angle values

class start_location:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y

class start_rotation:
    def __init__(self, pitch, yaw, roll):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll
    
    def get_yaw(self):
        return self.yaw

class Transform:
    def __init__(self, x, y, z, pitch, yaw, roll):
        super().__init__(x, y, z)
        super().__init__(pitch, yaw, roll)

class vehicle_control(Enum):
	throttle = auto()
	steer = auto()
	brake = auto()
	hand_brake = False
	reverse = False
	manual_gear_shift = False
	gear = auto()


class Environment():
    def __init__(self, throttleSize=4, steerSize=9, traj_num = 0, collectFlag = False, model='dqn', vehicleNum=1):
        super(Environment, self).__init__()

        self.declare_parameter('f_wheel_friction_mu', 0.01)
        self.f_wheel_friction_mu = self.get_parameter('f_wheel_friction_mu').get_parameter_value().double_value
        self.declare_parameter('f_wheel_friction_mu2', 0.01)
        self.f_wheel_friction_mu2 = self.get_parameter('f_wheel_friction_mu2'
                                               ).get_parameter_value().double_value
        self.declare_parameter('r_wheel_friction_mu', 0.01)
        self.r_wheel_friction_mu = self.get_parameter('r_wheel_friction_mu').get_parameter_value().double_value

        self.declare_parameter('r_wheel_friction_mu2', 0.01)
        self.r_wheel_friction_mu2 = self.get_parameter('r_wheel_friction_mu2').get_parameter_value().double_value

        # publisher to control
        # self.control_pub = self.create_publisher(Control, 'control', 10)

        # publisher to cmd_vel
        # self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # publisher to config
        # self.config_pub = self.create_publisher(Config, "config", 10)

        # subscriber to cmd_vel
        # self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.vel_cmd_callback, 10)

        # publisher to config
        # self.config_sub = self.create_sublisher(Config, 'config', 10)

        # # create a broadcaster that will repeatedly publish to /tf
        # self.broadcaster = TransformBroadcaster(self)

        # # create a listener for brick position
        # self.tf_buffer = Buffer()
        # self.tf_brick_listener = TransformListener(self.tf_buffer, self)

        if not collectFlag:
            start_location.x = self.route[0,0]
            start_location.y = self.route[0,1]
            start_location.z = 0.1
            start_rotation.pitch = 0
            start_rotation.yaw = self.route[0,2]
            start_rotation.roll = 0
        else:
            # start_location = carla.Location()
            # start_rotation = carla.Rotation()
            start_location.x = 0.0
            start_location.y = 0.0
            start_location.z = 0.0
            start_rotation.pitch = 0.0
            start_rotation.yaw = 0.0
            start_rotation.roll = 0.0

        # self.start_point = carla.Transform(location = start_location, rotation = start_rotation)  # type : Transform (location, rotation)
        self.start_point = Transform(start_location, start_rotation)

        # self.minDis = 0
        self.collectFlag = collectFlag
        self.traj_drawn_list = []

        vehicle_control.throttle = 1
        vehicle_control.steer = 0.0
        vehicle_control.brake = 0.0
        vehicle_control.hand_brake = False
        vehicle_control.reverse = False
        vehicle_control.manual_gear_shift = False
        vehicle_control.gear = 0
        
        self.destinationFlag = False
        self.away = False
        self.collisionFlag = False
        self.waypoints_ahead = []
        self.waypoints_neighbor = [] 
        self.steer_history = deque(maxlen=20)
        self.throttle_history = deque(maxlen=20)
        self.velocity_local = []
        self.model = model
        if model == 'dqn':
            self.step_T_pool = [step_T_bound[0]]
            self.step_S_pool = [step_S_bound[0]]
            t_step_rate = (step_T_bound[1]- step_T_bound[0])/throttleSize
            s_step_rate = (step_S_bound[1]- step_S_bound[0])/steerSize
            for i in range(throttleSize):
                self.step_T_pool.append(self.step_T_pool[-1]+t_step_rate)
            for i in range(steerSize):
                self.step_S_pool.append(self.step_S_pool[-1]+s_step_rate)
            print(self.step_T_pool)
            print(self.step_S_pool)
            self.tStateNum = len(self.step_T_pool)
            self.sStateNum = len(self.step_S_pool)

        self.e_heading = 0
        self.e_d_heading = 0
        self.e_dis = 0
        self.e_d_dis = 0
        self.e_slip = 0
        self.e_d_slip = 0
        self.e_vx = 0
        self.e_d_vx = 0
        self.e_vy = 0
        self.e_d_vy = 0

        self.tg = 0
        self.clock_history = 0 # pop the current location into self.waypoints_history every 0.2s

        self.k_heading = 0.1

        self.waypoints_ahead_local = []
        self.waypoints_history = deque(maxlen=5)
        self.waypoints_history_local = []

        self.last_steer = 0.0
        self.last_throttle = 0.0

        self.tire_friction_array = np.arange(3,4.1,0.1) # [3,4], 11D
        self.mass_array = np.arange(1700,1910,50) # array([1700, 1750, 1800, 1850, 1900])

        # self.ori_physics_control = self.world.player.get_physics_control()
        # get physics here from xacro file
        self.wheel_fl = self.f_wheel_friction_mu + self.f_wheel_friction_mu2
        self.wheel_fr = self.f_wheel_friction_mu + self.f_wheel_friction_mu2
        self.wheel_rl = self.r_wheel_friction_mu + self.r_wheel_friction_mu2
        self.wheel_rr = self.r_wheel_friction_mu + self.r_wheel_friction_mu2

        # self.world.world.set_weather(carla.WeatherParameters.ClearNoon)

        self.odom_base = TransformStamped()
        self.odom_base.header.frame_id = "odom"
        self.odom_base.child_frame_id = "base_link"
        self.done = 0

        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0
        self.vel.angular.w = 0.0

        self.config = Config()
        self.config.x = 0.0
        self.config.y = 0.0
        self.config.theta = 0.0

    def vel_cmd_callback(self, msg):
        self.vel.linear = msg.linear
        self.vel.angular = msg.angular

    # def timer(self):
    #     self.odom_base.header.stamp = self.get_clock().now().to_msg()
    #     self.broadcaster.sendTransform(self.odom_base)


    def refreshRoute(self, traj_num, vehicleNum):
        if vehicleNum == 1:
            traj = pd.read_csv('ref_trajectory/traj_' + str(traj_num) + '.csv')
        else:
            traj = pd.read_csv('ref_trajectory/traj_different_vehicles/' + str(vehicleNum) + '.csv')
            self.route = traj.values
            self.route_x = self.route[:,0]
            self.route_y = self.route[:,1]
            self.route_length = np.zeros(self.route.shape[0])
        for i in range(1, self.route.shape[0]):
            dx = self.route_x[i-1] - self.route_x[i]
            dy = self.route_y[i-1] - self.route_y[i]
            self.route_length[i] = self.route_length[i-1] + np.sqrt(dx * dx + dy * dy)



    def step(self, actionID = 4, steer = 0, throttle=0, manual_control = False):
		# apply the computed control commands, update endFlag and return state/reward
        if not manual_control:
            if self.model == 'dqn':
                self.control = self.getAction(actionID=actionID)
            else:
                self.control = self.getAction(steer = steer,throttle = throttle)

            if self.model == 'sac':
                self.control.steer = 0.1*self.control.steer + 0.9*self.last_steer
                self.control.throttle = 0.3*self.control.throttle + 0.7*self.last_throttle
            if self.model == 'ddpg':
                self.control.steer = 0.6*self.control.steer + 0.4*self.last_steer
                self.control.throttle = 0.3*self.control.throttle + 0.7*self.last_throttle

            self.last_steer = self.control.steer
            self.last_throttle = self.control.throttle

            # TODO publish self.control
            control = Control()
            control.throttle = self.control.throttle
            control.steer = self.control.steer
            self.control_pub.publish(control)

            # self.world.player.apply_control(self.control)
            self.steer_history.append(self.control.steer)
            self.throttle_history.append(self.control.throttle)
            time.sleep(0.05)

		
        if manual_control and not self.collectFlag:
            # can i get just from stored variables or do other nodes change? in which case make a different subscriber
            # control = self.world.player.get_control()
            self.steer_history.append(control.steer)
            self.throttle_history.append(control.throttle)
            time.sleep(0.05)

        newState = self.getState()

        if not self.collectFlag :
            reward = self.getReward(newState, self.steer_history, self.throttle_history)

            self.collisionFlag = self.collisionDetect()

            return newState, reward, self.collisionFlag, self.destinationFlag, self.away, self.control

        else:
            # can i get just from stored variables or do other nodes change? in which case make a different subscriber
            # control = self.world.player.get_control()
            return newState, control


    def reset(self, traj_num = 0, collect_x = 0, collect_y = 0, collect_yaw = 0,
              randomPosition = False, testFlag = False, test_friction = 3.5,
              test_mass = 1800.0, differentFriction=False, differentVehicles=False):
        # random change the tire friction and vehicle mass:
        if not testFlag:
            index_friction = np.random.randint(0,self.tire_friction_array.shape[0])
            index_mass = np.random.randint(0,self.mass_array.shape[0])


            self.tire_friction = self.tire_friction_array[index_friction]
            self.mass = self.mass_array[index_mass]
        else:
            self.tire_friction = test_friction
            self.mass = test_mass

        if not differentFriction:
            self.wheel_fl.tire_friction = self.f_wheel_friction_mu + self.f_wheel_friction_mu2
            self.wheel_fr.tire_friction = self.f_wheel_friction_mu + self.f_wheel_friction_mu2
            self.wheel_rl.tire_friction = self.r_wheel_friction_mu + self.r_wheel_friction_mu2
            self.wheel_rr.tire_friction = self.r_wheel_friction_mu + self.r_wheel_friction_mu2
        else:
            self.wheel_fl.tire_friction = 2.8
            self.wheel_fr.tire_friction = 2.8
            self.wheel_rl.tire_friction = 4.2
            self.wheel_rr.tire_friction = 4.2

        # wheels = [self.wheel_fl, self.wheel_fr, self.wheel_rl, self.wheel_rr]

        # self.ori_physics_control.wheels = wheels
        # if not differentVehicles:
        #     self.ori_physics_control.mass = float(self.mass)

        # self.world.player.apply_physics_control(self.ori_physics_control)
        time.sleep(0.5)

        # detect:
        # physics = self.world.player.get_physics_control()
        # print('firction: {}, mass: {}'.format(physics.wheels[0].tire_friction, physics.mass))
        # print('center of mass: ', physics.center_of_mass.x, physics.center_of_mass.y, physics.center_of_mass.z)

        if not self.collectFlag:
            self.refreshRoute(traj_num, self.vehicleNum)
            if not randomPosition:
                start_location.x = self.route[0,0]
                start_location.y = self.route[0,1]
                start_location.z = 0.1
                # start_rotation = carla.Rotation(pitch = 0, yaw = self.route[0,2], roll = 0)
                start_rotation.pitch = 0
                start_rotation.yaw = self.route[0,2]
                start_rotation.roll = 0
                velocity_local = [10,0]  # 5m/s
                angular_velocity = Vector3()

            else:
                k = np.random.randint(0,self.route.shape[0] - 100)
                # start_location = carla.Location(x = self.route[k,0], y = self.route[k,1], z = 0.1)
                start_location.x = self.route[k,0]
                start_location.y = self.route[k,1]
                start_location.z = 0.1
                # start_rotation = carla.Rotation(pitch = 0, yaw = self.route[k,2], roll = 0)
                start_rotation.pitch = 0
                start_rotation.yaw = self.route[k,2]
                start_rotation.roll = 0
                velocity_local = [10, 0] 
                # angular_velocity = carla.Vector3D(z = self.route[k,6])
                angular_velocity = Vector3()
        else:
            # start_location = carla.Location(x = collect_x, y=collect_y)
            start_location.x = collect_x
            start_location.y = collect_y
            # start_rotation = carla.Rotation(yaw = collect_yaw)
            start_rotation.yaw = collect_yaw

        self.start_point = Transform(location = start_location, rotation = start_rotation)  # type : Transform (location, rotation)
        ego_yaw = self.start_point.rotation.yaw

        if not self.collectFlag:
            if traj_num not in self.traj_drawn_list:
                self.drawPoints()
                self.traj_drawn_list.append(traj_num)

        ego_yaw = ego_yaw/180.0 * 3.141592653
        transformed_world_velocity = self.velocity_local2world(velocity_local, ego_yaw)

        # TODO need publishers and subscribers or services and clients
        # self.odom_base.transform.translation.x = self.start_point.location.x
        # self.odom_base.transform.translation.y = self.start_point.location.y
        # self.odom_base.transform.translation.z = self.start_point.location.z

        # q = Quaternion()
        # # if needed set roll and pitch to 0.0
        # q = euler_to_quaternion(self.start_point.rotation.roll, self.start_point.rotation.pitch, self.start_point.rotation.yaw)
        # self.odom_base.transform.rotation.x = q.x
        # self.odom_base.transform.rotation.y = q.y
        # self.odom_base.transform.rotation.z = q.z
        # self.odom_base.transform.rotation.w = q.w
        # # self.world.player.set_transform(self.start_point)
        # self.broadcaster.sendTransform(self.odom_base)

        config = Config()
        config.w = self.start_point.rotation.yaw
        config.x = self.start_point.location.x
        config.y = self.start_point.location.y
        self.config_pub.publish(config)

        vel = Twist()
        vel.linear = transformed_world_velocity
        vel.angular = angular_velocity
        self.cmd_vel_pub.publish(vel)
        # self.world.player.set_velocity(transformed_world_velocity)
        # self.world.player.set_angular_velocity(angular_velocity)

        control = Control()
        control.throttle = 0.0
        control.steer = 0.0
        self.control_pub.publish(control)
        # self.world.player.apply_control(vehicle_control)

        # self.world.collision_sensor.history = []
        self.away = False
        self.endFlag = False
        self.steer_history.clear()
        self.throttle_history.clear()
        self.waypoints_neighbor = []
        self.waypoints_ahead = []

        self.waypoints_ahead_local = [] # carla.location 10pts
        self.waypoints_history.clear()  # carla.location  5pts
        self.waypoints_history_local = []
        self.destinationFlag = False

        self.last_steer = 0.0
        self.last_throttle = 0.0

        self.drived_distance = 0

        return 0
    
    def config_callback(self, msg):
        self.config.x = msg.x
        self.config.y = msg.y
        self.config.theta = msg.theta

    def getState(self):
        # TODO need publishers and subscribers or services and clients
        # location = self.world.player.get_location()
        location = Transform()
        location.x = self.config.x
        location.y = self.config.y

        # angular_velocity = self.world.player.get_angular_velocity()
        angular_velocity = self.vel.angular

        # transform = self.world.player.get_transform()

        ego_yaw = self.config.yaw
        if ego_yaw < 0:
            ego_yaw += 360
        if ego_yaw > 360:
            ego_yaw -= 360
        ego_yaw = ego_yaw/180.0 * 3.141592653

        self.getNearby() # will update self.minDis

        self.getLocalHistoryWay(location, ego_yaw)
        self.getLocalFutureWay(location, ego_yaw)

        self.velocity_world2local(ego_yaw) # will update self.velocity_local

        ego_yaw = ego_yaw/3.141592653 * 180
        if ego_yaw > 180:
            ego_yaw = -(360-ego_yaw)

        if self.collectFlag:
            state = [location.x, location.y, ego_yaw, self.velocity_local[0], self.velocity_local[1], self.velocity_local[2], angular_velocity.z]

        self.control = self.world.player.get_control()
        steer = self.control.steer
        ct = time.time()
        if ct - self.clock_history > 0.3:
            self.waypoints_history.append(np.array([location.x, location.y, steer, self.velocity_local[2]]))
            self.clock_history = ct

            return state

        else:
            dt = time.time() - self.tg
            self.e_d_dis = (self.minDis - self.e_dis) / dt
            self.e_dis = self.minDis

            if self.e_dis > 15:
                self.away = True

        # error of heading:
        # 1. calculate the abs
        way_yaw = self.waypoints_ahead[0,2]
        # 2. update the way_yaw based on vector guidance field:
        vgf_left = self.vgf_direction(location)  
        # 3. if the vehicle is on the left of the nearst waypoint, according to the heading of the waypoint
        if vgf_left:
            way_yaw = math.atan(self.k_heading * self.e_dis)/3.141592653*180 + way_yaw
        else:
            way_yaw = -math.atan(self.k_heading * self.e_dis)/3.141592653*180 + way_yaw
        if way_yaw > 180:
            way_yaw = -(360-way_yaw)
        if way_yaw < -180:
            way_yaw += 360
        
        if ego_yaw*way_yaw > 0:
            e_heading = abs(ego_yaw - way_yaw)
        else:
            e_heading = abs(ego_yaw) + abs(way_yaw)
        if e_heading > 180:
            e_heading = 360 - e_heading
        # considering the +-:
        # waypoint to the vehicle, if clockwise, then +
        hflag = 1
        if ego_yaw*way_yaw > 0:
            if ego_yaw > 0:
                if abs(way_yaw) < abs(ego_yaw):
                    flag = -1
                else:
                    hflag = 1
            if ego_yaw < 0:
                if abs(way_yaw) < abs(ego_yaw):
                    hflag = 1
                else:
                    hflag = -1
        else:
            if ego_yaw > 0:
                t_yaw = ego_yaw-180
                if way_yaw > t_yaw:
                    hflag = -1
                else:
                    hflag = 1
            else:
                t_yaw = ego_yaw + 180
                if way_yaw > t_yaw:
                    hflag = -1
                else:
                    hflag = 1
        e_heading = e_heading * hflag
        if e_heading * self.e_heading > 0:
            if e_heading > 0:
                self.e_d_heading = (e_heading - self.e_heading)/dt
            else:
                self.e_d_heading = -(e_heading - self.e_heading)/dt
        else:
            self.e_d_heading = (abs(e_heading) - abs(self.e_heading)) / dt
        
        self.e_heading = e_heading
        
        e_slip = self.velocity_local[2] - self.waypoints_ahead[0,5]
        self.e_d_slip = (e_slip - self.e_slip)/dt
        self.e_slip = e_slip
        
        e_vx = self.velocity_local[0] - self.waypoints_ahead[0,3]
        self.e_d_vx = (e_vx - self.e_vx)/dt
        self.e_vx = e_vx
        
        e_vy = self.velocity_local[1] - self.waypoints_ahead[0,4]
        self.e_d_vy = (e_vy - self.e_vy)/dt
        self.e_vy = e_vy
        
        self.control = self.world.player.get_control()
        
        steer = self.control.steer
        throttle = self.control.throttle
        
        ct = time.time()
        if ct - self.clock_history > 0.2:
            self.waypoints_history.append(np.array([location.x, location.y, steer, self.velocity_local[2]]))
            self.clock_history = ct
        
        vx = self.velocity_local[0]
        vy = self.velocity_local[1]
        e_d_slip = self.e_d_slip
        if math.sqrt(vx*vx + vy*vy) < 2: # if the speed is too small we ignore the error of slip angle
            e_slip = 0
            e_d_slip = 0
        
        state = [steer, throttle , self.e_dis, self.e_d_dis, self.e_heading, self.e_d_heading, e_slip, e_d_slip,
        self.e_vx, self.e_d_vx, self.e_vy, self.e_d_vy]
        state.extend([k[0] for k in self.waypoints_ahead_local]) #x
        state.extend([k[1] for k in self.waypoints_ahead_local]) #y
        state.extend([k[2] for k in self.waypoints_ahead_local]) #slip
        self.tg = time.time()
        
        
        
        return state
	
    def getReward(self, state, steer_history, throttle_history):
        e_dis = state[2]
        e_slip = state[6]
        e_heading = state[4]
        std_steer = np.array(steer_history)
        std_steer = std_steer.std()
        
        std_throttle = np.array(throttle_history)
        std_throttle = std_throttle.std()
        
        r_dis = np.exp(-0.5*e_dis)
        
        if abs(e_heading)<90:
            r_heading = np.exp(-0.1*abs(e_heading))
        elif (e_heading)>= 90:
            r_heading = -np.exp(-0.1*(180-e_heading))
        else:
            r_heading = -np.exp(-0.1*(e_heading+180))
        
        if abs(e_slip)<90:
            r_slip = np.exp(-0.1*abs(e_slip))
        elif (e_slip)>= 90:
            r_slip = -np.exp(-0.1*(180-e_slip))
        else:
            r_slip = -np.exp(-0.1*(e_slip+180))
        
        r_std_steer = np.exp(-2*std_steer)
        r_std_throttle = np.exp(-2*std_throttle)
        
        vx = self.velocity_local[0]
        vy = self.velocity_local[1]
        v = math.sqrt(vx*vx + vy*vy)
        
        reward = v*(40*r_dis + 40*r_heading + 20*r_slip)
        
        if v < 6:
            reward  = reward / 2
        
        return reward

    def getNearby(self):

        self.waypoints_ahead = [] 
        self.waypoints_neighbor = []
        egoLocation = self.world.player.get_location()
        dx_array = self.route_x - egoLocation.x
        dy_array = self.route_y - egoLocation.y
        dis_array = np.sqrt(dx_array * dx_array + dy_array * dy_array)
        self.minDis = np.amin(dis_array)
        _ = np.where(dis_array == self.minDis)
        index = _[0][0]  # index for the min distance to all waypoints.
        
        self.drived_distance = self.route_length[index]
        self.waypoints_ahead = self.route[index:,:]
        
        if index >= 20:
            index_st = index - 20
        else:
            index_st = 0
        self.waypoints_neighbor = self.route[index_st:,:]
        self.traj_index = index


    # def render(self):
    #     # show ROS client window by pygame
    #     self.world.tick(self.clock, self.e_dis, self.e_heading, self.velocity_local[2] )
    #     self.world.render(self.display)
    #     pygame.display.flip()


    def velocity_world2local(self,yaw):
        # TODO need publishers and subscribers or services and clients
        # velocity_world = self.world.player.get_velocity()
        vx = velocity_world.x
        vy = velocity_world.y
        yaw = -yaw
        
        local_x = float(vx * math.cos(yaw) - vy * math.sin(yaw))
        local_y = float(vy * math.cos(yaw) + vx * math.sin(yaw))
        if local_x != 0:
            slip_angle = math.atan(local_y/local_x)/3.1415926*180
        else:
            slip_angle = 0
        
        self.velocity_local = [local_x,local_y,slip_angle]

    def velocity_local2world(self, velocity_local, yaw):
        vx = velocity_local[0]
        vy = velocity_local[1]
        
        world_x = vx * math.cos(yaw) - vy * math.sin(yaw)
        world_y = vy * math.cos(yaw) + vx * math.sin(yaw)
        
        return Vector3(world_x,world_y,0)

    # def collisionDetect(self):
    #     # TODO need publishers and subscribers or services and clients
    #     if self.world.collision_sensor.history:
    #         return True
    #     else:
    #         return False

    def getAction(self,actionID=4,steer=0, throttle=0):
        if self.model == 'dqn':
            throttleID = int(actionID / self.sStateNum)
            steerID = int(actionID % self.sStateNum)
        
            # self.control = carla.VehicleControl(
            #                     throttle = self.step_T_pool[throttleID],
            #                     steer = self.step_S_pool[steerID],
            #                     brake = 0.0,
            #                     hand_brake = False,
            #                     reverse = False,
            #                     manual_gear_shift = False,
            #                     gear = 0)
            self.control = vehicle_control(throttle = self.step_T_pool[throttleID],
                                steer = self.step_S_pool[steerID],
                                brake = 0.0,
                                hand_brake = False,
                                reverse = False,
                                manual_gear_shift = False,
                                gear = 0)
        else:
            self.control = vehicle_control(
                                throttle = throttle,
                                steer = steer,
                                brake = 0.0,
                                hand_brake = False,
                                reverse = False,
                                manual_gear_shift = False,
                                gear = 0)
        return self.control

    def coordinateTransform(self,egoLocation,yaw):
        # transfer the nearest waypoint to the local coordinate.
        way_x = self.waypoints_ahead[0,0]
        way_y = self.waypoints_ahead[0,1]
        yaw = -yaw
        
        dx = way_x - egoLocation.x
        dy = way_y - egoLocation.y
        
        nx = dx * math.cos(yaw) - dy * math.sin(yaw)
        ny = dy * math.cos(yaw) + dx * math.sin(yaw)
        
        if nx > 0 and ny > 0:
            return 1
        elif nx> 0 and ny < 0:
            return 2
        elif nx<0 and ny < 0:
            return 3
        elif nx<0 and ny>0:
            return 4

    def getLocalFutureWay(self,egoLocation,yaw):
        # transfer the future waypoints (#10) to the local coordinate.
        # x, y, slip (degree)
        
        ways = self.waypoints_ahead[0:-1:5,:]  # filter to 1m between way pts 
        if ways.shape[0] < 11:
            self.destinationFlag = True
        self.waypoints_ahead_local = []
        yaw = -yaw

        for w in ways[0:10]: 

            wx = w[0]
            wy = w[1]
            w_slip = w[5]
            dx = wx - egoLocation.x
            dy = wy - egoLocation.y

            nx = dx * math.cos(yaw) - dy * math.sin(yaw)
            ny = dy * math.cos(yaw) + dx * math.sin(yaw)
            self.waypoints_ahead_local.append(np.array([nx,ny,w_slip]))

    def getLocalHistoryWay(self,egoLocation,yaw):
        # x, y, steer, slip (degree)
        ways = self.waypoints_history
        yaw = -yaw
        self.waypoints_history_local = []
        if len(ways) < 5:
            for i in range(5 - len(ways)):
                self.waypoints_history_local.append(np.array([0,0,0,0]))
        
        for w in ways:
            wx = w[0]
            wy = w[1]
            w_steer = w[2]
            w_slip = w[3]
            dx = wx - egoLocation.x
            dy = wy - egoLocation.y

            nx = dx * math.cos(yaw) - dy * math.sin(yaw)
            ny = dy * math.cos(yaw) + dx * math.sin(yaw)
            self.waypoints_history_local.append(np.array([nx,ny,w_steer,w_slip]))
        
    def vgf_direction(self,egoLocation):
        way_x = self.waypoints_ahead[0,0]
        way_y = self.waypoints_ahead[0,1]
        yaw = -self.waypoints_ahead[0,2]/180.0 * 3.141592653
        
        dx = egoLocation.x - way_x
        dy = egoLocation.y - way_y
        
        nx = dx * math.cos(yaw) - dy * math.sin(yaw)
        ny = dy * math.cos(yaw) + dx * math.sin(yaw)
        
        if ny < 0:
            return True
        else:
            return False
        

#****************************************
# Train.py
#****************************************
if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument('--tau',  default=0.005, type=float) # target smoothing coefficient
    parser.add_argument('--target_update_interval', default=1, type=int)
    parser.add_argument('--gradient_steps', default=1, type=int)

    parser.add_argument('--learning_rate', default=3e-4, type=int)
    parser.add_argument('--gamma', default=0.99, type=int) # discount gamma
    
    parser.add_argument('--capacity', default=50000, type=int) # replay buffer size
    parser.add_argument('--iteration', default=100000, type=int) #  num of  games
    parser.add_argument('--batch_size', default=512, type=int) # mini batch size

    parser.add_argument('--seed', default=1, type=int)

    # optional parameters
    parser.add_argument('--num_hidden_layers', default=2, type=int)
    parser.add_argument('--num_hidden_units_per_layer', default=256, type=int)
    parser.add_argument('--sample_frequency', default=256, type=int)
    parser.add_argument('--activation', default='Relu', type=str)
    parser.add_argument('--render', default=False, type=bool) # show UI or not
    parser.add_argument('--log_interval', default=50, type=int) #
    parser.add_argument('--load', default=False, type=bool) # load model

    args = parser.parse_args()

    # self.declare_parameter('robot_length', 0.01)
    # self.robot_length = self.get_parameter('robot_length').get_parameter_value().double_value
    robot_length = 

    # subscriber to control
    # self.control_sub = self.create_subscription(Control, 'control', self.control_callback, 10)

    # publisher to control
    # self.control_pub = self.create_publisher(Control, 'control', 10)

    # publisher to cmd_vel
    # self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    # publisher to config
    # self.config_pub = self.create_publisher(Config, 'config', 10)

    # create timer that runs at 100 Hz
    # self.freq = 100
    # self.timer = self.create_timer(1 / (self.freq), self.timer)

    control = Control()
    control.throtte = 0.0
    control.steer = 0
    control.brake = 0.0
    control.hand_brake = False
    control.reverse = False
    control.manual_gear_shift = False
    control.gear = 0


    x_dot = 0.0
    y_dot = 0.0
    theta_dot = 0.0

    config = Config(0.0, 0.0, 0.0)

    # from xacro
    wheelbase = (2*(robot_length/2)/3) - (-2*(robot_length/2)/3)


    # print(1)
    # pygame.init()
    # print(2)
    # pygame.font.init()
    # print(3)
    env = Environment(traj_num=1)

    action_dim = 2
    state = env.getState()
    state_dim = len(state)
    print('action_dimension:', action_dim, ' & state_dimension:', state_dim)

    destinationFlag = False
    collisionFlag = False
    awayFlag = False
    carla_startFlag = False

    agent = SACAgent(state_dim=state_dim, action_dim=action_dim)

    if args.load: agent.load(epoch= 60, capacity= 1)

    print("====================================")
    print("Collection Experience...")
    print("====================================")

    ep_r = 0###expectation of reward R
    for i in range(args.iteration):
        state = env.reset(traj_num=1, randomPosition=False)
        t0 = time.time()
        first_step_pass = False

        count = 0
        speed = 0
        cte = 0
        hae = 0
        time_cost = 0

        while(True):
            count += 1
            # env.render()
            # plt.clf()

            # start training when the carla env is ready, before that we loop:
            # TODO make subscriber or service client
            # tmp_control = env.world.player.get_control()
            if control.throttle == 0 and carla_startFlag==False:
                tmp_control = vehicle_control(
                                throttle = 0.5,
                                steer = 0,
                                brake = 0.0,
                                hand_brake = False,
                                reverse = False,
                                manual_gear_shift = False,
                                gear = 0)

            # TODO make publisher or server
                control.throttle = tmp_control.throttle
                control.steer = tmp_control.steer
                control.brake = tmp_control.brake
                control.hand_brake = tmp_control.hand_brake
                control.reverse = tmp_control.reverse
                control.manual_gear_shift = tmp_control.manual_gear_shift
                control.gear = tmp_control.gear

                # self.control_pub.publish(self.control)

                move = Twist()
                move = Twist(linear=Vector3(x=control.throttle,
                                    y=0.0, z=0.0),
                                    angular=Vector3(x=0.0, y=0.0, z=control.steer))
                # self.cmd_vel_pub.publish(move)
                # env.world.player.apply_control(tmp_control)
                continue
            carla_startFlag = True

            # if time.time()-t0 < 0.5:
                # TODO make publisher or server
                # env.world.collision_sensor.history = []
            if i % 10 != 0 or agent.replay_buffer.num_transition <= 3000:
                if time.time()-t0 > 0.5:

                    if not first_step_pass:
                        steer = 0.0
                        throttle = 0.0
                        hand_brake = False
                    else:
                        action = agent.select_action(tState)
                        # print(action.shape)
                        action = np.reshape(action, [1,2])
                        # print(action.shape)

                        steer = action[0,0]
                        throttle = action[0,1]
                        print("mapped steer: ", steer, ", throttle: ",throttle)
                        if i%5==0:
                            agent.writer.add_scalar('Control/iteration_'+str(i)+'/steer', steer, global_step = count)
                            agent.writer.add_scalar('Control/iteration_'+str(i)+'/throttle', throttle, global_step = count)	

                    next_state, reward, collisionFlag, destinationFlag, awayFlag, control = env.step(steer, throttle)
                    next_state = np.reshape(next_state, [1, state_dim])

                    ep_r += reward
                    endFlag = collisionFlag or destinationFlag or awayFlag

                    if first_step_pass:

                        action[0,0] = (action[0,0] - agent.steer_range[0]) / (agent.steer_range[1] - agent.steer_range[0]) * 2 - 1
                        action[0,1] = (action[0,1] - agent.throttle_range[0]) / (agent.throttle_range[1] - agent.throttle_range[0]) * 2 - 1

                        agent.replay_buffer.push(tState, action, reward, next_state, endFlag)

                    tState = next_state

                    vx = env.velocity_local[0]
                    vy = env.velocity_local[1]
                    speed += np.sqrt(vx*vx + vy*vy)
                    cte += tState[0,2]
                    hae += abs(tState[0,4])

                    if endFlag:
                        break

                    print('buffer_size: %d'%agent.replay_buffer.num_transition)

                    first_step_pass = True 
            else:
                if time.time()-t0 > 0.5:

                    if not first_step_pass:
                        steer = 0.0
                        throttle = 0.0
                        hand_brake = False
                    else:
                        action = agent.test(tState)
                        action = np.reshape(action, [1,2])

                        steer = action[0,0]
                        throttle = action[0,1]
                        print('############### TESTING ##############')
                        print("mapped steer: ", steer, ", throttle: ",throttle)
                        if i%5==0:
                            agent.writer.add_scalar('TEST/Control/iteration_'+str(i)+'/steer', steer, global_step = count)
                            agent.writer.add_scalar('TEST/Control/iteration_'+str(i)+'/throttle', throttle, global_step = count)	

                    next_state, reward, collisionFlag, destinationFlag, awayFlag, control = env.step(steer, throttle)
                    next_state = np.reshape(next_state, [1, state_dim])
                    ep_r += reward
                    endFlag = collisionFlag or destinationFlag or awayFlag
                    
                    tState = next_state
                    
                    endFlag = collisionFlag or destinationFlag or awayFlag
                    
                    vx = env.velocity_local[0]
                    vy = env.velocity_local[1]
                    speed += np.sqrt(vx*vx + vy*vy)
                    cte += tState[0,2]
                    hae += abs(tState[0,4])

                    if endFlag:
                        break

                    first_step_pass = True 
            
            x_dot = throttle * cos(self.config.theta)
            y_dot = throttle * sin(self.config.theta)
            theta_dot = throttle / self.wheelbase * tan(steer)

            # update state (estimate dt)
            dt = 0.01
            self.config.x += x_dot*dt
            self.config.y += y_dot*dt
            self.config.theta += theta_dot*dt

            self.config_pub.publish(self.config)

        time_cost = time.time() - t0


        if i % 10 != 0 or agent.replay_buffer.num_transition <= 3000:
            print("*************TRAIN**************")
            if agent.replay_buffer.num_transition >= 1000 and agent.replay_buffer.num_transition<10000:
                for u in range(5):
                    agent.update()
            if agent.replay_buffer.num_transition >= 10000 and agent.replay_buffer.num_transition<40000:
                for u in range(100):
                    agent.update()
            if agent.replay_buffer.num_transition>=40000 and agent.replay_buffer.num_transition<80000:
                for u in range(300):
                    agent.update()
            if agent.replay_buffer.num_transition>=80000 and agent.replay_buffer.num_transition<150000:
                for u in range(400):
                    agent.update()
            if agent.replay_buffer.num_transition>=150000 and agent.replay_buffer.num_transition<300000:
                for u in range(600):
                    agent.update()
            if agent.replay_buffer.num_transition>=300000:
                for u in range(800):
                    agent.update()
            print("***********TRAIN OVER***********")


        speed = speed / count
        cte = cte/count
        hae = hae/count

        if i % 10 == 0 and agent.replay_buffer.num_transition > 3000:
            agent.save(i, args.capacity)

        print("Ep_i: %d, the ep_r is: %.2f" % (i, ep_r))

        agent.writer.add_scalar('Metrics/ep_r', ep_r, global_step=i)
        agent.writer.add_scalar('Metrics/time_cost', time_cost, global_step=i)
        agent.writer.add_scalar('Metrics/avg_speed', speed, global_step=i)
        agent.writer.add_scalar('Metrics/avg_cross_track_error', cte, global_step=i)
        agent.writer.add_scalar('Metrics/avg_heading_error', hae, global_step=i)
        agent.writer.add_scalar('Metrics/reward_every_second', ep_r/time_cost, global_step=i)
    
        agent.writer.add_scalar('Physics/Tire_friction', env.tire_friction, global_step = i)
        agent.writer.add_scalar('Physics/Mass', env.mass, global_step=i)


        if i % 10 ==0 and agent.replay_buffer.num_transition > 3000:
            agent.writer.add_scalar('Metrics_test/ep_r', ep_r, global_step=i)
            agent.writer.add_scalar('Metrics_test/time_cost', time_cost, global_step=i)
            agent.writer.add_scalar('Metrics_test/avg_speed', speed, global_step=i)
            agent.writer.add_scalar('Metrics_test/avg_cross_track_error', cte, global_step=i)
            agent.writer.add_scalar('Metrics_test/avg_heading_error', hae, global_step=i)
            agent.writer.add_scalar('Metrics_test/reward_every_second', ep_r/time_cost, global_step=i)

            agent.writer.add_scalar('Physics_test/Tire_friction', env.tire_friction, global_step = i)
            agent.writer.add_scalar('Physics_test/Mass', env.mass, global_step=i)
        ep_r = 0

    def control_callback(self, msg):
        self.control.throttle = msg.throttle
        self.control.steer = msg.steer
        self.control.brake = msg.brake
        self.control.hand_brake = msg.hand_brake
        self.control.reverse = msg.reverse
        self.control.manual_gear_shift = msg.manual_gear_shift
        self.control.gear = msg.gear
