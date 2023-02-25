import sys
# import environment
from .environment import *
from .SACAgent import *
import time
import random
# import pygame
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.patches as patches
from drift_interfaces.msg import Control, Config
from geometry_msgs.msg import Twist
from enum import Enum, auto
from math import sin, cos, tan
# from agents.navigation.basic_agent import BasicAgent


########SAC#######
# if __name__ == "__main__":
class Train(Node):
    def __init__(self):
        super().__init__('train')

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

        self.declare_parameter('robot_length', 0.01)
        self.robot_length = self.get_parameter('robot_length').get_parameter_value().double_value

        # subscriber to control
        self.control_sub = self.create_subscription(Control, 'control', self.control_callback, 10)

        # publisher to control
        self.control_pub = self.create_publisher(Control, 'control', 10)

        # publisher to cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # publisher to config
        self.config_pub = self.create_publisher(Config, 'config', 10)

        # create timer that runs at 100 Hz
        self.freq = 100
        self.timer = self.create_timer(1 / (self.freq), self.timer)

        self.control = Control()
        self.control.throtte = 0.0
        self.control.steer = 0
        self.control.brake = 0.0
        self.control.hand_brake = False
        self.control.reverse = False
        self.control.manual_gear_shift = False
        self.control.gear = 0


        x_dot = 0.0
        y_dot = 0.0
        theta_dot = 0.0

        self.config = Config()
        self.config.x = 0.0
        self.config.y = 0.0
        self.config.theta = 0.0

        # from xacro
        self.wheelbase = (2*(self.robot_length/2)/3) - (-2*(self.robot_length/2)/3)


    def timer(self):

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
                if self.control.throttle == 0 and carla_startFlag==False:
                    tmp_control = vehicle_control(
                                    throttle = 0.5,
                                    steer = 0,
                                    brake = 0.0,
                                    hand_brake = False,
                                    reverse = False,
                                    manual_gear_shift = False,
                                    gear = 0)

                # TODO make publisher or server
                    self.control.throttle = tmp_control.throttle
                    self.control.steer = tmp_control.steer
                    self.control.brake = tmp_control.brake
                    self.control.hand_brake = tmp_control.hand_brake
                    self.control.reverse = tmp_control.reverse
                    self.control.manual_gear_shift = tmp_control.manual_gear_shift
                    self.control.gear = tmp_control.gear

                    self.control_pub.publish(self.control)

                    self.move = Twist()
                    self.move = Twist(linear=Vector3(x=self.control.throttle,
                                      y=0.0, z=0.0),
                                      angular=Vector3(x=0.0, y=0.0, z=self.control.steer))
                    self.cmd_vel_pub.publish(self.move)
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


def train_entry(args=None):
    rclpy.init(args=args)
    train = Train()
    rclpy.spin(train)
    rclpy.shutdown()
