from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import SetBool
from enum import Enum, auto


class States(Enum):
    """
    Different possible states of the program.

    Purpose: determines what functions get called in the main timer
    """

    FORWARD = auto()
    REVERSE = auto()
    STOP = auto()


class Drift(Node):

    def __init__(self):
        super().__init__('drift')

        self.state = States.STOP

        # publisher to cmd_vel from ignition msgs
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # create a service to run Jerry
        self.flip_Jerry = self.create_service(
            SetBool, 'flip', self.flip_Jerry_callback)

        # create timer that runs at 100 Hz
        self.freq = 100
        self.timer = self.create_timer(1 / (self.freq), self.timer)

        self.vel = Twist()


    def timer(self):
        """
        Timer callback at 100 Hz.

        Publishes:
            cmd_vel (geometry_msgs/msg/Twist): velocity command for mobile robot

        Args:
            no arguments

        Returns
        -------
            no returns

        """
        if self.state == States.FORWARD:
            self.flip(-1.8)
            sleep(3)
            self.state = States.REVERSE
            self.vel_pub.publish(self.vel)

        elif self.state == States.REVERSE:
            self.flip(1.8)
            sleep(3)
            self.state = States.FORWARD
            self.vel_pub.publish(self.vel)

        elif self.state == States.STOP:
            self.flip(0.0)
            self.vel_pub.publish(self.vel)


def drift_entry(args=None):
    rclpy.init(args=args)
    drift = Drift()
    rclpy.spin(drift)
    rclpy.shutdown()
