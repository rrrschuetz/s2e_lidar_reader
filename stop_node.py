import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from .Motor import PwmMotor 
import numpy as np
import tensorflow as tf

class stopNode(Node):
    def __init__(self):
        super().__init__('s2e_lidar_reader_node')
   
        self._motor = PwmMotor()
        self._motor.setMotorModel(0,0,0,0)


def main(args=None):
    rclpy.init(args=args)
    stop_node = stopNode()
    rclpy.spin(stop_node)
    stop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

