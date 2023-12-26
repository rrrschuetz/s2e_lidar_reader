import rclpy, math
from rclpy.time import Time
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from sense_hat import SenseHat
import numpy as np
import pickle
import logging
from logging.handlers import RotatingFileHandler
from Adafruit_PCA9685 import PCA9685

dt = 0.1
velocity = [0.0,0.0,0.0]
acceleration = [0.0,0.0,0.0]

# Initialize sense hat
sense = SenseHat()
sense.clear()
sense.show_message("OK", text_colour=[255, 0, 0])

While True:
    accel = sense.get_accelerometer_raw()
    acceleration = [ accel['x'],accel['y'],accel['z']]
    velocity = [v + a * dt for v, a in zip(velocity, acceleration)]
    speed = sum(v**2 for v in velocity)**0.5
    get_logger().info('current speed m/s: "%s"' % speed)
