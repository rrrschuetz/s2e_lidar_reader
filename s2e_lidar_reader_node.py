import os, time, configparser
import rclpy, math
from rclpy.time import Time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
import tensorflow as tf
import pickle
from Adafruit_PCA9685 import PCA9685
from sense_hat import SenseHat
import RPi.GPIO as GPIO
import usb.core
import usb.util

G_LEFT_CAM_ID = ""
G_RIGHT_CAM_ID = ""

HPIX = 320
G_color1_g = np.zeros(HPIX, dtype=int)
G_color1_r = np.zeros(HPIX, dtype=int)
G_color2_g = np.zeros(HPIX, dtype=int)
G_color2_r = np.zeros(HPIX, dtype=int)

G_clockwise = False
G_cam_updates = 0

class s2eLidarReader(Node):
    VPIX = 200
    HFOV = 70.8
    num_scan = 1620
    num_scan2 = 810
    scan_max_dist = 2.8
    reverse_pulse = 204
    neutral_pulse = 307
    forward_pulse = 409
    servo_min = 260  # Min pulse length out of 4096
    servo_max = 380  # Max pulse length out of 4096
    servo_neutral = int((servo_max+servo_min)/2)
    servo_ctl = int(-(servo_max-servo_min)/2 *1.0)
    motor_ctl = 16
    relay_pin = 17

    def __init__(self):
        global G_color1_r,G_color1_g,G_color2_r,G_color2_g
        global G_clockwise,G_cam_updates
        global G_LEFT_CAM_ID, G_RIGHT_CAM_ID

        super().__init__('s2e_lidar_reader_node')
        self.publisher_ = self.create_publisher(String, 'main_logger', 10)

        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        config = configparser.ConfigParser()
        config.read('/home/rrrschuetz/ros2_ws4/config.ini')

        FWD_SPEED_initial = str(config['Speed']['forward_initial_counterclockwise'])
        FWD_SPEEDU_initial = str(config['Speed']['forward_initial_clockwise'])
        FWD_SPEED_obstacle = str(config['Speed']['forward_obstacle_counterclockwise'])
        FWD_SPEEDU_obstacle = str(config['Speed']['forward_obstacle_clockwise'])
        self.get_logger().info(f"Speed settings initial race: {FWD_SPEEDU_initial}/{FWD_SPEEDU_initial}")
        self.get_logger().info(f"Speed settings obstacle race: {FWD_SPEED_obstacle}/{FWD_SPEEDU_obstacle}")

        G_LEFT_CAM_ID = str(config['Hardware']['left_cam_id'])
        G_RIGHT_CAM_ID = str(config['Hardware']['right_cam_id'])
        self.get_logger().info(f"Left / right camera IDs: {G_LEFT_CAM_ID} / {G_RIGHT_CAM_ID}")

        scan_labels = [f'SCAN.{i}' for i in range(1, self.num_scan+1)]
        HPIX = 320
        col1_g_labels = [f'COL1_G.{i}' for i in range(1, HPIX+1)]
        col2_g_labels = [f'COL2_G.{i}' for i in range(1, HPIX+1)]
        col1_r_labels = [f'COL1_R.{i}' for i in range(1, HPIX+1)]
        col2_r_labels = [f'COL2_R.{i}' for i in range(1, HPIX+1)]

        labels = ['X', 'Y'] + scan_labels + col1_g_labels + col2_g_labels + col1_r_labels + col2_r_labels
        line = ','.join(labels) + '\n'

        filepath = '/home/rrrschuetz/test/file.txt'
        labels = os.path.exists(filepath)
        with open(filepath, 'a') as f:
            if not labels: f.write(line)

        G_color1_g = np.zeros(HPIX, dtype=int)
        G_color1_r = np.zeros(HPIX, dtype=int)
        G_color2_g = np.zeros(HPIX, dtype=int)
        G_color2_r = np.zeros(HPIX, dtype=int)
        self._scan_interpolated = np.zeros(self.num_scan)

        G_cam_updates = 0
        G_clockwise = False
        self._clockwise_def = False
        self._capture = False
        self._sequence_count = 0
        self._X = 0.0
        self._Y = 0.0

        # Initialize PCA9685
        self._pwm = PCA9685()
        self._pwm.set_pwm_freq(50)  # Set frequency to 50Hz
        self._pwm.set_pwm(0, 0, int(self.servo_neutral))
        self.get_logger().info('Steering unit initialized ...')

        self._pwm.set_pwm(1, 0, self.neutral_pulse)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.HIGH)
        self.get_logger().info('calibrating ESC')
        self.prompt("Switch on ESC")

        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )

        self.subscription_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.get_logger().info('calibrating ESC')
        self.prompt("Ready!")
        self.get_logger().info('Ready.')

    def __del__(self):
        self.motor_off()

    def prompt(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)

    def motor_off(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.LOW)
        GPIO.cleanup()

    def lidar_callback(self, msg):
        global G_color1_r,G_color1_g,G_color2_r,G_color2_g,G_color1_m,G_color2_m
        global G_clockwise,G_cam_updates

        if not self._capture: return
        #self.get_logger().info(f"Cam updates per lidar callback {G_cam_updates}")
        G_cam_updates = 0

        # Convert the laser scan data to a string
        scan = np.array(msg.ranges[self.num_scan+self.num_scan2:]+msg.ranges[:self.num_scan2])

        #scan[scan == np.inf] = 0.0
        #scan[scan > self.scan_max_dist] = 0.0
        #self._scan_interpolated = scan

        scan[scan == np.inf] = np.nan
        scan[scan > self.scan_max_dist] = np.nan
        x = np.arange(len(scan))

        if not self._clockwise_def:
            self._clockwise_def = True
            sum_first_half = np.nansum(scan[200:self.num_scan2])
            sum_second_half = np.nansum(scan[self.num_scan2+1:self.num_scan-200])
            self._clockwise = (sum_first_half <= sum_second_half)
            self.get_logger().info('lidar_callback: clockwise "%s" ' % self._clockwise)

        finite_vals = np.isfinite(scan)
        self._scan_interpolated = np.interp(x,x[finite_vals],scan[finite_vals])

        # Convert the laser scan data to a string
        scan_data = str(self._X)+','+str(self._Y)+','
        scan_data += ','.join(str(e) for e in self._scan_interpolated)+','
        #scan_data += ','.join(str(e) for e in scan)

        # add color data
        scan_data += ','.join(str(e) for e in G_color1_g)+','
        scan_data += ','.join(str(e) for e in G_color2_g)+','
        scan_data += ','.join(str(e) for e in G_color1_r)+','
        scan_data += ','.join(str(e) for e in G_color2_r)

        self._sequence_count += 1
        # Write the scan data to a file
        with open('/home/rrrschuetz/test/file.txt', 'a') as f:
            f.write(scan_data + '\n')

    def joy_callback(self, msg):
        #self.get_logger().info('Buttons: "%s"' % msg.buttons)
        #self.get_logger().info('Axes: "%s"' % msg.axes)

        if hasattr(msg, 'buttons') and len(msg.buttons) > 0:

            # Check if 'A' button is pressed - switch on AI steering
            if msg.buttons[0] == 1:
                self.get_logger().info('data capture started')
                self._capture = True
                self._sequence_count = 0

            # Check if 'B' button is pressed - switch off AI steering
            elif msg.buttons[1] == 1:
                self.get_logger().info('data capture stopped, %s lines' % self._sequence_count)
                self._capture = False

        elif hasattr(msg, 'axes') and len(msg.axes) > 2:
            self._X = msg.axes[2]
            self._Y = msg.axes[1]

        try:
            #self.get_logger().info('Steering: "%s"' % str(self.servo_neutral+self._X*self.servo_ctl))
            #self.get_logger().info('Power: "%s"' % str(self.neutral_pulse+self._Y*40))
            self._pwm.set_pwm(0, 0, int(self.servo_neutral+self._X*self.servo_ctl))
            self._pwm.set_pwm(1, 0, int(self.neutral_pulse-self._Y*self.motor_ctl))

        except IOError as e:
            self.get_logger().error('IOError I2C occurred: %s' % str(e))

class cameraNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(String, 'main_logger', 10)
        self._busy = False

        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.subscription = self.create_subscription(
            String,
            name,
            self.openmv_h7_callback,
            qos_profile
        )

    def prompt(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)

    def openmv_h7_callback(self, msg):
        global G_color1_r,G_color1_g,G_color2_r,G_color2_g,G_color1_m,G_color2_m
        global G_tf_control,G_parking_lot,G_clockwise, G_cam_updates
        global G_LEFT_CAM_ID, G_RIGHT_CAM_ID

        if self._busy: return
        self._busy = True

        HPIX = 320
        WEIGHT = 1

        try:
            data = msg.data.split(',')

            if data[0] == G_LEFT_CAM_ID: cam = 1      # 33001c000851303436373730 / 240024001951333039373338
            elif data[0] == G_RIGHT_CAM_ID: cam = 2   # 2d0024001951333039373338 / 340046000e51303434373339
            else:
                self.get_logger().error(f'Cam not found: {data[0]}')
                self._busy = False
                return

            if cam == 1:
                G_color1_g = np.zeros(HPIX, dtype=int)
                G_color1_r = np.zeros(HPIX, dtype=int)
            elif cam == 2:
                G_color2_g = np.zeros(HPIX, dtype=int)
                G_color2_r = np.zeros(HPIX, dtype=int)

            blobs = ((data[i],data[i+1],data[i+2]) for i in range (1,len(data),3))
            for blob in blobs:
                color, x1, x2 = blob
                color = int(color)
                ix1 = int(x1)
                ix2 = int(x2)

                if color == 1:
                    if cam == 1 and not G_clockwise:
                        G_cam_updates += 1
                        G_color1_g[ix1:ix2] = WEIGHT
                        self.prompt('*,'+x1+','+x2+',G')
                    if cam == 2 and G_clockwise:
                        G_cam_updates += 1
                        G_color2_g[ix1:ix2] = WEIGHT
                        self.prompt('*,'+x1+','+x2+',G')
                if color == 2:
                    if cam == 1 and not G_clockwise:
                        G_cam_updates += 1
                        G_color1_r[ix1:ix2] = WEIGHT
                        self.prompt('*,'+x1+','+x2+',R')
                    if cam == 2 and G_clockwise:
                        G_cam_updates += 1
                        G_color2_r[ix1:ix2] = WEIGHT
                        self.prompt('*,'+x1+','+x2+',R')

        except Exception as e:
            self.get_logger().error(f"Faulty cam msg received: {msg.data} {e}")
        finally:
            self._busy = False

def main(args=None):

    rclpy.init(args=args)
    lidar_reader_node = s2eLidarReader()
    cam1_node = cameraNode('openmv_topic1')
    cam2_node = cameraNode('openmv_topic2')

    # Use MultiThreadedExecutor to allow parallel callback execution
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(lidar_reader_node)
    executor.add_node(cam1_node)
    executor.add_node(cam2_node)

    try:
        while rclpy.ok():
            executor.spin_once()
    finally:
        executor.shutdown()
        full_drive_node.destroy_node()
        cam1_node.destroy_node()
        cam2_node.destroy_node()

if __name__ == '__main__':
    main()
