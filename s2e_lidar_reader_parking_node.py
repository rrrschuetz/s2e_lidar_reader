import os
import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
from Adafruit_PCA9685 import PCA9685

class s2eLidarReaderParkingNode(Node):
    num_scan = 1620 # consider only front 18ß degrees
    num_scan2 = 810
    reverse_pulse = 204
    neutral_pulse = 307
    forward_pulse = 409
    servo_min = 240  # Min pulse length out of 4096
    servo_max = 375  # Max pulse length out of 4096
    servo_neutral = int((servo_max+servo_min)/2)
    servo_ctl = int(-(servo_max-servo_min)/2 *1.7)
    motor_ctl = 12

    def __init__(self):
        super().__init__('s2e_lidar_reader_node')
        self.publisher_ = self.create_publisher(String, 'main_logger', 10)

        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self._scan_interpolated = np.zeros(self.num_scan)
        self._X = 0.0 
        self._Y = 0.0

        # Initialize PCA9685
        self._pwm = PCA9685()
        self._pwm.set_pwm_freq(50)  # Set frequency to 50Hz

        self.get_logger().info('calibrating ESC')
        self._pwm.set_pwm(1, 0, self.neutral_pulse)

        msg = String()
        msg.data = "Switch on ESC"
        self.publisher_.publish(msg)

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

    scan_labels = [f'SCAN.{i}' for i in range(1, num_scan+1)]
    labels = ['X', 'Y'] + scan_labels 
    line = ','.join(labels) + '\n'

    filepath = '/home/rrrschuetz/test/file_p.txt'
    labels = os.path.exists(filepath)
    with open(filepath, 'a') as f:
        if not labels: f.write(line)

    def lidar_callback(self, msg):
        # Convert the laser scan data to a string
        scan = np.array(msg.ranges[self.num_scan+self.num_scan2:]+msg.ranges[:self.num_scan2])

        #scan[scan == np.inf] = 0.0
        #scan[scan > self.scan_max_dist] = 0.0
        #self._scan_interpolated = scan

        scan[scan == np.inf] = np.nan
        scan[scan > self.scan_max_dist] = np.nan
        x = np.arange(len(scan))
        finite_vals = np.isfinite(scan)
        self._scan_interpolated = np.interp(x,x[finite_vals],scan[finite_vals])

        # Convert the laser scan data to a string
        scan_data = str(self._X)+','+str(self._Y)+','
        scan_data += ','.join(str(e) for e in self._scan_interpolated)+','
        #scan_data += ','.join(str(e) for e in scan)

        # Write the scan data to a file
        with open('/home/rrrschuetz/test/file_p.txt', 'a') as f:
            f.write(scan_data + '\n')

    def joy_callback(self, msg):
        #self.get_logger().info('Buttons: "%s"' % msg.buttons)
        #self.get_logger().info('Axes: "%s"' % msg.axes)
        if hasattr(msg, 'axes') and len(msg.axes) > 2:
            self._X = msg.axes[2]
            self._Y = msg.axes[1]
        
        #self.get_logger().info('Steering: "%s"' % str(self.servo_neutral+self._X*self.servo_ctl))
        #self.get_logger().info('Power: "%s"' % str(self.neutral_pulse+self._Y*40))
        self._pwm.set_pwm(0, 0, int(self.servo_neutral+self._X*self.servo_ctl))
        self._pwm.set_pwm(1, 0, int(self.neutral_pulse-self._Y*self.motor_ctl))

def main(args=None):
    rclpy.init(args=args)
    lidar_reader_parking_node = s2eLidarReaderParkingNode()
    rclpy.spin(lidar_reader_parking_node)
    lidar_reader_parking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
