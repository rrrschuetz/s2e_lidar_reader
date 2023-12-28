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

class s2eLidarReaderNode(Node):
    HPIX = 320
    VPIX = 200
    HFOV = 70.8
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
        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self._scan_interpolated = np.zeros(3240)
        self._color1 = np.zeros(self.HPIX)
        self._color2 = np.zeros(self.HPIX)
        self._X = 0.0 
        self._Y = 0.0
        self._speed = 0.0
        self._dt = 0.1
        self._start_time = self.get_clock().now()
        self._end_time = self.get_clock().now()

        # Initialize PCA9685
        self._pwm = PCA9685()
        self._pwm.set_pwm_freq(50)  # Set frequency to 50Hz

        self.get_logger().info('calibrating ESC')
        self._pwm.set_pwm(1, 0, self.neutral_pulse)
        #time.sleep(10)

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

        self.subscription_h71 = self.create_subscription(
            String,
            'openmv_topic1',
            self.openmv_h7_callback1,
            qos_profile
        )

        self.subscription_h72 = self.create_subscription(
            String,
            'openmv_topic2',
            self.openmv_h7_callback2,
            qos_profile
        )

        self.subscription_speed = self.create_subscription(
            String,
            'openmv_topic2',
            self.openmv_h7_callback2,
            qos_profile
        )

        self.subscription_speed = self.create_subscription(
            String,
            'speed_monitor',
            self.speed_monitor_callback,
            qos_profile
        )

        self.subscription_speed = self.create_subscription(
            Bool,
            'line_detector',
            self.line_detector_callback,
            qos_profile
        )

    num_scan = 3240
    num_colr = HPIX  # Assuming HPIX is defined elsewhere in your code
    
    scan_labels = [f'SCAN.{i}' for i in range(1, num_scan+1)]
    col1_labels = [f'COL1.{i}' for i in range(1, num_colr+1)]
    col2_labels = [f'COL2.{i}' for i in range(1, num_colr+1)]

    labels = ['X', 'Y'] + scan_labels + col1_labels + col2_labels
    line = ','.join(labels) + '\n'

    filepath = '/home/rrrschuetz/test/file.txt'
    labels = os.path.exists(filepath)
    with open(filepath, 'a') as f:
        if not labels: f.write(line)

    def lidar_callback(self, msg):
        # Convert the laser scan data to a string
        scan = np.array(msg.ranges)
        #scan[scan == np.inf] = 0.0 
        scan[scan == np.inf] = np.nan
        x = np.arange(len(scan))
        finite_vals = np.isfinite(scan)
        self._scan_interpolated = np.interp(x,x[finite_vals],scan[finite_vals])

        # Convert the laser scan data to a string
        scan_data = str(self._X)+','+str(self._Y)+','
        scan_data += ','.join(str(e) for e in self._scan_interpolated)+','
        #scan_data += ','.join(str(e) for e in scan)

        # add color data
        scan_data += ','.join(str(e) for e in self._color1)+','
        scan_data += ','.join(str(e) for e in self._color2)

        # Write the scan data to a file
        with open('/home/rrrschuetz/test/file.txt', 'a') as f:
            f.write(scan_data + '\n')

        self._start_time = self.get_clock().now()
        self._dt = (self._start_time - self._end_time).nanoseconds * 1e-9
        self._end_time = self._start_time

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

    def openmv_h7_callback1(self, msg):
        #self.get_logger().info('cam msg received: "%s"' % msg)
        if msg.data == "TARGET":
            self.get_logger().info("Target line crossing")
            return
            
        self._color1 = np.zeros(self.HPIX)
        data = msg.data.split(',')
        if not msg.data:
            self.get_logger().warning("Received empty message!")
            return
        if len(data) % 3 != 0:
            self.get_logger().error("Data length is not divisible by 3!")
            return

        blobs = ((data[i],data[i+1],data[i+2]) for i in range (0,len(data),3))
        for blob in blobs:
            color, x1, x2 = blob
            cx1 = int(x1)
            cx2 = int(x2)
            fcol = float(color)+1.0
            if fcol > 0.0:
                self._color1[cx1:cx2+1] = fcol
                self.get_logger().info('CAM1: blob inserted: %s,%s,%s' % (color,x1,x2))

    def openmv_h7_callback2(self, msg):
        #self.get_logger().info('cam msg received: "%s"' % msg)
        if msg.data == "TARGET":
            self.get_logger().info("Target line crossing")
            return

        self._color2 = np.zeros(self.HPIX)
        data = msg.data.split(',')
        if not msg.data:
            self.get_logger().warning("Received empty message!")
            return
        if len(data) % 3 != 0:
            self.get_logger().error("Data length is not divisible by 3!")
            return

        blobs = ((data[i],data[i+1],data[i+2]) for i in range (0,len(data),3))
        for blob in blobs:
            color, x1, x2 = blob
            cx1 = int(x1)
            cx2 = int(x2)
            fcol = float(color)+1.0
            if fcol > 0.0:
                self._color2[cx1:cx2+1] = fcol
                self.get_logger().info('CAM2: blob inserted: %s,%s,%s' % (color,x1,x2))

    def speed_monitor_callback(self, msg):
        self._speed = eval(msg.data)
        #self.get_logger().info('Speed monitor: %s m/s' % self._speed)
    def line_detector_callback(self, msg):
        self.get_logger().info('Line detected: %s ' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    lidar_reader_node = s2eLidarReaderNode()
    rclpy.spin(lidar_reader_node)
    lidar_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

