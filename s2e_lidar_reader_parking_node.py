import os
import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np
from Adafruit_PCA9685 import PCA9685
import RPi.GPIO as GPIO

class s2eLidarReaderParkingNode(Node):
    HPIX = 320
    VPIX = 200
    HFOV = 70.8
    num_scan = 1620 # consider only front 180 degrees
    num_scan2 = 810
    scan_max_dist = 2.8
    reverse_pulse = 204
    neutral_pulse = 310
    forward_pulse = 409
    servo_min = 260  # Min pulse length out of 4096
    servo_max = 380  # Max pulse length out of 4096
    servo_neutral = int((servo_max+servo_min)/2)
    servo_ctl = int(-(servo_max-servo_min)/2 *1.0)
    motor_ctl_fwd = 16
    motor_ctl_rev = 10
    relay_pin = 17
    WEIGHT = 1

    filepath = '/home/rrrschuetz/test/file_p.txt'

    scan_labels = [f'SCAN.{i}' for i in range(1, num_scan+1)]
    col1_g_labels = [f'COL1_G.{i}' for i in range(1, HPIX+1)]
    col2_g_labels = [f'COL2_G.{i}' for i in range(1, HPIX+1)]
    col1_r_labels = [f'COL1_R.{i}' for i in range(1, HPIX+1)]
    col2_r_labels = [f'COL2_R.{i}' for i in range(1, HPIX+1)]
    col1_m_labels = [f'COL1_M.{i}' for i in range(1, HPIX+1)]
    col2_m_labels = [f'COL2_M.{i}' for i in range(1, HPIX+1)]

    labels = ['X', 'Y'] + scan_labels + col1_g_labels + col2_g_labels + col1_r_labels + col2_r_labels + col1_m_labels + col2_m_labels
    line = ','.join(labels) + '\n'

    filepath = '~/test/file_p.txt'
    labels = os.path.exists(filepath)
    with open(filepath, 'a') as f:
        if not labels: f.write(line)

    def __init__(self):
        super().__init__('s2e_lidar_reader_node')
        self.publisher_ = self.create_publisher(String, 'main_logger', 10)

        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self._capture = False
        self._sequence_count = 0

        self._scan_interpolated = np.zeros(self.num_scan)
        self._color1_g = np.zeros(self.HPIX, dtype=int)
        self._color2_g = np.zeros(self.HPIX, dtype=int)
        self._color1_r = np.zeros(self.HPIX, dtype=int)
        self._color2_r = np.zeros(self.HPIX, dtype=int)
        self._color1_m = np.zeros(self.HPIX, dtype=int)
        self._color2_m = np.zeros(self.HPIX, dtype=int)
        self._clockwise = False
        self._clockwise_def = False
        self._X = 0.0
        self._Y = 0.0

        # Initialize PCA9685
        self._pwm = PCA9685()
        self._pwm.set_pwm_freq(50)  # Set frequency to 50Hz

        self.get_logger().info('calibrating ESC')
        self._pwm.set_pwm(1, 0, self.neutral_pulse)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.HIGH)

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

    def __del__(self):
        GPIO.output(self.relay_pin, GPIO.LOW)
        GPIO.cleanup()

    def lidar_callback(self, msg):
        if not self._capture: return

        # Convert the laser scan data to a string
        scan = np.array(msg.ranges[self.num_scan+self.num_scan2:]+msg.ranges[:self.num_scan2])

        scan[scan == np.inf] = np.nan
        scan[scan > self.scan_max_dist] = np.nan
        x = np.arange(len(scan))

        if not self._clockwise_def:
            self._clockwise_def = True
            sum_first_half = np.nansum(scan[:self.num_scan2])
            sum_second_half = np.nansum(scan[self.num_scan2+1:self.num_scan])
            self._clockwise = (sum_first_half <= sum_second_half)
            self.get_logger().info('lidar_callback: clockwise "%s" ' % self._clockwise)

        finite_vals = np.isfinite(scan)
        self._scan_interpolated = np.interp(x,x[finite_vals],scan[finite_vals])

        # Convert the laser scan data to a string
        scan_data = str(self._X)+','+str(self._Y)+','
        scan_data += ','.join(str(e) for e in self._scan_interpolated)+','
        #scan_data += ','.join(str(e) for e in scan)

        # add color data
        scan_data += ','.join(str(e) for e in self._color1_g)+','
        scan_data += ','.join(str(e) for e in self._color2_g)+','
        scan_data += ','.join(str(e) for e in self._color1_r)+','
        scan_data += ','.join(str(e) for e in self._color2_r)+','
        scan_data += ','.join(str(e) for e in self._color1_m)+','
        scan_data += ','.join(str(e) for e in self._color2_m)

        self._sequence_count += 1
        # Write the scan data to a file
        with open(self.filepath, 'a') as f:
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

        elif hasattr(msg, 'axes') and len(msg.axes) > 5:
            self._X = msg.axes[2]
            self._Y = msg.axes[1]

        try:
            motor_ctl = -self.motor_ctl_fwd if self._Y < 0 else -self.motor_ctl_rev
            #self.get_logger().info('Steering: "%s" ' % str(self.servo_neutral+self._X*self.servo_ctl))
            #self.get_logger().info('Power: "%s" ' % str(self.neutral_pulse-self._Y*self.motor_ctl))
            self._pwm.set_pwm(0, 0, int(self.servo_neutral+self._X*self.servo_ctl))
            self._pwm.set_pwm(1, 0, int(self.neutral_pulse+self._Y*motor_ctl))

        except IOError as e:
            self.get_logger().error('IOError I2C occurred: %s' % str(e))

    def openmv_h7_callback(self, msg):
        try:
            data = msg.data.split(',')

            if data[0] == '240024001951333039373338': cam = 1     # 33001c000851303436373730
            elif data[0] == '2d0024001951333039373338': cam = 2   # 340046000e51303434373339
            else: return

            if cam == 1:
                self._color1_g = np.zeros(self.HPIX, dtype=int)
                self._color1_r = np.zeros(self.HPIX, dtype=int)
                self._color1_m = np.zeros(self.HPIX, dtype=int)
            elif cam == 2:
                self._color2_g = np.zeros(self.HPIX, dtype=int)
                self._color2_r = np.zeros(self.HPIX, dtype=int)
                self._color2_m = np.zeros(self.HPIX, dtype=int)

            self._parking_lot_detect = False
            blobs = ((data[i],data[i+1],data[i+2]) for i in range (1,len(data),3))
            for blob in blobs:
                color, x1, x2 = blob
                color = int(color)
                x1 = int(x1)
                x2 = int(x2)
                if color == 1:
                    if cam == 1 and not self._clockwise: self._color1_g[x1:x2] = self.WEIGHT
                    if cam == 2 and self._clockwise: self._color2_g[x1:x2] = self.WEIGHT
                if color == 2:
                    if cam == 1 and not self._clockwise: self._color1_r[x1:x2] = self.WEIGHT
                    if cam == 2 and self._clockwise: self._color2_r[x1:x2] = self.WEIGHT
                if color == 4:
                    if cam == 1: self._color1_m[x1:x2] = self.WEIGHT
                    if cam == 2: self._color2_m[x1:x2] = self.WEIGHT

                #self.get_logger().info('CAM: blob inserted: %s,%s,%s,%s' % (cam,color,x1,x2))

        except:
            self.get_logger().error('Faulty cam msg received: "%s"' % msg)

    def openmv_h7_callback1(self, msg):
        self.openmv_h7_callback(msg)
    def openmv_h7_callback2(self, msg):
        self.openmv_h7_callback(msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_reader_parking_node = s2eLidarReaderParkingNode()
    rclpy.spin(lidar_reader_parking_node)
    lidar_reader_parking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
