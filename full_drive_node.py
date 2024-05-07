import time
import rclpy, math
from rclpy.time import Time
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
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

class fullDriveNode(Node):

    OBSTACLE_RACE_PATH_CC = "/home/rrrschuetz/test/model.tflite"
    OBSTACLE_RACE_PATH_CW = "/home/rrrschuetz/test/modelu.tflite"
    INITIAL_RACE_PATH_CC = "/home/rrrschuetz/test/model_i.tflite"
    INITIAL_RACE_PATH_CW = "/home/rrrschuetz/test/model_iu.tflite"

    OBSTACLE_SCALER_PATH_CC = "/home/rrrschuetz/test/scaler.pkl"
    OBSTACLE_SCALER_PATH_CW = "/home/rrrschuetz/test/scaleru.pkl"
    INITIAL_SCALER_PATH_CC = "/home/rrrschuetz/test/scaler_i.pkl"
    INITIAL_SCALER_PATH_CW = "/home/rrrschuetz/test/scaler_iu.pkl"

    initial_race = False
    HPIX = 320
    VPIX = 200
    HFOV = 70.8
    num_scan = 1620
    num_scan2 = 810
    scan_min_dist = 0.30
    scan_max_dist = 2.8
    servo_min = 260  # Min pulse length out of 4096
    servo_max = 380  # Max pulse length out of 4096
    servo_neutral = int((servo_max+servo_min)/2)
    servo_ctl_fwd = int(-(servo_max-servo_min)/2 * 1.1)
    servo_ctl_rev = int(-(servo_max-servo_min)/2 * 1.0)
    motor_ctl = -20
    relay_pin = 17
    WEIGHT = 1
    FWD_SPEED = "5"
    REV_SPEED = "-5"

    def __init__(self):
        super().__init__('full_drive_node')
        self.publisher_ = self.create_publisher(String, 'main_logger', 10)
        self.speed_publisher_ = self.create_publisher(String, 'set_speed', 10)

        qos_profile = QoSProfile(
                depth=1, 
                history=QoSHistoryPolicy.KEEP_LAST, 
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE)
    
        self._state = 'IDLE'
        self._processing = False
        self._tf_control = False
        self._clockwise = False
        self._clockwise_def = False
        self._obstacle_chk = False
        self._backward = False
        self._parking_lot = 0
        self._collision = False
        self._gyro_cnt = 0

        self._X = 0.0 
        self._Y = 0.0
        self._Xtrim = 0.0
        self._color1_g = np.zeros(self.HPIX, dtype=int)
        self._color1_r = np.zeros(self.HPIX, dtype=int)
        self._color2_g = np.zeros(self.HPIX, dtype=int)
        self._color2_r = np.zeros(self.HPIX, dtype=int)
        self._color1_m = np.zeros(self.HPIX, dtype=int)
        self._color2_m = np.zeros(self.HPIX, dtype=int)

        self._front_dist = np.inf

        self._speed_msg = String()
        self._speed_msg.data = "0"

        # Initialize compass
        self._sense = SenseHat()
        self.get_logger().info('Sense hat initialized ...')

        # Initialize PCA9685
        self._pwm = PCA9685()
        self._pwm.set_pwm_freq(50)  # Set frequency to 50Hz
        self._pwm.set_pwm(0, 0, int(self.servo_neutral))
        self.get_logger().info('Steering unit initialized ...')

        self._round_start_time = self.get_clock().now()

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
            qos_profile
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
            Bool,
            'touch_button',
            self.touch_button_callback,
            qos_profile
        )

        self.subscription_lidar = self.create_subscription(
            String,
            'collision',
            self.collision_callback,
            qos_profile
        )

        self.get_logger().info('Messages subscribed ...')

        tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

        # Look for initial race dongle
        # ID 2357:012e TP-Link 802.11ac NIC
        self.initial_race = self.check_usb_device('2357', '012e')

        if self.initial_race:
            self.get_logger().info('Inital race mode activated ...')
            self.RACE_PATH_CC = self.INITIAL_RACE_PATH_CC
            self.RACE_PATH_CW = self.INITIAL_RACE_PATH_CW
            self.SCALER_PATH_CC = self.INITIAL_SCALER_PATH_CC
            self.SCALER_PATH_CW = self.INITIAL_SCALER_PATH_CW
            self.FWD_SPEED = "12"
        else:
            self.get_logger().info('Obstacle race mode activated ...')
            self.RACE_PATH_CC = self.OBSTACLE_RACE_PATH_CC
            self.RACE_PATH_CW = self.OBSTACLE_RACE_PATH_CW
            self.SCALER_PATH_CC = self.OBSTACLE_SCALER_PATH_CC
            self.SCALER_PATH_CW = self.OBSTACLE_SCALER_PATH_CW
            self.FWD_SPEED = "5"

        # Load the trained racing model and the scaler counterclockwise and clockwise
        with open(self.SCALER_PATH_CC, 'rb') as f:
            self._scaler = pickle.load(f)
        self._interpreter = tf.lite.Interpreter(model_path=self.RACE_PATH_CC)
        self._interpreter.allocate_tensors()
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        self.get_logger().info('counterclockwise prediction model loaded')

        with open(self.SCALER_PATH_CW, 'rb') as f:
            self._scaleru = pickle.load(f)
        self._interpreteru = tf.lite.Interpreter(self.RACE_PATH_CW)
        self._interpreteru.allocate_tensors()
        self._input_detailsu = self._interpreter.get_input_details()
        self._output_detailsu = self._interpreter.get_output_details()
        self.get_logger().info('clockwise prediction model loaded')

        msg = String()
        msg.data = "Ready!"
        self.publisher_.publish(msg)
        self.get_logger().info('Ready.')

    def __del__(self):
        self.get_logger().info('Switch off ESC')
        self.motor_off()

    def motor_off(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.LOW)
        GPIO.cleanup()

    def start_race(self):
        self._state = "RACE"
        self._tf_control = True
        self._parking_lot = 0
        self._gyro_cnt = 0
        self._corner_cnt = 0
        self._rounds = 0

        self._initial_heading = self._sense.gyro['yaw']
        self._start_heading = self._initial_heading
        self._last_heading = self._initial_heading
        self._total_heading_change = 0
        self.get_logger().info(f"Initial heading: {self._initial_heading} degrees")
        self._round_start_time = self.get_clock().now()

    def stop_race(self):
        self._state = "IDLE"
        self._tf_control = False
        self._processing = False
        self._pwm.set_pwm(0, 0, int(self.servo_neutral))
        self._speed_msg.data = "0"
        self.speed_publisher_.publish(self._speed_msg)
        self.motor_off()
        try:
            with open('/tmp/ros2_pipe', 'w') as pipe:
                pipe.write('shutdown\n')
        except:
            pass

    def calculate_heading_change(self, start_heading, current_heading):
        # Calculate the raw difference
        raw_diff = current_heading - start_heading
        # Adjust the difference to find the shortest path and preserve the turn direction
        if raw_diff > 180:
            return raw_diff - 360
        elif raw_diff < -180:
            return raw_diff + 360
        else:
            return raw_diff

    def check_usb_device(self, vendor_id, product_id):
        # Convert vendor_id and product_id from hexadecimal string to integer
        vendor_id = int(vendor_id, 16)
        product_id = int(product_id, 16)
        # Find USB device with specific Vendor ID and Product ID
        device = usb.core.find(idVendor=vendor_id, idProduct=product_id)
        # Return True if device is found, else False
        if device is not None:
            self.get_logger().info("Device Found: ID {0}:{1}".format(vendor_id, product_id))
            return True
        else:
            return False

    def lidar_callback(self, msg):
        if self._processing:
            self.get_logger().info('Scan skipped')
            return
        else:
            self._processing = True

            ########################
            # RACE
            ########################
            if self._state == 'RACE' and self._tf_control:

                scan = np.array(msg.ranges[self.num_scan+self.num_scan2:]+msg.ranges[:self.num_scan2])
                scan[scan == np.inf] = np.nan
                scan[scan > self.scan_max_dist] = np.nan

                # Round completion check
                self._gyro_cnt += 1
                if self._gyro_cnt >= 1:
                    self._gyro_cnt = 0
                    self._current_heading = self._sense.gyro['yaw']
                    heading_change = self.calculate_heading_change(self._last_heading, self._current_heading)
                    #self.get_logger().info("Heading change: %s" % heading_change)
                    self._total_heading_change += heading_change
                    self._last_heading = self._current_heading

                    num_sections = 162
                    section_data = np.array_split(scan, num_sections)
                    section_means = [np.mean(section) for section in section_data]
                    self._front_dist = max(section_means[60:101])

                    #if abs(self._total_heading_change) >= 80 and self._front_dist > 1.0 and self._front_dist < 2.0:
                    if abs(self._total_heading_change) >= 340 and self._front_dist > 1.5:
                        #self._corner_cnt +=1
                        self._rounds += 1
                        #if abs(self._total_heading_change) > 140: self._corner_cnt +=1
                        #self.get_logger().info(f"Number of corners {self._corner_cnt} heading {self._total_heading_change}")
                        self.get_logger().info(f"Number off rounds {self._rounds} heading {self._total_heading_change}")
                        self._total_heading_change = 0

                    #if self._parking_lot > 50 and self._corner_cnt >= 4:
                    if self._parking_lot > 50 and self._rounds >= 1:
                        if ((not self._clockwise and sum(self._color2_m) > 10) or (self._clockwise and sum(self._color1_m) > 10)) and self._front_dist < 1.5:

                            duration_in_seconds = (self.get_clock().now() - self._round_start_time).nanoseconds * 1e-9
                            self.get_logger().info(f"Race in {duration_in_seconds} sec completed!")
                            self.get_logger().info(f"Heading change: {self._total_heading_change} Distance: {self._front_dist}")
                            self.get_logger().info(f"Parking lot detections {self._parking_lot}")
                            self._speed_msg.data = "0"
                            self.speed_publisher_.publish(self._speed_msg)
                            msg = String()
                            msg.data = "Parking ..."
                            self.publisher_.publish(msg)

                            X = -1.0 if self._clockwise else 1.0
                            XX = int(self.servo_neutral+X*self.servo_ctl_fwd)
                            self._pwm.set_pwm(0, 0, XX)
                            time.sleep(1.0)
                            self._speed_msg.data = "F20"
                            self.speed_publisher_.publish(self._speed_msg)
                            time.sleep(2.0)

                            self._state = "IDLE"
                            self._processing = False
                            return

                    #elif self._parking_lot <= 50 and self._corner_cnt >= 12:
                    elif self._parking_lot <= 50 and self._rounds >= 3:
                        duration_in_seconds = (self.get_clock().now() - self._round_start_time).nanoseconds * 1e-9
                        self.get_logger().info(f"Race in {duration_in_seconds} sec completed!")
                        self.get_logger().info(f"Heading change: {self._total_heading_change} Distance: {self._front_dist}")
                        self.get_logger().info(f"Parking lot detections {self._parking_lot}")
                        self.stop_race()
                        self._processing = False
                        return

                try:
                    if not self._obstacle_chk:
                        self._obstacle_chk = True

                        num_sections = 162
                        section_data = np.array_split(scan, num_sections)
                        section_means = [np.mean(section) for section in section_data]
                        #self.get_logger().info('Distances "%s" ' % section_means)
                        min_far_dist = min(section_means[60:101])
                        min_near_dist = min(section_means[40:121])
                        self._front_dist = max(section_means[76:86])

                        if not self.initial_race and (min_far_dist < 0.8 or min_near_dist < 0.2):
                            self._backward = True
                            self._speed_msg.data = "R"+str(int((2.5 - self._front_dist) * 40))
                            self.get_logger().info(f"Obstacle: {min_far_dist}, {min_near_dist}, distance: {self._front_dist}, moving backward: {self._speed_msg.data}")
                            self.speed_publisher_.publish(self._speed_msg)
                            time.sleep(3)
                            self._processing = False
                            return
                        else:
                            if self._front_dist > 1.3:
                                self._speed_msg.data = "F"+str(int((self._front_dist - 1.0) * 40))
                                self.get_logger().info(f"Distance: {self._front_dist}, moving Forward: {self._speed_msg.data}")
                                self.speed_publisher_.publish(self._speed_msg)
                                time.sleep(2)
                                self._processing = False
                                return

                    elif not self._clockwise_def:
                        self._clockwise_def = True
                        sum_first_half = np.nansum(scan[:self.num_scan2])
                        sum_second_half = np.nansum(scan[self.num_scan2+1:self.num_scan])
                        self._clockwise = (sum_first_half <= sum_second_half)
                        self.get_logger().info('lidar_callback: clockwise "%s" ' % self._clockwise)

                        if self._backward:
                            X = -1.0 if self._clockwise else 1.0
                            XX = int(self.servo_neutral+X*self.servo_ctl_fwd)
                            self._pwm.set_pwm(0, 0, XX)
                            time.sleep(1.0)
                            self._speed_msg.data = "F20"
                            self.speed_publisher_.publish(self._speed_msg)
                            time.sleep(2.0)

                        self._speed_msg.data = "RESET"
                        self.speed_publisher_.publish(self._speed_msg)
                        self._speed_msg.data = self.FWD_SPEED
                        self.speed_publisher_.publish(self._speed_msg)

                    x = np.arange(len(scan))
                    finite_vals = np.isfinite(scan)
                    scan_interpolated = np.interp(x, x[finite_vals], scan[finite_vals])
                    scan_interpolated = [1/value if value != 0 else 0 for value in scan_interpolated]
                    scan_interpolated = list(scan_interpolated)
                    color_data = list(self._color1_g) + list(self._color2_g) + list(self._color1_r) + list(self._color2_r) \

                    lidar_data = np.reshape(scan_interpolated, (1, -1))               # Reshape LIDAR data
                    color_data_standardized = np.reshape(color_data, (1, -1))         # Reshape COLOR data
                    # Reshape color_data to (1, 1, 1) to match dimensions
                    color_data_standardized = np.reshape(color_data_standardized, (1, color_data_standardized.shape[1], 1)).astype(np.float32)

                    if not self._clockwise:
                        lidar_data_standardized = self._scaler.transform(lidar_data)
                        # Reshape for TFLite model input
                        lidar_data_standardized = np.reshape(lidar_data_standardized, (1, lidar_data_standardized.shape[1], 1)).astype(np.float32)
                        # Combine LIDAR and color data for the model input (concatenation, as required by your model)
                        self._interpreter.set_tensor(self._input_details[0]['index'], lidar_data_standardized)
                        self._interpreter.set_tensor(self._input_details[1]['index'], color_data_standardized)
                        # Run inference
                        self._interpreter.invoke()
                        # Retrieve the output of the model
                        predictions = self._interpreter.get_tensor(self._output_details[0]['index'])
                    else:
                        lidar_data_standardized = self._scaleru.transform(lidar_data)
                        lidar_data_standardized = np.reshape(lidar_data_standardized, (1, lidar_data_standardized.shape[1], 1)).astype(np.float32)
                        self._interpreteru.set_tensor(self._input_detailsu[0]['index'], lidar_data_standardized)
                        self._interpreteru.set_tensor(self._input_detailsu[1]['index'], color_data_standardized)
                        self._interpreteru.invoke()
                        predictions = self._interpreteru.get_tensor(self._output_detailsu[0]['index'])

                    self._X = predictions[0, 0]
                    self._Y = predictions[0, 1]
                    #self.get_logger().info('Predicted axes: "%s"' % predictions)

                    XX = int(self.servo_neutral+(self._X+self._Xtrim)*self.servo_ctl_fwd)
                    self._pwm.set_pwm(0, 0, XX)

                except ValueError as e:
                    self.get_logger().error('Model rendered nan: %s' % str(e))

                except IOError as e:
                    self.get_logger().error('IOError I2C occurred: %s' % str(e))


            ########################
            # IDLE
            ########################
            elif self._state == 'IDLE':
                pass

            self._processing = False

       
    def joy_callback(self, msg):
        if hasattr(msg, 'buttons') and len(msg.buttons) > 0:

            # Check if 'A' button is pressed - switch on AI steering, counterclockwise
            if msg.buttons[0] == 1:
                self.start_race()

            # Check if 'B' button is pressed - switch off AI steering
            elif msg.buttons[1] == 1:
                self.get_logger().info('emergency shutdown initiated by supervisor')
                self.stop_race()

        elif hasattr(msg, 'axes') and len(msg.axes) > 5:
            self._X = msg.axes[2]
            self._pwm.set_pwm(0, 0, int(self.servo_neutral+(self._X+self._Xtrim)*self.servo_ctl_fwd))

    def touch_button_callback(self, msg):
        ack = String()
        if not self._tf_control:
            ack.data = "Starting ..."
            self.publisher_.publish(ack)
            self.get_logger().info('Start button pressed!')
            self.start_race()
        else:
            ack.data = "Shutting down ..."
            self.publisher_.publish(ack)
            self.get_logger().info('Stop button pressed!')
            self.stop_race()

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
                    self._parking_lot += 1

                #self.get_logger().info('CAM: blob inserted: %s,%s,%s,%s' % (cam,color,x1,x2))

        except:
            self.get_logger().error('Faulty cam msg received: "%s"' % msg)

    def openmv_h7_callback1(self, msg):
        self.openmv_h7_callback(msg)
    def openmv_h7_callback2(self, msg):
        self.openmv_h7_callback(msg)

    def collision_callback(self, msg):
        self.get_logger().info('Collision msg received')
        self._collision = True
        return


def main(args=None):
    rclpy.init(args=args)

    full_drive_node = fullDriveNode()
    rclpy.spin(full_drive_node)
    full_drive_node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
