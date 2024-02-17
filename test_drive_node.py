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
from std_msgs.msg import Float32
import numpy as np
import tensorflow as tf
import pickle
#import logging
#from logging.handlers import RotatingFileHandler
from Adafruit_PCA9685 import PCA9685
from sense_hat import SenseHat
import RPi.GPIO as GPIO

class testDriveNode(Node):
    HPIX = 320
    VPIX = 200
    HFOV = 70.8
    MIN_DIST = 0.45
    scan_max_dist = 2.8
    num_scan = 1620
    num_scan2 = 810
    relay_pin = 17
    SPEED = "10"
    WEIGHT = 1

    servo_min = 230  # Min pulse length out of 4096
    servo_max = 385  # Max pulse length out of 4096
    servo_neutral = int((servo_max+servo_min)/2)
    servo_ctl = int(-(servo_max-servo_min)/2 * 1.7)

    def __init__(self):
        super().__init__('test_drive_node')
        self.publisher_ = self.create_publisher(String, 'main_logger', 10)
        self.speed_publisher_ = self.create_publisher(String, 'set_speed', 10)

        qos_profile = QoSProfile(
                depth=1, 
                history=QoSHistoryPolicy.KEEP_LAST, 
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE)
            
        self._processing = False
        self._tf_control = False
        self._clockwise = False
        self._X = 0.0 
        self._Y = 0.0
        self._Xtrim = 0.0
        self._line_cnt = 0
        self._dt = 0.1
        self._color1_g = np.zeros(self.HPIX, dtype=int)
        self._color1_r = np.zeros(self.HPIX, dtype=int)
        self._color2_g = np.zeros(self.HPIX, dtype=int)
        self._color2_r = np.zeros(self.HPIX, dtype=int)

        self._RED = False
        self._passed = False

        self._speed_msg = String()
        self._speed_msg.data = "0"

        # Initialize compass
        self._sense = SenseHat()
        self._initial_heading = self._sense.gyro['yaw']
        self._start_heading = self._initial_heading
        self._last_heading = self._initial_heading
        self._total_rounds = 0
        self._total_heading_change = 0
        self.get_logger().info(f"Initial heading: {self._initial_heading} degrees")

        # Initialize PCA9685
        self._pwm = PCA9685()
        self._pwm.set_pwm_freq(50)  # Set frequency to 50Hz

 #       self.get_logger().info('calibrating ESC')
 #       self._pwm.set_pwm(1, 0, self.neutral_pulse)

 #       msg = String()
 #       msg.data = "Switch on ESC"
 #       self.publisher_.publish(msg)

        self._start_time = self.get_clock().now()
        self._end_time = self._start_time
        self._round_start_time = self._start_time
        self._round_end_time = self._start_time

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

        # Load the trained model and the scaler
        tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)
        with open('/home/rrrschuetz/test/scaler.pkl', 'rb') as f:
            self._scaler = pickle.load(f)
            
        self._interpreter = tf.lite.Interpreter(model_path="/home/rrrschuetz/test/model.tflite")
        self._interpreter.allocate_tensors()
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        
        self.get_logger().info('prediction model loaded')

        msg = String()
        msg.data = "Ready!"
        self.publisher_.publish(msg)

    def __del__(self):
        self.get_logger().info('Switch off ESC')
        self.motor_off()

    def motor_off(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.LOW)
        GPIO.cleanup()

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

    def lidar_callback(self, msg):

        if not self._tf_control: return
        if self._processing:
            self.get_logger().info('Scan skipped')
            return
        else:
            self._processing = True

            # Round completion check
            self._current_heading = self._sense.gyro['yaw']
            heading_change = self.calculate_heading_change(self._last_heading, self._current_heading)
            #self.get_logger().info("Heading change: %s" % heading_change)
            if abs(heading_change) > 1:
                self._total_heading_change += heading_change
                self._last_heading = self._current_heading
                #self.get_logger().info("Current heading: %s degrees, total change: %s degrees" % (self._current_heading,self._total_heading_change))
                if abs(self._total_heading_change) >= 360:
                    self._total_rounds += 1
                    self._total_heading_change = 0
                    self._round_end_time = self.get_clock().now()
                    duration_in_seconds = (self._round_end_time - self._round_start_time).nanoseconds * 1e-9
                    self._round_start_time = self._round_end_time
                    self.get_logger().info(f"Round {self._total_rounds} in {duration_in_seconds} sec completed!")
                    if self._total_rounds >= 3:
                        self._total_rounds = 0
                        self._tf_control = False
                        self._processing = False
                        self._pwm.set_pwm(0, 0, int(self.servo_neutral))
 #                       self._pwm.set_pwm(1, 0, int(self.neutral_pulse))
                        self._speed_msg.data = "0"
                        self.speed_publisher_.publish(self._speed_msg)
                        self.get_logger().info("Race completed!")
                        return

            self._clockwise = (self._total_heading_change > 0)
            #if not self._clockwise:
            #    self.get_logger().info("Using CAM1")
            #else:
            #    self.get_logger().info("Using CAM2")

            self._start_time = self.get_clock().now()
            self._dt = (self._start_time - self._end_time).nanoseconds * 1e-9
            self._end_time = self._start_time

            try:
                # raw data
                #scan = np.array(msg.ranges)
                scan = np.array(msg.ranges[self.num_scan+self.num_scan2:]+msg.ranges[:self.num_scan2])

                # obstacle passing detector
                num_sections = 18
                section_data = np.array_split(scan, num_sections)
                section_means = [np.mean(section) for section in section_data]
                min_section_index = np.argmin(section_means)
                if section_means[min_section_index] < self.MIN_DIST:

                    #if self._RED: self.get_logger().info("RED in focus");
                    #else: self.get_logger().info("GREEN in focus");

                    if self._RED and min_section_index <= 2:
                        self._passed = True
                        self.get_logger().info("Obstacle to the left #%s in distance %s" % (min_section_index, section_means[min_section_index]))
                    elif not self._RED and min_section_index >= 15:
                        self._passed = True
                        self.get_logger().info("Obstacle to the right #%s in distance %s" % (min_section_index, section_means[min_section_index]))

                scan[scan == np.inf] = np.nan
                scan[scan > self.scan_max_dist] = np.nan
                x = np.arange(len(scan))
                finite_vals = np.isfinite(scan)
                scan_interpolated = np.interp(x, x[finite_vals], scan[finite_vals])
                scan_interpolated = [1/value if value != 0 else 0 for value in scan_interpolated]
                scan_interpolated = list(scan_interpolated)
                color_data = list(self._color1_g) + list(self._color2_g) + list(self._color1_r) + list(self._color2_r)
                
                lidar_data = np.reshape(scan_interpolated, (1, -1))  # Reshape LIDAR data
                lidar_data_standardized = self._scaler.transform(lidar_data)
                color_data_standardized = np.reshape(color_data, (1, -1))         # Reshape COLOR data
            
                # Reshape for TFLite model input
                lidar_data_standardized = np.reshape(lidar_data_standardized, (1, lidar_data_standardized.shape[1], 1)).astype(np.float32)

                # Reshape color_data to (1, 1, 1) to match dimensions
                color_data_standardized = np.reshape(color_data_standardized, (1, color_data_standardized.shape[1], 1)).astype(np.float32)

                # Combine LIDAR and color data for the model input (concatenation, as required by your model)
                self._interpreter.set_tensor(self._input_details[0]['index'], lidar_data_standardized)
                self._interpreter.set_tensor(self._input_details[1]['index'], color_data_standardized)

                # Run inference
                self._interpreter.invoke()

                # Retrieve the output of the model
                predictions = self._interpreter.get_tensor(self._output_details[0]['index'])

                self._X = predictions[0, 0]
                self._Y = predictions[0, 1]
                #self.get_logger().info('Predicted axes: "%s"' % predictions)

                XX = int(self.servo_neutral+(self._X+self._Xtrim)*self.servo_ctl)
                self._pwm.set_pwm(0, 0, XX)

            except ValueError as e:
                self.get_logger().error('Model rendered nan: %s' % str(e))

            except IOError as e:
                self.get_logger().error('IOError I2C occurred: %s' % str(e))

            self._processing = False
       
    def joy_callback(self, msg):
        #self.get_logger().info('current speed m/s: %s' % self._speed)

        if hasattr(msg, 'buttons') and len(msg.buttons) > 0:

            # Check if 'A' button is pressed - switch on AI steering, counterclockwise
            if msg.buttons[0] == 1:
                self._tf_control = True
                self._clockwise = False
                self._Y = 1.0
                self._start_heading = self._sense.gyro['yaw']
                self._last_heading = self._start_heading
                self._round_start_time = self.get_clock().now()
                self._speed_msg.data = self.SPEED
                self.speed_publisher_.publish(self._speed_msg)

            # Check if 'X' button is pressed - switch on AI steering, clockwise
            if msg.buttons[2] == 1:
                self._tf_control = True
                self._clockwise = True
                self._Y = 1.0
                self._start_heading = self._sense.gyro['yaw']
                self._last_heading = self._start_heading
                self._round_start_time = self.get_clock().now()
                self._speed_msg.data = self.SPEED
                self.speed_publisher_.publish(self._speed_msg)

            # Check if 'B' button is pressed - switch off AI steering
            elif msg.buttons[1] == 1:
                self.get_logger().info('emergency shutdown initiated by supervisor')
                self._tf_control = False
                self._processing = False
                self._pwm.set_pwm(0, 0, int(self.servo_neutral))
 #              self._pwm.set_pwm(1, 0, int(self.neutral_pulse))
                self._speed_msg.data = "0"
                self.speed_publisher_.publish(self._speed_msg)
                self.motor_off()

        elif hasattr(msg, 'axes') and len(msg.axes) > 5:
            self._X = msg.axes[2]
#           self._Y = msg.axes[1]
        
            #self.get_logger().info('Steering: %s,%s ' % (self._X,self._Xtrim))
#           #self.get_logger().info('Power: %s ' % self._Y)
            self._pwm.set_pwm(0, 0, int(self.servo_neutral+(self._X+self._Xtrim)*self.servo_ctl))
#           self._pwm.set_pwm(1, 0, int(self.neutral_pulse-self._Y*self.motor_ctl))

    def touch_button_callback(self, msg):
        ack = String()
        if not self._tf_control:
            self._tf_control = True
            self._Y = 1.0
            self._start_heading = self._sense.gyro['yaw']
            self._last_heading = self._start_heading
            self._round_start_time = self.get_clock().now()
            self._speed_msg.data = self.SPEED
            self.speed_publisher_.publish(self._speed_msg)
            ack.data = "Race Mode ON"
            self.get_logger().info('Start button pressed!')
        else:
            self._tf_control = False
            self._processing = False
            self._pwm.set_pwm(0, 0, int(self.servo_neutral))
 #          self._pwm.set_pwm(1, 0, int(self.neutral_pulse))
            self._speed_msg.data = "STOP"
            self.speed_publisher_.publish(self._speed_msg)
            ack.data = "Race Mode OFF"
            self.get_logger().info('Stop button pressed!')
        self.publisher_.publish(ack)

    def openmv_h7_callback1(self, msg):
        if self._clockwise: return
        #self.get_logger().info('cam msg received: "%s"' % msg)
        self._color1_g = np.zeros(self.HPIX, dtype=int)
        self._color1_r = np.zeros(self.HPIX, dtype=int)
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
            color = int(color)
            x1 = int(x1)
            x2 = int(x2)
            if color == 1:
                self._color1_g[x1:x2] = self.WEIGHT
                #self._color1_g[0:self.HPIX] = self.WEIGHT
                self._RED = False
                #self.get_logger().info('GREEN plan activated')
            if color == 2:
                self._color1_r[x1:x2] = self.WEIGHT
                #self._color1_r[0:self.HPIX] = self.WEIGHT
                self._RED = True
                #self.get_logger().info('RED plan activated')
            #self.get_logger().info('CAM1: blob inserted: %s,%s,%s' % (color,x1,x2))

    def openmv_h7_callback2(self, msg):
        if not self._clockwise: return
        #self.get_logger().info('cam msg received: "%s"' % msg)
        self._color2_g = np.zeros(self.HPIX, dtype=int)
        self._color2_r = np.zeros(self.HPIX, dtype=int)
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
            color = int(color)
            x1 = int(x1)
            x2 = int(x2)
            if color == 1:
                self._color2_g[x1:x2] = self.WEIGHT
                #self._color2_g[0:self.HPIX] = self.WEIGHT
                self._RED = False
                #self.get_logger().info('GREEN plan activated')
            if color == 2:
                self._color2_r[x1:x2] = self.WEIGHT
                #self._color2_r[0:self.HPIX] = self.WEIGHT
                self._RED = True
                #self.get_logger().info('RED plan activated')
            #self.get_logger().info('CAM2: blob inserted: %s,%s,%s' % (color,x1,x2))


class parkingNode(Node):
    HPIX = 320
    VPIX = 200
    HFOV = 70.8
    num_scan = 1620
    num_scan2 = 810
    num_scan3 = 405
    scan_max_dist = 2.8
    servo_min = 210  # Min pulse length out of 4096
    servo_max = 400  # Max pulse length out of 4096
    servo_neutral = int((servo_max+servo_min)/2)
    servo_ctl = int(-(servo_max-servo_min)/2 * 1.7)
    motor_ctl = -20
    relay_pin = 17
    WEIGHT = 1
    FWD_SPEED = "10"
    REV_SPEED = "-6"
    
    def __init__(self):
        super().__init__('test_drive_node')
        self.publisher_ = self.create_publisher(String, 'main_logger', 10)
        self.speed_publisher_ = self.create_publisher(String, 'set_speed', 10)

        qos_profile = QoSProfile(
                depth=1, 
                history=QoSHistoryPolicy.KEEP_LAST, 
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE)
            
        self._processing = False
        self._tf_control = False
        self._tf_parking = False
        self._dist_sensor = False
        self._collision = False
        self._X = 0.0 
        self._Y = 0.0
        self._front_dist = np.inf
        self._side_dist = np.inf

        self._color1_m = np.zeros(self.HPIX, dtype=int)
        self._color2_m = np.zeros(self.HPIX, dtype=int)

        self._speed_msg = String()
        self._speed_msg.data = "0"

        # Initialize PCA9685
        self._pwm = PCA9685()
        self._pwm.set_pwm_freq(50)  
        self._pwm.set_pwm(0, 0, int(self.servo_neutral))

        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
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

        self.subscription_lidar = self.create_subscription(
            String,
            'collision',
            self.collision_callback,
            qos_profile
        )

        self.subscription_distance = self.create_subscription(
            Float32,
            'distance_sensor',
            self.distance_sensor_callback,
            qos_profile
        )

        self.subscription_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            qos_profile
        )

        # Load the trained model and the scaler
        tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)
        with open('/home/rrrschuetz/test/scaler_p.pkl', 'rb') as f:
            self._scaler_p = pickle.load(f)
            
        self._interpreter_p = tf.lite.Interpreter(model_path="/home/rrrschuetz/test/model_p.tflite")
        self._interpreter_p.allocate_tensors()
        # Get input and output tensors information
        self._input_details_p = self._interpreter_p.get_input_details()
        self._output_details_p = self._interpreter_p.get_output_details()
        self.get_logger().info('parking prediction model loaded')

        msg = String()
        msg.data = "Ready!"
        self.publisher_.publish(msg)

    def __del__(self):
        self.get_logger().info('Switch off ESC')
        self.motor_off()

    def motor_off(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.LOW)
        GPIO.cleanup()

    def lidar_callback(self, msg):
        if self._processing:
            self.get_logger().info('Scan skipped')
            return
        else:
            self._processing = True
            # raw data
            scan = np.array(msg.ranges[self.num_scan+self.num_scan3:]+msg.ranges[:self.num_scan2+self.num_scan3])

            if self._tf_parking:
                num_sections = 18
                section_data = np.array_split(scan, num_sections)
                section_means = [np.mean(section) for section in section_data]
                self._front_dist = section_means[9]
                self._side_dist = min(section_means[3],section_means[15])
                self.get_logger().info('Parking Distance: %s,%s ' % (self._front_dist,self._side_dist))

                if self._front_dist > 0.13 and self._side_dist > 0.14:

                    self._X = 1.2 # right
                    XX = int(self.servo_neutral+self._X*self.servo_ctl)
                    self._pwm.set_pwm(0, 0, XX)
                    time.sleep(1)

                    self._speed_msg.data = "F5"
                    self.speed_publisher_.publish(self._speed_msg)
                    time.sleep(1)

                    self._X = -1.2  # left
                    XX = int(self.servo_neutral+self._X*self.servo_ctl)
                    self._pwm.set_pwm(0, 0, XX)
                    time.sleep(1)

                    self._speed_msg.data = "R5"
                    self.speed_publisher_.publish(self._speed_msg)
                    time.sleep(1)

                else:
                    self.get_logger().info('Parking ended ')
                    self._tf_parking = False
                    self._speed_msg.data = "STOP"
                    self.speed_publisher_.publish(self._speed_msg)

            elif self._tf_control:
                try:
                    scan[scan == np.inf] = np.nan
                    scan[scan > self.scan_max_dist] = np.nan
                    x = np.arange(len(scan))
                    finite_vals = np.isfinite(scan)
                    scan_interpolated = np.interp(x, x[finite_vals], scan[finite_vals])
                    scan_interpolated = [1/value if value != 0 else 0 for value in scan_interpolated]
                    scan_interpolated = list(scan_interpolated)
                    color_data = list(self._color1_m) + list(self._color2_m)

                    lidar_data = np.reshape(scan_interpolated, (1, -1))  # Reshape LIDAR data
                    lidar_data_standardized = self._scaler_p.transform(lidar_data)
                    color_data_standardized = np.reshape(color_data, (1, -1))         # Reshape COLOR data

                    lidar_data_standardized = np.reshape(lidar_data_standardized, (1, lidar_data_standardized.shape[1], 1)).astype(np.float32)
                    color_data_standardized = np.reshape(color_data_standardized, (1, color_data_standardized.shape[1], 1)).astype(np.float32)

                    self._interpreter_p.set_tensor(self._input_details_p[0]['index'], lidar_data_standardized)
                    self._interpreter_p.set_tensor(self._input_details_p[1]['index'], color_data_standardized)

                    # Run inference
                    self._interpreter_p.invoke()
                    # Retrieve the output of the model
                    predictions = self._interpreter_p.get_tensor(self._output_details_p[0]['index'])
                    self._X = predictions[0, 0]
                    self._Y = predictions[0, 1]

                    XX = int(self.servo_neutral+self._X*self.servo_ctl)
                    #self.get_logger().info('Steering: %s,%s ' % (self._X,XX))
                    #self.get_logger().info('Power: %s ' % self._Y)

                    if self._collision:
                        self.get_logger().info('Collision: STOP ')
                        self._collision = False
                        self._tf_control = False
                        self._speed_msg.data = "STOP"
                    else:
                        self._pwm.set_pwm(0, 0, XX)
                        if self._Y >= 0:
                            self._speed_msg.data = self.REV_SPEED
                            #self.get_logger().info('Reverse: %s / %s ' % (self._Y,self._speed_msg.data))
                        else:
                            self._speed_msg.data = self.FWD_SPEED
                            #self.get_logger().info('Forward: %s / %s ' % (self._Y,self._speed_msg.data))

                    self.speed_publisher_.publish(self._speed_msg)
            
                except ValueError as e:
                    self.get_logger().error('Model rendered nan: %s' % str(e))

                except IOError as e:
                    self.get_logger().error('IOError I2C occurred: %s' % str(e))

            self._processing = False

    def openmv_h7_callback1(self, msg):
        #self.get_logger().info('CAM1 msg received: "%s"' % msg)
        self._color1_m = np.zeros(self.HPIX, dtype=int)
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
            color = int(color)
            x1 = int(x1)
            x2 = int(x2)
            if color == 4:
                self._color1_m[x1:x2] = self.WEIGHT

    def openmv_h7_callback2(self, msg):
        #self.get_logger().info('CAM2 msg received: "%s"' % msg)
        self._color2_m = np.zeros(self.HPIX, dtype=int)
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
            color = int(color)
            x1 = int(x1)
            x2 = int(x2)
            if color == 4:
                self._color2_m[x1:x2] = self.WEIGHT

    def joy_callback(self, msg):
        #self.get_logger().info('current speed m/s: %s' % self._speed)

        if hasattr(msg, 'buttons') and len(msg.buttons) > 0:

            # Check if 'A' button is pressed - switch on AI steering, counterclockwise
            if msg.buttons[0] == 1:
                self._speed_msg.data = "RESET"
                self.speed_publisher_.publish(self._speed_msg)
                self._speed_msg.data = self.FWD_SPEED
                self.speed_publisher_.publish(self._speed_msg)
                self._tf_control = True
                self._dist_sensor = True

            # Check if 'B' button is pressed - switch off all steering
            elif msg.buttons[1] == 1:
                self.get_logger().info('emergency shutdown initiated by supervisor')
                self._tf_control = False
                self._tf_parking = False
                self._processing = False
                self._pwm.set_pwm(0, 0, int(self.servo_neutral))
                self.motor_off()

            # Check if 'X' button is pressed - test move
            elif msg.buttons[2] == 1:
                self._tf_parking = True
                self._dist_sensor = False

    def distance_sensor_callback(self, msg):
        if not self._dist_sensor: return
        #self.get_logger().info('Distance msg received: "%s"' % msg)
        if float(msg.data) < 0.09:
            self.get_logger().info('Parking mode switched')
            self._tf_control = False
            self._speed_msg.data = "0"
            self.speed_publisher_.publish(self._speed_msg)
            self._tf_parking = True
            self._dist_sensor = False
        return

    def collision_callback(self, msg):
        self.get_logger().info('Collision msg received')
        self._collision = True
        return

def main(args=None):
    rclpy.init(args=args)
    
#    test_drive_node = testDriveNode()
#    rclpy.spin(test_drive_node)
#    test_drive_node.destroy_node()

    parking_node = parkingNode()
    rclpy.spin(parking_node)
    parking_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

