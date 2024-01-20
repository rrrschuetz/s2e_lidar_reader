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
#import logging
#from logging.handlers import RotatingFileHandler
from Adafruit_PCA9685 import PCA9685
from sense_hat import SenseHat

#class PIDController:
#    def __init__(self, kp, ki, kd):
#        self.kp = kp
#        self.ki = ki
#        self.kd = kd
#        self._previous_error = 0
#        self._integral = 0
#    def update(self, setpoint, measured_value):
#        error = setpoint - measured_value
#        self._integral += error
#        derivative = error - self._previous_error
#        output = self.kp * error + self.ki * self._integral + self.kd * derivative
#        self._previous_error = error
#        return output

class testDriveNode(Node):
    HPIX = 320
    VPIX = 200
    HFOV = 70.8
    MIN_DIST = 0.15
    scan_max_dist = 2.8
    num_scan = 1620
    num_scan2 = 810
    SPEED = "10"

#    reverse_pulse = 204
#    neutral_pulse = 307
#    forward_pulse = 409
    
    servo_min = 230  # Min pulse length out of 4096
    servo_max = 385  # Max pulse length out of 4096
    servo_neutral = int((servo_max+servo_min)/2)
    servo_ctl = int(-(servo_max-servo_min)/2 * 1.7)

#    speed_min = 0.1
#    speed_max = 2.0
#    speed_target = 0.7
#    Ymax = 0.2
#    motor_ctl = 36
    
    def __init__(self):
        super().__init__('s2e_lidar_reader_node')
        self.publisher_ = self.create_publisher(String, 'main_logger', 10)
        self.speed_publisher_ = self.create_publisher(String, 'set_speed', 10)

        qos_profile = QoSProfile(
                depth=1, 
                history=QoSHistoryPolicy.KEEP_LAST, 
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE)
            
        self._processing = False
        self._tf_control = False
        self._X = 0.0 
#        self._Y = 0.0
        self._Xtrim = 0.0
#        self._speed = 0.0
        self._line_cnt = 0
        self._dt = 0.1
        self._cx1 = 0
        self._cx2 = 0
        self._color1 = np.zeros(self.HPIX)
        self._color2 = np.zeros(self.HPIX)

        self._speed_msg = String()
        self._speed_msg.data = "0"

 #       #self.pid_controller = PIDController(kp=0.1, ki=0.01, kd=0.05)  # Tune these parameters
 #       self.pid_controller = PIDController(kp=16, ki=0.5, kd=0.0)  # Tune these parameters

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

#        self.subscription_speed = self.create_subscription(
#            String,
#            'speed_monitor',
#            self.speed_monitor_callback,
#            qos_profile
#        )

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
            
        #self._model = tf.keras.models.load_model('/home/rrrschuetz/test/model')
        self._interpreter = tf.lite.Interpreter(model_path="/home/rrrschuetz/test/model.tflite")
        self._interpreter.allocate_tensors()
        # Get input and output tensors information
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        self.get_logger().info('prediction model loaded')

        msg = String()
        msg.data = "Ready!"
        self.publisher_.publish(msg)

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
        #self.get_logger().info('current speed m/s: %s' % self._speed)

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
                        self._tf_control = False
                        self._processing = False
                        self._pwm.set_pwm(0, 0, int(self.servo_neutral))
 #                       self._pwm.set_pwm(1, 0, int(self.neutral_pulse))
                        self._speed_msg.data = "0"
                        self.speed_publisher_.publish(self._speed_msg)
                        self.get_logger().info("Race completed!")
                        return

            self._start_time = self.get_clock().now()
            self._dt = (self._start_time - self._end_time).nanoseconds * 1e-9
            self._end_time = self._start_time

            try:
                # raw data
                #scan = np.array(msg.ranges)
                scan = np.array(msg.ranges[self.num_scan+self.num_scan2:]+msg.ranges[:self.num_scan2])

                # emergency brake assistant
                num_sections = 36
                section_data = np.array_split(scan, num_sections)
                section_means = [np.mean(section) for section in section_data]
                min_section_index = np.argmin(section_means)
                #self.get_logger().info('Min distance: "%s"' % section_means[min_section_index])
                if section_means[min_section_index] < self.MIN_DIST:
                    self._tf_control = False
                    self._processing = False
                    self._pwm.set_pwm(0, 0, int(self.servo_neutral))
 #                   self._pwm.set_pwm(1, 0, int(self.neutral_pulse))
                    self._speed_msg.data = "0"
                    self.speed_publisher_.publish(self._speed_msg)
                    self.get_logger().info('Emergency brake active: "%s"' % min_section_index)
                    return
        
                scan[scan == np.inf] = np.nan
                scan[scan > self.scan_max_dist] = np.nan
                x = np.arange(len(scan))
                finite_vals = np.isfinite(scan)
                scan_interpolated = np.interp(x, x[finite_vals], scan[finite_vals])

                # add color data
                combined = list(scan_interpolated)  # Convert to list for easier appending
                combined.extend(self._color1)
                combined.extend(self._color2)

                # Reshape and standardize
                combined = np.reshape(combined, (1, -1))
                combined_standardized = self._scaler.transform(combined)

                # reshape for 1D CNN input
                combined_standardized = np.reshape(combined_standardized, (combined_standardized.shape[0], combined_standardized.shape[1], 1))
                combined_standardized = combined_standardized.astype(np.float32)

                # Model prediction
                #predictions = self._model.predict(combined_standardized)

                # Set the value of the input tensor
                self._interpreter.set_tensor(self._input_details[0]['index'], combined_standardized)
                # Run inference
                self._interpreter.invoke()
                # Retrieve the output of the model
                predictions = self._interpreter.get_tensor(self._output_details[0]['index'])
                
                self._X = predictions[0, 0]
                #self._Y = predictions[0, 1]
                #self.get_logger().info('Predicted axes: "%s"' % predictions)

#                #self.get_logger().info('current speed m/s: %s' % self._speed)

#                if self._speed > self.speed_max:
#                    self._Y = 0
#                    self.get_logger().info('emergency brake, max speed exceeded')
#                else:
#                    self._Y = max(0,min(self.Ymax,self.pid_controller.update(self.speed_target, self._speed)))  # max() for safety!

                XX = int(self.servo_neutral+(self._X+self._Xtrim)*self.servo_ctl)
#                YY = int(self.neutral_pulse+self._Y*self.motor_ctl)
                #self.get_logger().info('Steering: %s,%s ' % (self._X,self._Xtrim))
#                #self.get_logger().info('Power: %s,%s,%s ' % (self._Y,YY,self._dt))

                self._pwm.set_pwm(0, 0, XX)
#                self._pwm.set_pwm(1, 0, YY)
            
            except ValueError as e:
                self.get_logger().error('Model rendered nan: %s' % str(e))

            self._processing = False
       
    def joy_callback(self, msg):
        #self.get_logger().info('current speed m/s: %s' % self._speed)

        if hasattr(msg, 'buttons') and len(msg.buttons) > 0:
            # Check if 'A' button is pressed - switch on AI steering
            if msg.buttons[0] == 1:
                self._tf_control = True
                self._Y = 1.0
                self._start_heading = self._sense.gyro['yaw']
                self._last_heading = self._start_heading
                self._round_start_time = self.get_clock().now()
                self._speed_msg.data = self.SPEED
                self.speed_publisher_.publish(self._speed_msg)
            # Check if 'B' button is pressed - switch off AI steering
            elif msg.buttons[1] == 1:
                self._tf_control = False
            # Check if 'X' button is pressed - trim right
            elif msg.buttons[2] == 1:
                self._Xtrim += 0.05
                self.get_logger().info('X Trim: "%s"' % self._Xtrim)
            # Check if 'Y' button is pressed - trim left
            elif msg.buttons[3] == 1:
                self._Xtrim -= 0.05
                self.get_logger().info('X Trim: "%s"' % self._Xtrim)

        elif hasattr(msg, 'axes') and len(msg.axes) > 5:
            self._X = msg.axes[2]
#           self._Y = msg.axes[1]
        
            #self.get_logger().info('Steering: %s,%s ' % (self._X,self._Xtrim))
#           #self.get_logger().info('Power: %s ' % self._Y)
            self._pwm.set_pwm(0, 0, int(self.servo_neutral+(self._X+self._Xtrim)*self.servo_ctl))
#           self._pwm.set_pwm(1, 0, int(self.neutral_pulse-self._Y*self.motor_ctl))
            self._speed_msg.data = self.SPEED
            self.speed_publisher_.publish(self._speed_msg)

    def touch_button_callback(self, msg):
        if not self._tf_control:
            self._tf_control = True
            self._Y = 1.0
            self._start_heading = self._sense.gyro['yaw']
            self._last_heading = self._start_heading
            self._round_start_time = self.get_clock().now()
            self._speed_msg.data = self.SPEED
            self.speed_publisher_.publish(self._speed_msg)
            ack = String()
            ack.data = "Race Mode ON"
            self.publisher_.publish(ack)
            self.get_logger().info('Start button pressed!')
        else:
            self._tf_control = False
            self._processing = False
            self._pwm.set_pwm(0, 0, int(self.servo_neutral))
 #          self._pwm.set_pwm(1, 0, int(self.neutral_pulse))
            self._speed_msg.data = "-1"
            self.speed_publisher_.publish(self._speed_msg)


    def openmv_h7_callback1(self, msg):
        #self.get_logger().info('cam msg received: "%s"' % msg)
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
                # self.get_logger().info('CAM1: blob inserted: %s,%s,%s' % (color,x1,x2))

    def openmv_h7_callback2(self, msg):
        #self.get_logger().info('cam msg received: "%s"' % msg)
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
                # self.get_logger().info('CAM2: blob inserted: %s,%s,%s' % (color,x1,x2))
    
#    def speed_monitor_callback(self, msg):
#        self._speed = eval(msg.data)
#        #self.get_logger().warning("speed update received %s" % self._speed)


class parkingNode(Node):
    num_scan = 1620
    num_scan2 = 810
    
    reverse_pulse = 204
    neutral_pulse = 307
    forward_pulse = 409
    
    servo_min = 240  # Min pulse length out of 4096
    servo_max = 375  # Max pulse length out of 4096
    servo_neutral = int((servo_max+servo_min)/2)
    servo_ctl = int(-(servo_max-servo_min)/2 * 1.7)

    motor_ctl = 12
    
    def __init__(self):
        super().__init__('s2e_lidar_reader_node')

        qos_profile = QoSProfile(
                depth=1, 
                history=QoSHistoryPolicy.KEEP_LAST, 
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE)
            
        self._processing = False
        self._tf_control = False
        self._X = 0.0 
        self._Y = 0.0
        self._speed = 0.0

        # Initialize PCA9685
        self._pwm = PCA9685()
        self._pwm.set_pwm_freq(50)  
        self._pwm.set_pwm(0, 0, int(self.servo_neutral))
        self._pwm.set_pwm(1, 0, self.neutral_pulse)
 
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        
        # Load the trained model and the scaler
        tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)
        with open('/home/rrrschuetz/test/scaler_p.pkl', 'rb') as f:
            self._scaler = pickle.load(f)
            
        self._interpreter = tf.lite.Interpreter(model_path="/home/rrrschuetz/test/model_p.tflite")
        self._interpreter.allocate_tensors()
        # Get input and output tensors information
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        self.get_logger().info('parking prediction model loaded')

    def lidar_callback(self, msg):
        if not self._tf_control: return
        if self._processing:
            self.get_logger().info('Scan skipped')
            return
        else:
            self._processing = True

            try:
                # raw data
                #scan = np.array(msg.ranges)
                scan = np.array(msg.ranges[self.num_scan+self.num_scan2:]+msg.ranges[:self.num_scan2])
        
                scan[scan == np.inf] = np.nan
                scan[scan > self.scan_max_dist] = np.nan
                x = np.arange(len(scan))
                finite_vals = np.isfinite(scan)
                scan_interpolated = np.interp(x, x[finite_vals], scan[finite_vals])

                # add color data
                combined = list(scan_interpolated)  # Convert to list for easier appending
                combined = np.reshape(combined, (1, -1))
                combined_standardized = self._scaler.transform(combined)

                # reshape for 1D CNN input
                combined_standardized = np.reshape(combined_standardized, (combined_standardized.shape[0], combined_standardized.shape[1], 1))
                combined_standardized = combined_standardized.astype(np.float32)

                # Model prediction
                #predictions = self._model.predict(combined_standardized)

                # Set the value of the input tensor
                self._interpreter.set_tensor(self._input_details[0]['index'], combined_standardized)
                # Run inference
                self._interpreter.invoke()
                # Retrieve the output of the model
                predictions = self._interpreter.get_tensor(self._output_details[0]['index'])
                
                self._X = predictions[0, 0]
                self._Y = predictions[0, 1]
                #self.get_logger().info('Predicted axes: "%s"' % predictions)
                #self.get_logger().info('current speed m/s: %s' % self._speed)

                XX = int(self.servo_neutral+(self._X+self._Xtrim)*self.servo_ctl)
                YY = int(self.neutral_pulse+self._Y*self.motor_ctl)
                #self.get_logger().info('Steering: %s,%s ' % (self._X,XX))
                #self.get_logger().info('Power: %s,%s ' % (self._Y,YY))

                self._pwm.set_pwm(0, 0, XX)
                self._pwm.set_pwm(1, 0, YY)
            
            except ValueError as e:
                self.get_logger().error('Model rendered nan: %s' % str(e))

            self._processing = False


def main(args=None):
    rclpy.init(args=args)
    
    test_drive_node = testDriveNode()
    rclpy.spin(test_drive_node)
    test_drive_node.destroy_node()
    
#    parking_node = parkingNode()
#    rclpy.spin(parking_node)
#    parking_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

