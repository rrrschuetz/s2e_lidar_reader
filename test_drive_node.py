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
import tensorflow as tf
import pickle
import logging
from logging.handlers import RotatingFileHandler
from Adafruit_PCA9685 import PCA9685

class testDriveNode(Node):
    HPIX = 320
    HFOV = 70.8
    MIN_DIST = 0.20
    MIN_SPEED = -0.50
    MAX_SPEED = -0.80
    reverse_pulse = 204
    neutral_pulse = 307
    forward_pulse = 409
    servo_min = 260  # Min pulse length out of 4096
    servo_max = 375  # Max pulse length out of 4096
    servo_neutral = int((servo_max+servo_min)/2)
    servo_ctl = int((servo_max-servo_min)/2)
    
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
        self._cx1 = 0
        self._cx2 = 0
        self._color = np.zeros(self.HPIX)

        # Initialize sense hat
        self._sense = SenseHat()
        self._sense.clear()
        
        # Initialize PCA9685
        self.get_logger().info('calibrating ESC')
        self._pwm = PCA9685()
        self._pwm.set_pwm_freq(50)  # Set frequency to 50Hz
        self._pwm.set_pwm(1, 0, self.neutral_pulse)
        self._sense.show_message("ESC", text_colour=[0, 255, 0])

        self._counter = 0
        self._start_time = 0
        self._end_time = self.get_clock().now()

        self._custom_logger = self.setup_custom_logger('/home/rrrschuetz/test/logfile.txt')

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
        self.subscription_h7 = self.create_subscription(
            String,
            'openmv_topic',
            self.openmv_h7_callback,
            qos_profile)
      
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

        self._sense.show_message("OK", text_colour=[0, 255, 0])
        self.get_logger().info('prediction model loaded')

    def setup_custom_logger(self, filename):
        logger = logging.getLogger('rclpy')
        logger.setLevel(logging.DEBUG)
    
        # Create a file handler and set its level to debug
        handler = RotatingFileHandler(filename, maxBytes=1024*1024*10, backupCount=5) # 10MB file
        handler.setLevel(logging.DEBUG)
    
        # Create a logging format
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
    
        # Add the handler to the logger
        logger.addHandler(handler)
        return logger
    
    def lidar_callback(self, msg):
        if not self._tf_control: return
        if self._processing:
            self.get_logger().info('Scan skipped')
            return
        else:
            self._processing = True
            current_time = self.get_clock().now()
            cycle_age = (self._end_time - current_time).nanoseconds * 1e-9
            message_time = Time.from_msg(msg.header.stamp)
            message_age = (current_time - message_time).nanoseconds * 1e-9
            self._start_time = current_time       
            try:
                # raw data
                scan = np.array(msg.ranges)
                    
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
                    self._pwm.set_pwm(1, 0, int(self.neutral_pulse))
                    self._sense.show_message("STOP", text_colour=[255, 0, 0])
                    self.get_logger().info('Emergency brake active: "%s"' % min_section_index)
                    return
        
                scan[scan == np.inf] = np.nan
                x = np.arange(len(scan))
                finite_vals = np.isfinite(scan)
                scan_interpolated = np.interp(x, x[finite_vals], scan[finite_vals])

                # add color data
                combined = list(scan_interpolated)  # Convert to list for easier appending
                combined.extend(self._color)

                # add magnetometer data
                mag = self._sense.get_compass_raw()
                combined.extend([mag['x'], mag['y'], mag['z']])
                # add accelerometer data
                accel = self._sense.get_accelerometer_raw()
                #combined.extend([accel['x'], accel['y'], accel['z']])
                combined.extend([0.0,0.0,0.0])
                # add gyroscope data
                gyro = self._sense.get_gyroscope_raw()
                combined.extend([gyro['x'], gyro['y'], gyro['z']])
                    
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
                self._Y = min(max(predictions[0, 1],self.MAX_SPEED),self.MIN_SPEED)
                #self.get_logger().info('Predicted axes: "%s"' % predictions)

                #self.get_logger().info('Steering: "%s"' % str(self.servo_neutral + self._X * self.servo_ctl))
                #self.get_logger().info('Power: "%s"' % str(self.neutral_pulse + self._Y * 40))
                self._pwm.set_pwm(0, 0, int(self.servo_neutral+self._X*self.servo_ctl))
                self._pwm.set_pwm(1, 0, int(self.neutral_pulse+self._Y*40))
        
            except ValueError as e:
                self.get_logger().error('Model rendered nan: %s' % str(e))

            self._end_time = self.get_clock().now()
            call_age = (self._end_time - self._start_time).nanoseconds * 1e-9
            self._custom_logger.info('Cycle age: {:.4f} seconds, Call age: {:.4f}, Message age: {:.4f} seconds', cycle_age,call_age,message_age)
            self._processing = False
       
    def joy_callback(self, msg):
        self.get_logger().info('Buttons: "%s"' % msg.buttons)
        self.get_logger().info('Axes: "%s"' % msg.axes)

        if hasattr(msg, 'buttons') and len(msg.buttons) > 0:
            # Check if 'A' button is pressed - switch on AI steering
            if msg.buttons[0] == 1:
                self._tf_control = True
                self._sense.show_message("ON", text_colour=[0, 0, 255])
            # Check if 'B' button is pressed - switch off AI steering
            if msg.buttons[1] == 1:
                self._tf_control = False
                self._sense.show_message("OFF", text_colour=[0, 0, 255])

        elif hasattr(msg, 'axes') and len(msg.axes) > 2:
            self._X = msg.axes[2]
            self._Y = msg.axes[1]

        #self.get_logger().info('Steering: "%s"' % str(self.servo_neutral+self._X*self.servo_ctl))
        #self.get_logger().info('Power: "%s"' % str(self.neutral_pulse+self._Y*40))     
        self._pwm.set_pwm(0, 0, int(self.servo_neutral+self._X*self.servo_ctl))
        self._pwm.set_pwm(1, 0, int(self.neutral_pulse+self._Y*40))

    def openmv_h7_callback(self, msg):
        blue = (0,0,255)
        red = (255,0,0)

        self._color = np.zeros(self.HPIX)
        #self.get_logger().info('blob detected: %s' % msg.data)
        try:
            color, x1, x2 = msg.data.split(',')
            cx1 = int(x1)
            cx2 = int(x2)
            fcol = float(color)
            if fcol > 0.0:
                self._color[cx1:cx2+1] = fcol
                #self.get_logger().info('blob inserted: %s,%s,%s' % (color,x1,x2))
                # sense hat
                pixcol = blue if fcol == 1.0 else red
                if cx1 != self._cx1 or cx2 != self._cx2: self._sense.clear()
                for i in range(int(cx1/40),int(cx2/40)+1):
                    self._sense.set_pixel(0,min(i,7),pixcol)
                    self._sense.set_pixel(1,min(i,7),pixcol)
                self._cx1 = cx1
                self._cx2 = cx2

        except (SyntaxError) as e:
            self.get_logger().error('Failed to get blob coordinates: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    test_drive_node = testDriveNode()

    executor = SingleThreadedExecutor()
    executor.add_node(test_drive_node)

    try:
        executor.spin()  # Handle callbacks until shutdown
    finally:
        # Shutdown and cleanup
        executor.shutdown()
        test_drive_node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()

