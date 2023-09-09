import rclpy, math, time
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

#from .Motor import PwmMotor 
from Adafruit_PCA9685 import PCA9685

class testDriveNode(Node):

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
   
        self._X = 0.0 
        self._Y = 0.0
        self._color = np.zeros(3240)
        
        #self._motor = PwmMotor()
        #self._motor.setMotorModel(0,0,0,0)
        
        # Initialize PCA9685
        self._pwm = PCA9685()
        self._pwm.set_pwm_freq(50)  # Set frequency to 50Hz

        self.get_logger().info('calibrating ESC')
        self._pwm.set_pwm(1, 0, self.neutral_pulse)
        #time.sleep(10)

        self._sense = SenseHat()
        self._sense.clear()
        self._sense.show_message("OK", text_colour=[255, 0, 0])
        
        self._counter = 0
        self._start_time = time.time()
        self._end_time = self._start_time

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
        
        self._processing = False
    
        # Load the trained model and the scaler
        self._model = tf.keras.models.load_model('/home/rrrschuetz/test/model')
        with open('/home/rrrschuetz/test/scaler.pkl', 'rb') as f:
            self._scaler = pickle.load(f)

    def lidar_callback(self, msg):
        
        if not self._processing:
            self._processing = True
            start_time = time.time()
            self._counter += 1
            
            if self._counter % 1 == 0:
                try:
                    scan = np.array(msg.ranges)
                    scan[scan == np.inf] = np.nan
                    x = np.arange(len(scan))
                    finite_vals = np.isfinite(scan)
                    scan_interpolated = np.interp(x,x[finite_vals],scan[finite_vals])

                    # add color data
                    combined = np.hstack((scan_interpolated, self._color))
                    
                    # add magentometer data
                    mag = self._sense.get_compass_raw()
                    combined.append({mag['x']})
                    combined.append({mag['y']})
                    combined.append({mag['z']})
        
                    # add accelerometer data
                    accel = self._sense.get_accelerometer_raw()
                    combined.append({accel['x']})
                    combined.append({accel['y']})
                    combined.append({accel['z']})

                    # add gyroscope data
                    gyro = self._sense.get_gyroscope_raw()
                    combined.append({gyro['x']})
                    combined.append({gyro['y']})
                    combined.append({gyro['z']})
                    
                    combined = np.reshape(combined, (1, -1))
                    predictions = self._model.predict(combined)
                    self._X = predictions[0,0]
                    self._Y = predictions[0,1]
                    self.get_logger().info('Predicted axes: "%s"' % predictions)
                    
                    #self._motor.setMotorModel(
                    #    int(700*self._Y*(1+1.7*self._X)),
                    #    int(700*self._Y*(1+1.7*self._X)),
                    #    int(700*self._Y*(1-1.7*self._X)),
                    #    int(700*self._Y*(1-1.7*self._X)))
        
                    self.get_logger().info('Steering: "%s"' % str(self.servo_neutral+self._X*self.servo_ctl))
                    self.get_logger().info('Power: "%s"' % str(self.neutral_pulse+self._Y*40))

                    scan = np.array(msg.ranges)
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
                    combined.extend([accel['x'], accel['y'], accel['z']])

                    # add gyroscope data
                    gyro = self._sense.get_gyroscope_raw()
                    combined.extend([gyro['x'], gyro['y'], gyro['z']])

                    # Reshape and standardize
                    combined = np.reshape(combined, (1, -1))
                    combined_standardized = self._scaler.transform(combined)

                    # reshape for 1D CNN input
                    combined_standardized = np.reshape(combined_standardized, (combined_standardized.shape[0], combined_standardized.shape[1], 1))

                    # Model prediction
                    predictions = self._model.predict(combined_standardized)
                    self._X = predictions[0, 0]
                    self._Y = predictions[0, 1]

                    self.get_logger().info('Predicted axes: "%s"' % predictions)

                    # Uncomment below if you want to set the motor models
                    # self._motor.setMotorModel(
                    #     int(700*self._Y*(1+1.7*self._X)),
                    #     int(700*self._Y*(1+1.7*self._X)),
                    #     int(700*self._Y*(1-1.7*self._X)),
                    #     int(700*self._Y*(1-1.7*self._X)))

                    self.get_logger().info('Steering: "%s"' % str(self.servo_neutral + self._X * self.servo_ctl))
                    self.get_logger().info('Power: "%s"' % str(self.neutral_pulse + self._Y * 40))

                    #self._pwm.set_pwm(0, 0, int(self.servo_neutral+self._X*self.servo_ctl))
                    #self._pwm.set_pwm(1, 0, int(self.neutral_pulse+self._Y*40))
        
                except ValueError as e:
                    self.get_logger().error('Model rendered nan: %s' % str(e))

            self._end_time = time.time() 
            self.get_logger().info('Scans processed "%s"' % self._counter)
            self.get_logger().info('Cycle time: "%s" seconds' % (self._end_time-self._start_time))
            self.get_logger().info('Step time: "%s" seconds' % (self._end_time-start_time))
            self._start_time = self._end_time
            self._processing = False
        else:
            self.get_logger().info('Scan skipped')


    def joy_callback(self, msg):
        #self.get_logger().info('Buttons: "%s"' % msg.buttons)
        #self.get_logger().info('Axes: "%s"' % msg.axes)
        self._X = msg.axes[2]
        self._Y = msg.axes[1]
        
        #self._motor.setMotorModel(
        #    int(1000*self._Y*(1+self._X)),
        #    int(1000*self._Y*(1+self._X)),
        #    int(1000*self._Y*(1-self._X)),
        #    int(1000*self._Y*(1-self._X)))

        self.get_logger().info('Steering: "%s"' % str(self.servo_neutral+self._X*self.servo_ctl))
        self.get_logger().info('Power: "%s"' % str(self.neutral_pulse+self._Y*40))
                    
        #self._pwm.set_pwm(0, 0, int(self.servo_neutral+self._X*self.servo_ctl))
        #self._pwm.set_pwm(1, 0, int(self.neutral_pulse+self._Y*40))


    def openmv_h7_callback(self, msg):

        HPIX2 = 160
        VPIX2 = 120
        HFOV = 70.8
        VFOV = 55.6

        self._color = np.zeros(3240)
        self.get_logger().info('blob detected: %s' % msg.data)
        try:
            color, y1, y2 = msg.data.split(',')
            if float(color) > 0.0:
                #alphaH=(HPIX2-cxy[0])/HPIX2*HFOV/2*math.pi/180
                alphaV1=(float(x1)-HPIX2)/HPIX2
                alphaV2=(float(x2)-HPIX2)/HPIX2
                idx1 = int(alphaV1*320)+1620
                idx2 = int(alphaV2*320)+1620
                self._color[idx1:idx2+1] = float(color)
                self.get_logger().info('blob inserted: %s,%s,%s' % (color,idx1,idx2))

                # sense hat
                ish1 = int(alphaV1*4)+4
                ish2 = int(alphaV2*4)+4
                if color == 1.0:
                    pixcol = blue
                else:
                    pixcol = red
                self._sense.clear()
                for i in range(ish1,ish2+1):
                    self._sense.set_pixel(0,7-i,pixcol)
                    #self._sense.set_pixel(1,7-i,pixcol)
        
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

