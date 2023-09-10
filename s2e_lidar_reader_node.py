import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from sense_hat import SenseHat
import numpy as np

from Adafruit_PCA9685 import PCA9685

class s2eLidarReaderNode(Node):
    HPIX = 320
    HFOV = 70.8
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

        self._scan_interpolated = np.zeros(3240)
        self._color = np.zeros(HPIX)
        self._X = 0.0 
        self._Y = 0.0

        self._sense = SenseHat()
        self._sense.clear()
        self._sense.show_message("OK", text_colour=[255, 0, 0])

        # Read temperature
        temp = self._sense.get_temperature()
        self.get_logger().info(f"Temperature: {temp}C")
        # Read humidity
        humidity = self._sense.get_humidity()
        self.get_logger().info(f"Humidity: {humidity}%")
        # Read pressure
        pressure = self._sense.get_pressure()
        self.get_logger().info(f"Pressure: {pressure}mbar")
        # Read accelerometer data
        accel = self._sense.get_accelerometer_raw()
        self.get_logger().info(f"Accelerometer: x={accel['x']}, y={accel['y']}, z={accel['z']}")
        # Read gyroscope data
        gyro = self._sense.get_gyroscope_raw()
        self.get_logger().info(f"Gyroscope: x={gyro['x']}, y={gyro['y']}, z={gyro['z']}")
        # Read magnetometer data
        mag = self._sense.get_compass_raw()
        self.get_logger().info(f"Magnetometer: x={mag['x']}, y={mag['y']}, z={mag['z']}")

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

        self.subscription_h7 = self.create_subscription(
            String,
            'openmv_topic',
            self.openmv_h7_callback,
            qos_profile
        )

    with open('/home/rrrschuetz/test/file.txt', 'a') as f:
        f.write('X,Y,' + ','.join(['SCAN']*3240) + ','+','.join(['COLR']*HPIX)
            +',MAGX,MAGY,MAGZ,ACCX,ACCY,ACCZ,GYRX,GYRY,GYRZ\n')

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
        scan_data += ','.join(str(e) for e in self._color)
        
        # add magentometer data
        mag = self._sense.get_compass_raw()
        scan_data += ','+str({mag['x']})+','+str({mag['y']})+','+str({mag['z']})
        # add accelerometer data
        accel = self._sense.get_accelerometer_raw()
        scan_data += ','+str({accel['x']})+','+str({accel['y']})+','+str({accel['z']})
        # add gyroscope data
        gyro = self._sense.get_gyroscope_raw()
        scan_data += ','+str({gyro['x']})+','+str({gyro['y']})+','+str({gyro['z']})
        
        # Write the scan data to a file
        with open('/home/rrrschuetz/test/file.txt', 'a') as f:
            f.write(scan_data + '\n')

    def calculate_steering_angle():
        max_steering_angle = 40
        
        # Number of sections to split the LiDAR data into
        num_sections = 36  # i.e., each section covers 10 degrees

        # Split data into sections
        arr = self._scan_interpolated[:810] + self._scan_interpolated[2430:]
        arr = arr[-810:] + arr[:810]
        section_data = np.array_split(arr, num_sections)

        # Calculate the mean distance in each section
        section_means = [np.mean(section) for section in section_data]

        # Find the section with the maximum mean distance
        max_section_index = np.argmax(section_means)

        # Calculate the steering angle
        # Assuming 0 degrees is straight ahead, -180 is far left, and 180 is far right
        steering_angle = (max_section_index - num_sections / 2) * (360.0 / num_sections)

        X = min(max_steering_angle,abs(steering_angle))/max_steering_angle
        X = X if steering_angle >= 0 else -X
        return X

    def joy_callback(self, msg):
        #self.get_logger().info('Buttons: "%s"' % msg.buttons)
        #self.get_logger().info('Axes: "%s"' % msg.axes)

        self._X = self.calculate_steering_angle()
        #self._X = msg.axes[2]
        self._Y = msg.axes[1]

        self.get_logger().info('Steering: "%s"' % str(self.servo_neutral+self._X*self.servo_ctl))
        self.get_logger().info('Power: "%s"' % str(self.neutral_pulse+self._Y*40))
        #self._pwm.set_pwm(0, 0, int(self.servo_neutral+self._X*self.servo_ctl))
        #self._pwm.set_pwm(1, 0, int(self.neutral_pulse+self._Y*40))

    def openmv_h7_callback(self, msg):
        blue = (0,0,255)
        red = (255,0,0)

        self._color = np.zeros(HPIX)
        #self.get_logger().info('blob detected: %s' % msg.data)
        try:
            color, x1, x2 = msg.data.split(',')
            if float(color) > 0.0:
                self._color[x1:x2+1] = float(color)
                self.get_logger().info('blob inserted: %s,%s,%s' % (color,x1,x2))
                # sense hat
                pixcol = blue if color == 1.0 else red
                self._sense.clear()
                for i in range(int(x1/8),int(x2/8)+1):
                    self._sense.set_pixel(0,7-i,pixcol)
                    #self._sense.set_pixel(1,7-i,pixcol)

        except (SyntaxError) as e:
            self.get_logger().error('Failed to get blob coordinates: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    lidar_reader_node = s2eLidarReaderNode()
    rclpy.spin(lidar_reader_node)
    lidar_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

