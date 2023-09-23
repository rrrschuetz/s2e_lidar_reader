import os
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
        self._color = np.zeros(self.HPIX)
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

    num_scan = 3240
    num_colr = HPIX  # Assuming HPIX is defined elsewhere in your code
    
    scan_labels = [f'SCAN.{i}' for i in range(1, num_scan+1)]
    colr_labels = [f'COLR.{i}' for i in range(1, num_colr+1)]

    labels = ['X', 'Y'] + scan_labels + colr_labels + ['MAGX', 'MAGY', 'MAGZ', 'ACCX', 'ACCY', 'ACCZ', 'GYRX', 'GYRY', 'GYRZ']
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
        scan_data += ','.join(str(e) for e in self._color)
        
        # add magentometer data
        mag = self._sense.get_compass_raw()
        scan_data += ','+str(mag['x'])+','+str(mag['y'])+','+str(mag['z'])
        # add accelerometer data
        accel = self._sense.get_accelerometer_raw()
        #scan_data += ','+str(accel['x'])+','+str(accel['y'])+','+str(accel['z'])
        scan_data += ',0.0,0.0,0.0'
        # add gyroscope data
        gyro = self._sense.get_gyroscope_raw()
        scan_data += ','+str(gyro['x'])+','+str(gyro['y'])+','+str(gyro['z'])

        # Write the scan data to a file
        with open('/home/rrrschuetz/test/file.txt', 'a') as f:
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
        self._pwm.set_pwm(1, 0, int(self.neutral_pulse+self._Y*40))

    def openmv_h7_callback(self, msg):
        self.get_logger().info('cam msg received: "%s"' % msg)
        self._color = np.zeros(self.HPIX)
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
                self._color[x1:x2+1] = fcol
                self.get_logger().info('blob inserted: %s,%s,%s' % (color,x1,x2))

def main(args=None):
    rclpy.init(args=args)
    lidar_reader_node = s2eLidarReaderNode()
    rclpy.spin(lidar_reader_node)
    lidar_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

