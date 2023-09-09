import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from sense_hat import SenseHat
import numpy as np

#from .Motor import PwmMotor
from Adafruit_PCA9685 import PCA9685

class s2eLidarReaderNode(Node):

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

        self._color = np.zeros(3240)
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

        #self._motor = PwmMotor()
        #self._motor.setMotorModel(0,0,0,0)

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
        f.write('X,Y,' + ','.join(['SCAN']*3240) + ','+','.join(['COLR']*3240) + 
            '\n')
            #',MAGX,MAGY,MAGZ,ACCX,ACCY,ACCZ,GYRX,GYRY,GYRZ\n')

    def lidar_callback(self, msg):

        # Convert the laser scan data to a string
        #scan_data = "axes: {}\n".format(self._joy_msg.axes)

        #scan_data += "angle_min: {}\n".format(msg.angle_min)
        #scan_data += "angle_max: {}\n".format(msg.angle_max)
        #scan_data += "angle_increment: {}\n".format(msg.angle_increment)
        #scan_data += "time_increment: {}\n".format(msg.time_increment)
        #scan_data += "scan_time: {}\n".format(msg.scan_time)
        #scan_data += "range_min: {}\n".format(msg.range_min)
        #scan_data += "range_max: {}\n".format(msg.range_max)
        #scan_data += "ranges: {}\n".format(msg.ranges)
        #scan_data += "intensities: {}\n".format(msg.intensities)

        scan = np.array(msg.ranges)
        #scan[scan == np.inf] = 0.0 
        scan[scan == np.inf] = np.nan
        x = np.arange(len(scan))
        finite_vals = np.isfinite(scan)
        scan_interpolated = np.interp(x,x[finite_vals],scan[finite_vals])

        # Convert the laser scan data to a string
        scan_data = str(self._X)+','+str(self._Y)+','
        scan_data += ','.join(str(e) for e in scan_interpolated)+','
        #scan_data += ','.join(str(e) for e in scan)

        # add color data
        scan_data += ','.join(str(e) for e in self._color)
        
        # add magentometer data
        #mag = self._sense.get_compass_raw()
        #scan_data += ','+str({mag['x']})
        #scan_data += ','+str({mag['y']})
        #scan_data += ','+str({mag['z']})
        
        # add accelerometer data
        #accel = self._sense.get_accelerometer_raw()
        #scan_data += ','+str({accel['x']})
        #scan_data += ','+str({accel['y']})
        #scan_data += ','+str({accel['z']})

        # add gyroscope data
        #gyro = self._sense.get_gyroscope_raw()
        #scan_data += ','+str({gyro['x']})
        #scan_data += ','+str({gyro['y']})
        #scan_data += ','+str({gyro['z']})
        
        # Write the scan data to a file
        with open('/home/rrrschuetz/test/file.txt', 'a') as f:
            f.write(scan_data + '\n')


    def joy_callback(self, msg):
        #self.get_logger().info('Buttons: "%s"' % msg.buttons)
        #self.get_logger().info('Axes: "%s"' % msg.axes)
        self._X = msg.axes[2]
        self._Y = msg.axes[1]

        #self._motor.setMotorModel(
        #    int(700*self._Y*(1+self._X)),
        #    int(700*self._Y*(1+self._X)),
        #    int(700*self._Y*(1-self._X)),
        #    int(700*self._Y*(1-self._X)))

        #self.get_logger().info('Steering: "%s"' % str(self.servo_neutral+self._X*self.servo_ctl))
        #self.get_logger().info('Power: "%s"' % str(self.neutral_pulse+self._Y*40))
        self._pwm.set_pwm(0, 0, int(self.servo_neutral+self._X*self.servo_ctl))
        self._pwm.set_pwm(1, 0, int(self.neutral_pulse+self._Y*40))

    def openmv_h7_callback(self, msg):

        HPIX2 = 160
        VPIX2 = 120
        HFOV = 70.8
        VFOV = 55.6

        blue = (0,0,255)
        red = (255,0,0)

        self._color = np.zeros(3240)
        self.get_logger().info('blob detected: %s' % msg.data)
        try:
            color, x1, x2 = msg.data.split(',')
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
    lidar_reader_node = s2eLidarReaderNode()
    rclpy.spin(lidar_reader_node)
    lidar_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

