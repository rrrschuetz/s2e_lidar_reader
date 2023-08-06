import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np

from .Motor import PwmMotor 

class s2eLidarReaderNode(Node):
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
        self._motor = PwmMotor()
        self._motor.setMotorModel(0,0,0,0)

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
            10)


    with open('/home/rrrschuetz/test/file.txt', 'a') as f:
        f.write('X,Y,' + ','.join(['SCAN']*3240) + ','+','.join(['COLR']*3240) + '\n')


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

        scan_data += ','.join(str(e) for e in self._color)

        # Write the scan data to a file
        with open('/home/rrrschuetz/test/file.txt', 'a') as f:
            f.write(scan_data + '\n')


    def joy_callback(self, msg):
        #self.get_logger().info('Buttons: "%s"' % msg.buttons)
        #self.get_logger().info('Axes: "%s"' % msg.axes)
        self._X = msg.axes[2]
        self._Y = msg.axes[1]
        self._motor.setMotorModel(
            int(700*self._Y*(1+self._X)),
            int(700*self._Y*(1+self._X)),
            int(700*self._Y*(1-self._X)),
            int(700*self._Y*(1-self._X)))
        

    def openmv_h7_callback(self, msg):

        HPIX2 = 160
        VPIX2 = 120
        HFOV = 70.8
        VFOV = 55.6

        #self.get_logger().info('blob detected: %s' % msg.data)
        try:
            color, y1, y2 = msg.data.split(',')
            #alphaH=(HPIX2-cxy[0])/HPIX2*HFOV/2*math.pi/180
            alphaV1=(float(y1)-VPIX2)/VPIX2*VFOV/2*math.pi/180
            alphaV2=(float(y2)-VPIX2)/VPIX2*VFOV/2*math.pi/180

            self._color = np.zeros(3240)
            idx1 = int(alphaV1/math.pi*1620)+1620
            idx2 = int(alphaV2/math.pi*1620)+1620
            self._color[idx1:idx2+1] = float(color)
            #self.get_logger().info('blob inserted: %s,%s,%s' % (color,idx1,idx2))

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

