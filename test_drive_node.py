import rclpy
import time
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from .Motor import PwmMotor 
import numpy as np
import tensorflow as tf

class testDriveNode(Node):
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
        self._motor = PwmMotor()
        self._motor.setMotorModel(0,0,0,0)

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
        self._model = tf.keras.models.load_model('/home/rrrschuetz/test/model')


    def lidar_callback(self, msg):
        
        if not self._processing:
            self._processing = True
            start_time = time.time()
            self._counter += 1
            
            if self._counter % 1 == 0:
                scan = np.array(msg.ranges)
                #scan[scan == np.inf] = 0.0
                scan[scan == np.inf] = np.nan
                x = np.arange(len(scan))
                finite_vals = np.isfinite(scan)
                scan_interpolated = np.interp(x,x[finite_vals],scan[finite_vals])
                combined = np.concatenate((scan_interpolated, self._color), axis=1)
                combined = np.reshape(combined, (1, -1))
                predictions = self._model.predict(combined)
                #scan = np.reshape(scan, (1, -1))
                #predictions = self._model.predict(scan)
                #self.get_logger().info('Predicted axes: "%s"' % predictions)
                self._X = predictions[0,0]
                self._Y = predictions[0,1]
                self._motor.setMotorModel(
                    int(700*self._Y*(1+1.7*self._X)),
                    int(700*self._Y*(1+1.7*self._X)),
                    int(700*self._Y*(1-1.7*self._X)),
                    int(700*self._Y*(1-1.7*self._X)))
                
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
        self._motor.setMotorModel(
            int(1000*self._Y*(1+self._X)),
            int(1000*self._Y*(1+self._X)),
            int(1000*self._Y*(1-self._X)),
            int(1000*self._Y*(1-self._X)))


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

