import time
import rclpy
from rclpy.node import Node
import VL53L1X
from std_msgs.msg import Float32

class DistanceSensorNode(Node):  # Corrected class name
    def __init__(self):
        super().__init__('distance_sensor_node')
        self.publisher = self.create_publisher(Float32, 'distance_sensor', 10)  # Corrected message type

        # Create a VL53L1X object
        self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)  # Default I2C address is 0x29
        # Open and start ranging
        self.tof.open()
        #self.tof.start_ranging(VL53L1X.VL53L1X_LONG_RANGE_MODE)  # Choose ranging mode
        self.tof.start_ranging(1)  # Choose ranging mode
        self.timer = self.create_timer(0.1, self.timer_callback)  # Check every 0.1 second

    def __del__(self):
        self.tof.stop_ranging()  # Stop ranging
        self.tof.close()

    def timer_callback(self):  # Removed unused parameter
        msg = Float32()
        msg.data = self.tof.get_distance()  # Corrected to use self to access tof
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: Distance in mm "%f"' % msg.data)  # Corrected logging format

def main(args=None):
    rclpy.init(args=args)
    distance_sensor_node = DistanceSensorNode()

    try:
        rclpy.spin(distance_sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        distance_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
