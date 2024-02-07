import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import gpiozero
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory

class DistanceSensorNode(Node):  # Corrected class name
    def __init__(self):
        super().__init__('distance_sensor_node')
        self.publisher = self.create_publisher(Float32, 'distance_sensor', 10)  # Corrected message type
        self.sensor = DistanceSensor(echo=12, trigger=11, pin_factory=PiGPIOFactory())
        self.timer = self.create_timer(0.1, self.timer_callback)  # Check every 0.1 second

    def timer_callback(self):  # Removed unused parameter
        msg = Float32()
        msg.data = self.sensor.distance
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
