import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

class LineDetectorNode(Node):
    def __init__(self):
        super().__init__('ir_sensor_node')
        self.publisher = self.create_publisher(Bool, 'line_detector', 10)
        self.sensor_pin = 22  # Change as per your GPIO connection
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.sensor_pin, GPIO.IN)

        # Timer to read the sensor value every second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        obstacle_detected = not GPIO.input(self.sensor_pin)
        msg = Bool()
        msg.data = obstacle_detected
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    line_detector_node = LineDetectorNode()

    try:
        rclpy.spin(ir_sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        line_detector_node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
