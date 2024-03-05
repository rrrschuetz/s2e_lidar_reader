import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

class LineDetectorNode(Node):
    def __init__(self):
        super().__init__('line_detector_node')
        self.publisher = self.create_publisher(Bool, 'line_detector', 10)
        self.sensor_pin = 14  # Change as per your GPIO connection
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.sensor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Set up event detection for the sensor pin
        GPIO.add_event_detect(self.sensor_pin, GPIO.FALLING, callback=self.gpio_callback)
        self.get_logger().info('line detector enabled')

    def gpio_callback(self, channel):
        msg = Bool()
        msg.data = (GPIO.input(self.sensor_pin) == 0)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    line_detector_node = LineDetectorNode()

    try:
        rclpy.spin(line_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        line_detector_node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
