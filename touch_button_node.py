import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

class TouchButtonNode(Node):
    def __init__(self):
        super().__init__('touch_button')
        self.gpio_pin = 5
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.publisher = self.create_publisher(Bool, 'touch_button', 10)
        GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING, callback=self.gpio_callback)

    def gpio_callback(self, channel):
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    touch_button = TouchButtonNode()
    rclpy.spin(touch_button)
    touch_button.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
