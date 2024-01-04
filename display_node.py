import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont

class DisplayNode(Node):
    def __init__(self):
        super().__init__('oled_display_node')

        # Subscription for color_detected
        self.color_subscription = self.create_subscription(
            Bool,
            'color_detected',
            self.color_callback,
            10)
        self.color_subscription  # prevent unused variable warning

        # Subscription for lidar_data from s2elidar_node
        self.lidar_subscription = self.create_subscription(
            String,
            'lidar_data',
            self.lidar_callback,
            10)
        self.lidar_subscription  # prevent unused variable warning

        # OLED display initialization
        # [Same as previous setup]

    def color_callback(self, msg):
        message = 'Color Detected' if msg.data else 'No Color'
        self.update_display(message)

    def lidar_callback(self, msg):
        message = f'Lidar: {msg.data}'
        self.update_display(message)

    def update_display(self, message):
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        self.draw.text((0, 0), message, font=self.font, fill=255)
        self.display.image(self.image)
        self.display.display()

def main(args=None):
    rclpy.init(args=args)
    display_node = DisplayNode()
    rclpy.spin(display_node)
    display_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
