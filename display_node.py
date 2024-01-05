import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont

class DisplayNode(Node):
    def __init__(self):
        super().__init__('oled_display_node')

        # Subscription for lidar_data from s2elidar_node
        self.logger_subscription = self.create_subscription(
            String,
            'main_logger',
            self.lidar_callback,
            10)
        self.logger_subscription  # prevent unused variable warning

        # Initialize the display
        self.display = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_address=0x3C)
        self.display.begin()
        self.display.clear()
        self.display.display()

        # Create blank image for drawing
        self.width = self.display.width
        self.height = self.display.height
        self.image = Image.new('1', (self.width, self.height))
        self.draw = ImageDraw.Draw(self.image)

        # Load default font
        self.font = ImageFont.load_default()

    def logger_callback(self, msg):
        message = f'MAIN: {msg.data}'
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
