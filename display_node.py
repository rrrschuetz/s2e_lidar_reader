import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont

class DisplayNode(Node):
    def __init__(self):
        super().__init__('oled_display_node')

        # Subscription for data from main
        self.logger_subscription = self.create_subscription(
            String,
            'main_logger',
            self.logger_callback,
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

        # Initialize a list to store lines of text
        self.lines = []
        self.max_lines = self.height // 12  # Assuming 8 pixels per line of text

    def logger_callback(self, msg):
        message = f'INFO: {msg.data}'
        self.lines.append(message)
        # Remove the oldest line if we exceed max_lines
        if len(self.lines) > self.max_lines:
            self.lines.pop(0)
        self.update_display()

    def update_display(self):
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        for i, line in enumerate(self.lines):
            self.draw.text((0, i*12), line, font=self.font, fill=255)
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
