import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont

class DisplayNode(Node):
    def __init__(self):
        super().__init__('oled_display_node')

        qos_profile = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE)

        # Subscription for data from main
        self.logger_subscription = self.create_subscription(
            String,
            'main_logger',
            self.logger_callback,
            qos_profile)

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
        self.lines.append('**************')
        self.max_lines = self.height // 15  # Assuming 15 pixels per line of text

        self.get_logger().info(f"Display dimensions: {self.width} x {self.height}")
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)  # Clear the display area

    def logger_callback(self, msg):
        self.get_logger().info(f'Display message received: {msg.data}')
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)  # Clear the display area
        data = msg.data.split(',')
        if data[0] == '*':
            if data[3] == 'G':
                Y1 = 0
                Y2 = self.height/2
            else:
                Y1 = self.height/2
                Y2 = self.height
            self.draw.rectangle((int(int(data[1])*self.width/320), Y1, int(int(data[2])*self.width/320), Y2), outline=1, fill=1)
        else:
            self.lines.append(f'{msg.data}')
            if len(self.lines) > self.max_lines:
                self.lines.pop(0)
            for i, line in enumerate(self.lines):
                self.draw.text((0, i*15), line, font=self.font, fill=255)
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
