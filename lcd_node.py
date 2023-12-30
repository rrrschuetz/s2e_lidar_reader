import rclpy
from rclpy.node import Node
from RPLCD.i2c import CharLCD

class LCDNode(Node):
    def __init__(self):
        super().__init__('lcd_node')
        self.lcd = CharLCD('PCF8574', 0x27)
        self.subscription = self.create_subscription(
            std_msgs.msg.String,
            'lcd_display',
            self.display_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.display_buffer = []

    def display_callback(self, msg):
        self.display_message(msg.data)

    def display_message(self, message):
        self.display_buffer.append(message)
        # Keep only the last 2 lines for LCD1602
        self.display_buffer = self.display_buffer[-2:]
        self.lcd.clear()
        for line in self.display_buffer:
            self.lcd.write_string(line)
            self.lcd.crlf()

def main(args=None):
    rclpy.init(args=args)
    lcd_node = LCDNode()
    rclpy.spin(lcd_node)
    lcd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
