import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Bool  # Import the Bool message type

class ColorSensorNode(Node):
    def __init__(self):
        super().__init__('color_sensor_node')
        # GPIO pin setup
        # [Same as previous setup]

        # Frequency range definitions
        # [Same as previous setup]

        # Initialize the publisher
        self.publisher_ = self.create_publisher(Bool, 'color_detected', 10)

        self.get_logger().info('Color Sensor Node initialized!')

    def read_color(self):
        # [Same as previous read_color method]

    def measure_frequency(self):
        # [Same as previous measure_frequency method]

    def color_frequency_callback(self, color, frequency):
        self.get_logger().info(f"Detected {color} color in frequency range! Frequency: {frequency} Hz")
        # Publish a True Boolean message
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            color_data = self.read_color()
            self.get_logger().info(f"Color Intensities: {color_data}")
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    color_sensor_node = ColorSensorNode()
    try:
        color_sensor_node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        color_sensor_node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
