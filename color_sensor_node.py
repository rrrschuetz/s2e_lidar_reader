import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Bool  # Import the Bool message type

class ColorSensorNode(Node):
    def __init__(self):
        super().__init__('color_sensor_node')
        # GPIO pin setup
        self.S0 = 16  # Example GPIO pin number
        self.S1 = 21  # Example GPIO pin number
        self.S2 = 26  # Example GPIO pin number
        self.S3 = 6   # Example GPIO pin number
        self.OUT = 5  # Example GPIO pin number
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.OUT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        #GPIO.setup(self.S0, GPIO.OUT)
        #GPIO.setup(self.S1, GPIO.OUT)
        GPIO.setup(self.S2, GPIO.OUT)
        GPIO.setup(self.S3, GPIO.OUT)
        #GPIO.output(self.S0, GPIO.HIGH)
        #GPIO.output(self.S1, GPIO.LOW)

        # Blue color setup
        GPIO.output(self.S2, GPIO.LOW)
        GPIO.output(self.S3, GPIO.HIGH)

        # Frequency range for dark blue
        self.blue_frequency_range = (13000, 20000)

        # Initialize the publisher
        self.publisher_ = self.create_publisher(Bool, 'color_sensor', 10)
        self.get_logger().info('Color Sensor Node initialized!')

        # Use interrupts for efficient detection
        GPIO.add_event_detect(self.OUT, GPIO.FALLING, callback=self.frequency_measurement_callback, bouncetime=10)

    def frequency_measurement_callback(self, channel):
        frequency = self.measure_frequency()
        self.get_logger().info(f"Detected Blue color - Frequency: {frequency} Hz")
        msg = Bool()
        msg.data = self.blue_frequency_range[0] <= frequency <= self.blue_frequency_range[1]
        self.publisher_.publish(msg)

    def measure_frequency(self):
        start_time = time.time()
        for _ in range(5):  # Reduced number of samples for faster response
            GPIO.wait_for_edge(self.OUT, GPIO.FALLING)
        return 5 / (time.time() - start_time)

def main(args=None):
    rclpy.init(args=args)
    node = ColorSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
