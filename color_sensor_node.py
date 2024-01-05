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
        GPIO.setup(self.S0, GPIO.OUT)
        GPIO.setup(self.S1, GPIO.OUT)
        GPIO.setup(self.S2, GPIO.OUT)
        GPIO.setup(self.S3, GPIO.OUT)
        GPIO.output(self.S0, GPIO.HIGH)
        GPIO.output(self.S1, GPIO.LOW)

        # Frequency range definitions
        self.frequency_ranges = {
            'Red':   (12000, 15500),
            'Green': (10500, 14000),
            'Blue':  (15000, 19000)
        }

        # Initialize the publisher
        self.publisher_ = self.create_publisher(Bool, 'color_sensor', 10)
        self.get_logger().info('Color Sensor Node initialized!')

        self.timer = self.create_timer(0.5, self.timer_callback)  # Adjust the timer callback rate as needed

    def timer_callback(self):
        color_readings = {'Red': 0, 'Green': 0, 'Blue': 0}
        msg = Bool()
        msg.data = True
        for color, pins in {'Red': (GPIO.LOW, GPIO.LOW), 'Blue': (GPIO.LOW, GPIO.HIGH), 'Green': (GPIO.HIGH, GPIO.HIGH)}.items():
            GPIO.output(self.S2, pins[0])
            GPIO.output(self.S3, pins[1])
            frequency = self.measure_frequency()
            color_readings[color] = frequency
            #self.get_logger().info(f"Detected {color} color - Frequency: {frequency} Hz")
            # Check if the frequency is in the predefined range
            msg.data = msg.data and ( self.frequency_ranges[color][0] <= frequency <= self.frequency_ranges[color][1])
        if msg.data:
            self.get_logger().info('Line detected!')
            self.publisher_.publish(msg)
        return color_readings

    def measure_frequency(self):
        GPIO.wait_for_edge(self.OUT, GPIO.FALLING)
        start_time = time.time()
        for _ in range(10):
            GPIO.wait_for_edge(self.OUT, GPIO.FALLING)
        return 10 / (time.time() - start_time)

def main(args=None):
    rclpy.init(args=args)
    node = ColorSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
