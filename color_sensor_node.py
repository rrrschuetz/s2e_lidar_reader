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
        self.S3 = 6  # Example GPIO pin number
        self.OUT = 5 # Example GPIO pin number
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.OUT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.S0, GPIO.OUT)
        GPIO.setup(self.S1, GPIO.OUT)
        GPIO.output(self.S0, GPIO.HIGH)
        GPIO.output(self.S1, GPIO.LOW)

        # Frequency range definitions
        self.frequency_ranges = {
            'Red': (300, 400),        
            'Green': (200, 300),
            'Blue': (400, 500)
        }

        # Initialize the publisher
        self.publisher_ = self.create_publisher(Bool, 'color_detected', 10)
        self.get_logger().info('Color Sensor Node initialized!')

    def read_color(self):
        color_readings = {'Red': 0, 'Green': 0, 'Blue': 0}
        for color, pins in {'Red': (GPIO.LOW, GPIO.LOW), 'Blue': (GPIO.LOW, GPIO.HIGH), 'Green': (GPIO.HIGH, GPIO.HIGH)}.items():
            GPIO.output(self.S2, pins[0])
            GPIO.output(self.S3, pins[1])
            frequency = self.measure_frequency()
            color_readings[color] = frequency

            # Check if the frequency is in the predefined range
            if self.frequency_ranges[color][0] <= frequency <= self.frequency_ranges[color][1]:
                self.color_frequency_callback(color, frequency)

        return color_readings

    def measure_frequency(self):
        GPIO.wait_for_edge(self.OUT, GPIO.FALLING)
        start_time = time.time()
        for _ in range(10):
            GPI.wait_for_edge(self.OUT, GPIO.FALLING)
        return 10 / (time.time() - start_time)

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
