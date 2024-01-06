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

       # Continuous monitoring setup
        self.start_time = time.time()
        self.pulse_count = 0
        GPIO.add_event_detect(self.OUT, GPIO.FALLING, callback=self.count_pulse)

    def count_pulse(self, channel):
        self.pulse_count += 1

    def monitor_frequency(self):
        self.get_logger().info('monitor_frequency() called')
        current_time = time.time()
        if current_time - self.start_time >= 1:  # One-second interval
            frequency = self.pulse_count / (current_time - self.start_time)
            self.get_logger().info(f"Detected Frequency: {frequency} Hz")
            is_blue = self.blue_frequency_range[0] <= frequency <= self.blue_frequency_range[1]
            msg = Bool()
            msg.data = is_blue
            self.publisher_.publish(msg)
            # Reset the counter and timer
            self.pulse_count = 0
            self.start_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = ColorSensorNode()
    rclpy.spin(speed_monitor)
    speed_monitor.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
