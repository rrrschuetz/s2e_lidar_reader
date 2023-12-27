import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class SpeedMonitorNode(Node):
    def __init__(self):
        super().__init__('speed_monitor')
        # Set up GPIO pin
        self.gpio_pin = 17  # Example GPIO pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # Set up a ROS2 publisher
        self.publisher = self.create_publisher(String, 'speed', 10)

        # Initialize variables for speed calculation
        self.count = 0
        self.last_time = None
        self.distance_per_rotation = 0.15  # Set the distance covered per rotation (e.g., circumference of a wheel)

        # Set up a GPIO event detect
        GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING, callback=self.pin_callback)

    def pin_callback(self, channel):
        current_time = time.time()
        if self.last_time is not None:
            time_interval = current_time - self.last_time
            speed = self.calculate_speed(time_interval)
            self.publish_speed(speed)
        self.last_time = current_time

        # Increment count on every falling edge (HIGH to LOW transition)
        self.count += 1
        speed = self.calculate_speed()
        self.publish_speed(speed)

    def calculate_speed(self, time_interval):
        # Speed = Distance / Time
        if time_interval > 0:
            return self.distance_per_rotation / time_interval
        else:
            return 0

    def publish_speed(self, speed):
        msg = String()
        msg.data = speed
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    speed_monitor = SpeedMonitorNode()
    rclpy.spin(speed_monitor)
    speed_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
