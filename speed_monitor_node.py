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
        self.publisher = self.create_publisher(String, 'speed_monitor', 10)

        # Initialize variables for speed calculation
        self._count = 0
        self._speed = 0
        self._last_time = None
        self._distance_per_rotation = 0.0033  # 24 marks, 1cm per 3 marks

        # Set up a GPIO event detect
        GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING, callback=self.pin_callback)

    def pin_callback(self, channel):
        if self._last_time is None:
            self._last_time = time.time()
        else:
            self._count += 1
            if self._count > 0:
                self._count = 0
                current_time = time.time()
                self._speed = self.calculate_speed(current_time - self._last_time)
                self._last_time = current_time
        self.publish_speed(self._speed)

    def calculate_speed(self, time_interval):
        # Speed = Distance / Time
        if time_interval > 0:
            return self._distance_per_rotation / time_interval
        else:
            return 0

    def publish_speed(self, speed):
        msg = String()
        msg.data = str(speed)
        self.publisher.publish(msg)
        #self.get_logger().info('speed published: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    speed_monitor = SpeedMonitorNode()
    rclpy.spin(speed_monitor)
    speed_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
