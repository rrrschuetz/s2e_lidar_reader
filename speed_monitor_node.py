import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class SpeedMonitorNode(Node):
    def __init__(self):
        super().__init__('speed_monitor')
        self.gpio_pin = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.publisher = self.create_publisher(String, 'speed_monitor', 10)
        self._count = 0
        self._speed = 0
        self._last_time = None
        self._last_movement_time = time.time()
        self._distance_per_rotation = 0.05
        self._stop_timeout = 0.1  # seconds to consider as stopped
        self.speeds = []  # List to store recent speeds
        self.rolling_average_size = 5  # Number of measurements to consider for rolling average

        GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING, callback=self.pin_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Check every 0.1 second

    def pin_callback(self, channel):
        self._last_movement_time = time.time()
        if self._last_time is None:
            self._last_time = self._last_movement_time
        else:
            self._count += 1
            if self._count > 3:
                self._count = 0
                current_time = self._last_movement_time
                speed = self.calculate_speed(current_time - self._last_time)
                if speed != 0:  # Only add non-zero speeds to the list
                    self.add_to_rolling_average(speed)
                    self._last_time = current_time
                    self.publish_speed(self.calculate_rolling_average())
    def add_to_rolling_average(self, speed):
        self.speeds.append(speed)
        if len(self.speeds) > self.rolling_average_size:
            self.speeds.pop(0)  # Remove the oldest speed measurement
    def calculate_rolling_average(self):
        if len(self.speeds) == 0:
            return 0
        return sum(self.speeds) / len(self.speeds)
    def timer_callback(self):
        if time.time() - self._last_movement_time > self._stop_timeout:
            self._speed = 0
            self.add_to_rolling_average(self._speed)
            self.publish_speed(self.calculate_rolling_average())
    def calculate_speed(self, time_interval):
        if time_interval > 0:
            return self._distance_per_rotation / time_interval
        else:
            return 0
    def publish_speed(self, speed):
        msg = String()
        msg.data = str(speed)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    speed_monitor = SpeedMonitorNode()
    rclpy.spin(speed_monitor)
    speed_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
