import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class SpeedMonitorNode(Node):
    def __init__(self):
        super().__init__('speed_monitor')
        self.gpio_pin = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.publisher = self.create_publisher(String, 'speed_monitor', 10)
        self._count = 0
        self._speed = 0
        self._last_time = None
        self._last_movement_time = time.time()
        self._distance_per_rotation = 0.01
        self._stop_timeout = 0.1  # seconds to consider as stopped

        GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING, callback=self.pin_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Check every second

    def pin_callback(self, channel):
        self._last_movement_time = time.time()
        if self._last_time is None:
            self._last_time = self._last_movement_time
        else:
            self._count += 1
            if self._count > 3:
                self._count = 0
                current_time = self._last_movement_time
                self._speed = self.calculate_speed(current_time - self._last_time)
                self._last_time = current_time
                self.publish_speed(self._speed)

    def timer_callback(self):
        if time.time() - self._last_movement_time > self._stop_timeout:
            self._speed = 0
            self.publish_speed(self._speed)

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
