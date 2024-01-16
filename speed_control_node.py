import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
from simple_pid import PID

class SpeedControlNode(Node):
    def __init__(self):
        super().__init__('speed_monitor')
        self.gpio_pin = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.publisher = self.create_publisher(String, 'motor_power', 10)
        self.subscriber = self.create_subscription(String, 'set_speed', self.set_speed_callback, 10)

        self.impulse_count = 0
        self.desired_speed = 5  # Default desired speed
        self.pid = PID(1.0, 0.1, 0.05, setpoint=self.desired_speed)
        self.pid.sample_time = 0.01  # Update every 0.01 seconds

        GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING, callback=self.impulse_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def set_speed_callback(self, msg):
        try:
            new_speed = float(msg.data)
            self.pid.setpoint = new_speed
            self.get_logger().info(f"New desired speed set to: {new_speed}")
        except ValueError:
            self.get_logger().error("Received invalid speed setting")

    def impulse_callback(self, channel):
        self.impulse_count += 1

    def timer_callback(self):
        motor_power = self.pid(self.impulse_count)
        self.impulse_count = 0  # Reset the count after each measurement
        self.publish_motor_power(motor_power)

    def publish_motor_power(self, power):
        msg = String()
        msg.data = str(power)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    speed_control = SpeedControlNode()
    rclpy.spin(speed_control)
    speed_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
