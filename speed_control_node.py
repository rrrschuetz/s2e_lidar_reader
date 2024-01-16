import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
from simple_pid import PID
from Adafruit_PCA9685 import PCA9685

class SpeedControlNode(Node):
    reverse_pulse = 204
    neutral_pulse = 307
    forward_pulse = 409

    def __init__(self):
        super().__init__('speed_control')
        self.gpio_pin = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)   
        self.subscriber = self.create_subscription(String, 'set_speed', self.set_speed_callback, 10)

        self.get_logger().info('calibrating ESC')
        self.pwm = PCA9685()
        self.pwm.set_pwm_freq(50)  # Set frequency to 50Hz
        self.pwm.set_pwm(1, 0, self.neutral_pulse)

        self.motor_ctl = 36
        self.max_y = 350
        self.max_impulse_count = 10
        self.impulse_count = 0
        self.desired_speed = 0
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
        if self.impulse_count > self.max_impulse_count:
            y = 0
        else:
            y = self.pid(self.impulse_count)
            y = min(self.max_y,abs(int(self.neutral_pulse+y*self.motor_ctl)))
        self.impulse_count = 0  # Reset the count after each measurement
        self.pwm.set_pwm(1, 0, y)
        self.get_logger().info(f"pwm y value set: {y}")

def main(args=None):
    rclpy.init(args=args)
    speed_control = SpeedControlNode()
    rclpy.spin(speed_control)
    speed_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
