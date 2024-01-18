import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import collections
from simple_pid import PID
from Adafruit_PCA9685 import PCA9685

class SpeedControlNode(Node):
    reverse_pulse = 204
    neutral_pulse = 307
    forward_pulse = 409
    gpio_pin = 22
    relay_pin = 17

    def __init__(self):
        super().__init__('speed_control')
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.subscriber = self.create_subscription(String, 'set_speed', self.set_speed_callback, 10)

        self.get_logger().info('calibrating ESC')
        self.pwm = PCA9685()
        self.pwm.set_pwm_freq(50)  # Set frequency to 50Hz
        self.pwm.set_pwm(1, 0, self.neutral_pulse)
        GPIO.output(self.relay_pin, GPIO.HIGH)

        self.motor_ctl = 0.5
        self.max_y = 330
        self.base_pwm = self.neutral_pulse  # Base PWM value for steady motor speed
        self.max_impulse_count = 10
        self.rolling_avg_size = 50  # Number of measurements for the rolling average
        self.impulse_history = collections.deque(maxlen=self.rolling_avg_size)
        self.desired_speed = 0
        self.pid = PID(1.0, 0.0, 0.00, setpoint=self.desired_speed)
        self.pid.sample_time = 0.01  # Update every 0.01 seconds

        GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING, callback=self.impulse_callback)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def __del__(self):
        GPIO.output(self.relay_pin, GPIO.LOW)
        GPIO.cleanup()

    def set_speed_callback(self, msg):
        try:
            new_speed = float(msg.data)
            if new_speed < 0:
                GPIO.output(self.relay_pin, GPIO.LOW)
                GPIO.cleanup()
            else:
                self.pid.setpoint = new_speed
                self.base_pwm = self.neutral_pulse + new_speed * self.motor_ctl
                self.get_logger().info(f"New desired speed set to: {new_speed}")
        except ValueError:
            self.get_logger().error("Received invalid speed setting")

    def impulse_callback(self, channel):
        self.impulse_history.append(1)

    def timer_callback(self):
        impulse_count = sum(self.impulse_history)
        pid_output = self.pid(impulse_count)
        y_pwm = min(self.max_y, abs(int(self.base_pwm + pid_output * self.motor_ctl)))
        self.get_logger().info(f"impulse count: {impulse_count} - y_pwm value set: {y_pwm}")
        self.impulse_history.clear()  # Reset the history after each measurement
        self.pwm.set_pwm(1, 0, y_pwm)

def main(args=None):
    relay_pin = 17
    rclpy.init(args=args)
    speed_control = SpeedControlNode()
    rclpy.spin(speed_control)
    speed_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
