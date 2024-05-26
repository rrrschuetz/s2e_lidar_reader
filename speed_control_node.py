import time
import configparser
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import collections
import threading
from simple_pid import PID
from Adafruit_PCA9685 import PCA9685

class SpeedControlNode(Node):
    reverse_pulse = 204
    neutral_pulse = 310
    forward_pulse = 409
    base_fwd = 6   # 10
    base_rev = -4
    gpio_pin = 22
    relay_pin = 17
    pid_output_min = 4
    pid_output_max = 8
    PID_Kp = 0.2
    PID_Ki = 0.05
    PID_Kd = 0.00
    impulse_count_max = 20
    rolling_avg_size = 100  # Number of measurements for the rolling average
    rolling_avg_period = 5

    def __init__(self):
        super().__init__('speed_control')
        self.subscriber_ = self.create_subscription(String, 'set_speed', self.set_speed_callback, 10)
        self.publisher_ = self.create_publisher(String, 'collision', 10)
        self._msg = String()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.get_logger().info('calibrating ESC')
        self.pwm = PCA9685()
        self.pwm.set_pwm_freq(50)  # Set frequency to 50Hz
        self.pwm.set_pwm(1, 0, self.neutral_pulse)
        GPIO.output(self.relay_pin, GPIO.HIGH)

        self.pid_steering = False
        self.motor_ctl = 1.2
        self.y_pwm = 0
        self.max_y = 350
        self.min_y = 250

        self.impulse_count = 0
        self.impulse_history = collections.deque(maxlen=self.rolling_avg_size)
        self.impulse_history_long = collections.deque(maxlen=1000)
        self.last_impulse_time = self.get_clock().now()

        self.reset_pid()

        GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING, callback=self.impulse_callback)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.log_timer = self.create_timer(self.rolling_avg_period, self.log_timer_callback)

        config = configparser.ConfigParser()
        config.read('/home/rrrschuetz/ros2_ws4/config.ini')
        self.pid_output_max = int(config['Speed']['pid_output_max'])
        self.pid_output_min = int(config['Speed']['pid_output_min'])
        self.PID_Kp = float(config['Speed']['pid_Kp'])
        self.PID_Ki = float(config['Speed']['pid_Ki'])
        self.PID_Kd = float(config['Speed']['pid_Kd'])
        self.get_logger().info(f"PID min / max setting: {self.pid_output_min} / {self.pid_output_max}")
        self.get_logger().info(f"PID Kp / Ki / Kd: {self.PID_Kp} / {self.PID_Ki} / {self.PID_Kd}")

        self.impulse_speed_fwd = int(config['Speed']['impulse_speed_fwd'])
        self.impulse_speed_rev = int(config['Speed']['impulse_speed_rev'])
        self.get_logger().info(f"Impulse speed fwd / rev setting: {self.impulse_speed_fwd} / {self.impulse_speed_rev}")

    def __del__(self):
        GPIO.output(self.relay_pin, GPIO.LOW)
        GPIO.cleanup()

    def reset_pid(self):
        self.pid = PID(self.PID_Kp, self.PID_Ki, self.PID_Kd, setpoint=0, output_limits = (self.pid_output_min,self.pid_output_max)) # 0.2/0.05/0.00
        self.pid.sample_time = 0.1

    def move_to_impulse(self, impulse_goal):
        self.get_logger().info("move_to_impulse called: %s" % impulse_goal)
        if impulse_goal == 0:
            self.get_logger().info("move_to_impulse: No move!")
            return
        power = self.impulse_speed_rev if impulse_goal < 0 else self.impulse_speed_fwd   # -12/12
        self.impulse_history.clear()
        self.impulse_count = 0

        self.y_pwm = self.neutral_pulse + power
        self.pwm.set_pwm(1, 0, self.y_pwm)

        while self.impulse_count < abs(impulse_goal):
            self.impulse_count += sum(self.impulse_history)
            self.impulse_history.clear()
            time.sleep(0.1)

        self.y_pwm = self.neutral_pulse
        self.pwm.set_pwm(1, 0, self.y_pwm)

        self.get_logger().info("impulses moved: %s" % self.impulse_count)
        return

    def impulse_callback(self, channel):
        self.last_impulse_time = self.get_clock().now()
        self.impulse_history.append(1)
        self.impulse_history_long.append(self.last_impulse_time)
        while self.impulse_history_long:
            duration_seconds = (self.last_impulse_time - self.impulse_history_long[0]).nanoseconds / 1e9
            if duration_seconds > self.rolling_avg_period:
                self.impulse_history_long.popleft()  # Remove old data
            else:
                break

    def set_speed_callback(self, msg):
        try:
            new_speed = msg.data  # Assuming speed is passed as a string.
            if new_speed == "STOP":
                self.pid_steering = False
                self.pwm.set_pwm(1, 0, self.neutral_pulse-2)  # brake mode
            elif new_speed =="RESET":
                self.impulse_history.clear()
                self.reset_pid()
            elif new_speed.startswith('F'):
                self.pid_steering = False
                self.get_logger().info(f"Received move forward command {new_speed}" )
                self.move_to_impulse(int(new_speed[1:]))
            elif new_speed.startswith('R'):
                self.pid_steering = False
                self.get_logger().info(f"Received move backward command {new_speed}")
                self.move_to_impulse(-int(new_speed[1:]))
            else:
                self.pid_steering = True
                new_speedf = float(new_speed)
                self.pid.setpoint = abs(new_speedf)  # Set PID setpoint to desired speed, including direction.
                self.reverse = (new_speedf < 0)
        except ValueError:
            self.get_logger().error("Received invalid speed setting")

    def timer_callback(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_impulse_time).nanoseconds/1e9 >= 1:
            self.impulse_history_long.clear()

        if not self.pid_steering: return

        self.impulse_count = sum(self.impulse_history)
        pid_output = self.pid(self.impulse_count)
        #self.get_logger().info(f"Impulses {self.impulse_count},pid_output {pid_output}")
        self.y_pwm = self.neutral_pulse + self.base_fwd + int(pid_output * self.motor_ctl)
        self.y_pwm = min(self.max_y, self.y_pwm)  # Ensure PWM is within forward range.
        self.impulse_history.clear()  # Clear history after each measurement
        try:
            #self.get_logger().info(f"'y_pwm {self.y_pwm} pid_output {pid_output}")
            self.pwm.set_pwm(1, 0, self.y_pwm)
        except IOError as e:
            self.get_logger().error("IOError I2C occurred: %s" % str(e))

    def log_timer_callback(self):
        self.get_logger().info(f"Speed: {len(self.impulse_history_long)/self.rolling_avg_period} impulses/sec")

def main(args=None):
    rclpy.init(args=args)
    speed_control = SpeedControlNode()
    rclpy.spin(speed_control)
    speed_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
