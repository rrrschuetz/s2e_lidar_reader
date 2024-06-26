import time, configparser, os, signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import gpiozero
import collections
from simple_pid import PID
from Adafruit_PCA9685 import PCA9685

class SpeedControlNode(Node):
    reverse_pulse = 204
    neutral_pulse = 310
    forward_pulse = 409
    base_speed = 6   # 10
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
    average_min_speed = 25

    def __init__(self):
        super().__init__('speed_control')
        self.subscriber_ = self.create_subscription(String, 'set_speed', self.set_speed_callback, 10)
        self._msg = String()

        GPIO.setmode(GPIO.BCM)
        self.pwm = PCA9685()
        self.pwm.set_pwm_freq(50)  # Set frequency to 50Hz
        self.pwm.set_pwm(1, 0, self.neutral_pulse)

        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.HIGH)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.get_logger().info('ESC calibrated.')

        self.reverse = False
        self.impulse_target = 0
        self.move_to_impulse_mode = False
        self.pid_steering = False
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
        config.read(os.path.expanduser('~/ros2_ws4/config.ini'))
        self.pid_output_max = int(config['Speed']['pid_output_max'])
        self.pid_output_min = int(config['Speed']['pid_output_min'])
        self.PID_Kp = float(config['Speed']['pid_Kp'])
        self.PID_Ki = float(config['Speed']['pid_Ki'])
        self.PID_Kd = float(config['Speed']['pid_Kd'])
        self.break_intensity = int(config['Speed']['break_intensity'])
        self.average_min_speed = int(config['Speed']['average_min_speed'])
        self.move_speed = int(config['Speed']['move_speed'])

        self.get_logger().info(f"PID min / max setting: {self.pid_output_min} / {self.pid_output_max}")
        self.get_logger().info(f"PID Kp / Ki / Kd: {self.PID_Kp} / {self.PID_Ki} / {self.PID_Kd}")
        self.get_logger().info(f"Break intensity: {self.break_intensity}")
        self.get_logger().info(f"Average minimal speed: {self.average_min_speed}")
        self.get_logger().info(f"Move speed: {self.move_speed}")

    def __del__(self):
        self.cleanup()

    def cleanup(self):
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.LOW)
        GPIO.cleanup()
        self.get_logger().info('Cleanup complete.')

    def reset_pid(self):
        self.get_logger().info(f"PID reset.")
        self.pid = PID(self.PID_Kp, self.PID_Ki, self.PID_Kd, setpoint=0, output_limits = (self.pid_output_min,self.pid_output_max)) # 0.2/0.05/0.00
        self.pid.sample_time = 0.1

    def move_to_impulse(self, num):
        self.get_logger().info("move_to_impulse called: %s" % num)
        if num == 0:
            self.get_logger().info("move_to_impulse: No move!")
            return

        self.pid_steering = True
        self.move_to_impulse_mode = True
        self.impulse_target = num
        self.impulse_history.clear()
        self.impulse_count = 0
        self.reset_pid()
        self.pid.setpoint = self.move_speed

        #power = self.impulse_speed_rev_med if impulse_goal < 0 else self.impulse_speed_fwd_med
        #self.y_pwm = self.neutral_pulse + power
        #self.pwm.set_pwm(1, 0, self.y_pwm)

        #while self.impulse_count < abs(impulse_goal):
        #    self.impulse_count += sum(self.impulse_history)
        #    self.impulse_history.clear()
        #    time.sleep(0.1)

        #self.y_pwm = self.neutral_pulse
        #self.pwm.set_pwm(1, 0, self.y_pwm)

        #self.get_logger().info("impulses moved: %s" % self.impulse_count)
        return

    def impulse_callback(self, channel):
        if self.move_to_impulse_mode:
            #self.get_logger().info(f"impulses to go {self.impulse_target}.")
            if self.impulse_target > 0: self.impulse_target -=1
            elif self.impulse_target < 0: self.impulse_target +=1
            else:
                self.get_logger().info("impulses target reached.")
                break_impulse = self.break_intensity if self.reverse else -self.break_intensity
                self.pid_steering = False
                self.move_to_impulse_mode = False
                self.reverse = False

                self.pwm.set_pwm(1, 0, self.neutral_pulse + break_impulse)
                time.sleep(0.1)
                self.pwm.set_pwm(1, 0, self.neutral_pulse)

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
                self.pwm.set_pwm(1, 0, self.neutral_pulse-self.break_intensity)  # brake mode
                time.sleep(0.1)
                self.pwm.set_pwm(1, 0, self.neutral_pulse)
            elif new_speed == "BOOST":
                self.pid_output_min = 4
            elif new_speed =="RESET":
                self.reverse = False
                self.impulse_history.clear()
                self.reset_pid()
            elif new_speed.startswith('F'):
                #self.pid_steering = True #False
                self.get_logger().info(f"Received move forward command {new_speed}")
                self.reverse = False
                self.move_to_impulse(int(new_speed[1:]))
            elif new_speed.startswith('R'):
                #self.pid_steering = True # False
                self.get_logger().info(f"Received move backward command {new_speed}")
                self.reverse = True
                self.move_to_impulse(-int(new_speed[1:]))
            else:
                self.reverse = False
                self.pid_steering = True
                new_speedf = float(new_speed)
                self.pid.setpoint = abs(new_speedf)  # Set PID setpoint to desired speed, including direction.
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
        dir = -1 if self.reverse else 1
        self.y_pwm = self.neutral_pulse + (self.base_speed + int(pid_output))*dir
        self.y_pwm = min(self.max_y, self.y_pwm)  # Ensure PWM is within forward range.
        self.impulse_history.clear()  # Clear history after each measurement
        try:
            #self.get_logger().info(f"'y_pwm {self.y_pwm} pid_output {pid_output}")
            self.pwm.set_pwm(1, 0, self.y_pwm)
        except IOError as e:
            self.get_logger().error("IOError I2C occurred: %s" % str(e))

    def log_timer_callback(self):
        average_speed = len(self.impulse_history_long)/self.rolling_avg_period
        #self.get_logger().warn(f"Speed: {average_speed}, minimum speed: {self.average_min_speed} impulses/sec")
        if self.pid_steering and 0 < average_speed < self.average_min_speed:
            self.pid.setpoint += 1
            self.get_logger().info(f"setpoint increased to: {self.pid.setpoint}")

def signal_handler(signal_received, frame, node):
    # Handle any additional cleanup if necessary
    node.get_logger().info('Signal caught, cleaning up...')
    node.cleanup()
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    speed_control = SpeedControlNode()

    # Setup the signal handler
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, speed_control))

    try:
        rclpy.spin(speed_control)
    finally:
        # If shutdown wasn't caused by SIGINT
        speed_control.cleanup()

    speed_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
