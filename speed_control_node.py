import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import collections
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
    impulse_count_max = 20

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
        self.reverse = False
        self.reverse_p = False
        self.impulse_count = 0
        self.impulse_count_p = 0
        self.pid_output_max = 15
        self.rolling_avg_size = 100  # Number of measurements for the rolling average
        self.impulse_history = collections.deque(maxlen=self.rolling_avg_size)
        self.reset_pid()

        GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING, callback=self.impulse_callback)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def __del__(self):
        GPIO.output(self.relay_pin, GPIO.LOW)
        GPIO.cleanup()

    def reset_pid(self):
        self.pid = PID(0.2, 0.20, 0.05, setpoint=0)   #0.2,0.05,0.00
        self.pid.sample_time = 0.1

    def move_to_impulse(self, impulse_goal):
        power = -12 if impulse_goal < 0 else 12   # -10
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
        self.impulse_history.append(1)

    def set_speed_callback(self, msg):
        try:
            new_speed = msg.data  # Assuming speed is passed as a string.
            if new_speed == "STOP":
                self.pid_steering = False
                self.pwm.set_pwm(1, 0, self.neutral_pulse)  # Set motor to neutral.
                GPIO.output(self.relay_pin, GPIO.LOW)
            elif new_speed =="RESET":
                self.impulse_history.clear()
                self.reset_pid()
            elif new_speed.startswith('F'):
                self.pid_steering = False
                self.get_logger().info("Received move forward command")
                self.move_to_impulse(int(new_speed[1:]))
            elif new_speed.startswith('R'):
                self.pid_steering = False
                self.get_logger().info("Received move backward command")
                self.move_to_impulse(-int(new_speed[1:]))
            else:
                self.pid_steering = True
                new_speedf = float(new_speed)
                self.pid.setpoint = abs(new_speedf)  # Set PID setpoint to desired speed, including direction.
                self.reverse = (new_speedf < 0)
        except ValueError:
            self.get_logger().error("Received invalid speed setting")

    def timer_callback(self):
        if not self.pid_steering: return
        pid_output = 0
        self.impulse_count_p = self.impulse_count
        self.impulse_count = sum(self.impulse_history)

        if self.impulse_count > self.impulse_count_max:
            self.pid_steering = False
            self.pwm.set_pwm(1, 0, self.neutral_pulse)  # Set motor to neutral.
            GPIO.output(self.relay_pin, GPIO.LOW)
            return

        if self.reverse != self.reverse_p:
            self.reverse_p = self.reverse
            self.reset_pid()
            # break mode
            # self.y_pwm = self.neutral_pulse
            # self.y_pwm = self.min_y if self.reverse else self.max_y
        else:
            pid_output = self.pid(self.impulse_count)
            #self.get_logger().info('impulses %s power: %s %s' % (self.impulse_count,pid_output,self.reverse))
            # Determine PWM adjustment based on PID output and desired direction.
            if self.reverse:
                # If desired speed is negative, adjust for reverse.
                self.y_pwm = self.neutral_pulse + self.base_rev - abs(int(pid_output * self.motor_ctl))
                self.y_pwm = max(self.min_y, self.y_pwm)  # Ensure PWM is within reverse range.
            else:
                # If desired speed is positive or zero, adjust for forward.
                self.y_pwm = self.neutral_pulse + self.base_fwd + int(pid_output * self.motor_ctl)
                self.y_pwm = min(self.max_y, self.y_pwm)  # Ensure PWM is within forward range.
            
        self.impulse_history.clear()  # Clear history after each measurement

        if abs(pid_output) > self.pid_output_max and self.impulse_count_p > 0 and self.impulse_count == 0:
            self.get_logger().error("Track blocked: %s" % pid_output)
            self.reset_pid()
            self.y_pwm = self.neutral_pulse
            self._msg.data = "COLLISION"
            self.publisher_.publish(self._msg)

        try:
            #self.get_logger().info('y_pwm %s ' % y_pwm)
            self.pwm.set_pwm(1, 0, self.y_pwm)
        except IOError as e:
            self.get_logger().error("IOError I2C occurred: %s" % str(e))

        
def main(args=None):
    rclpy.init(args=args)
    speed_control = SpeedControlNode()
    rclpy.spin(speed_control)
    speed_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
