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
        self.max_y = 350
        self.min_y = 250
        self.reverse = False
        self.reverse_p = False
        self.base_pwm = self.neutral_pulse  # Base PWM value for steady motor speed
        self.rolling_avg_size = 100  # Number of measurements for the rolling average
        self.impulse_history = collections.deque(maxlen=self.rolling_avg_size)
        self.desired_speed = 0
        self.pid = PID(0.4, 0.15, 0.00, setpoint=self.desired_speed)
        self.pid.sample_time = 0.1  # Update every 0.2 seconds
        self.lock = False
        self.brake = False

        GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING, callback=self.impulse_callback)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def __del__(self):
        GPIO.output(self.relay_pin, GPIO.LOW)
        GPIO.cleanup()

    def impulse_callback(self, channel):
        self.impulse_history.append(1)

    def set_speed_callback(self, msg):
        try:
            new_speed = msg.data  # Assuming speed is passed as a string.
            if new_speed == "STOP":
                self.pwm.set_pwm(1, 0, self.neutral_pulse)  # Set motor to neutral.
                GPIO.output(self.relay_pin, GPIO.LOW)
            else:
                new_speedf = float(new_speed)
                self.pid.setpoint = abs(new_speedf)  # Set PID setpoint to desired speed, including direction.
                self.reverse = (new_speedf < 0)
        except ValueError:
            self.get_logger().error("Received invalid speed setting")

    def timer_callback(self):
        if self.lock: return
        self.lock = True
        impulse_count = sum(self.impulse_history)

        if self.brake or self.reverse != self.reverse_p:
            self.get_logger().info('brake active ... ')
            self.reverse_p = self.reverse
            if impulse_count > 0: 
                self.brake = True
                #y_pwm = self.min_y if self.reverse else self.max_y
                y_pwm = self.neutral_pulse
            else: 
                self.brake = False
                y_pwm = self.neutral_pulse
                self.pid.setpoint = 0
                self.pid(0)
            #wait_time = 1.0
        else:
            pid_output = self.pid(impulse_count)
            #self.get_logger().info('impulses %s power: %s %s ' % (impulse_count,pid_output,self.reverse))
            # Determine PWM adjustment based on PID output and desired direction.
            if self.reverse:
                # If desired speed is negative, adjust for reverse.
                y_pwm = self.neutral_pulse - abs(int(pid_output * self.motor_ctl))
                y_pwm = max(self.min_y, y_pwm)  # Ensure PWM is within reverse range.
            else:
                # If desired speed is positive or zero, adjust for forward.
                y_pwm = self.neutral_pulse + int(pid_output * self.motor_ctl)
                y_pwm = min(self.max_y, y_pwm)  # Ensure PWM is within forward range.
            wait_time = 0.0
            
        self.impulse_history.clear()  # Clear history after each measurement

        try:
            #self.get_logger().info('y_pwm %s ' % y_pwm)
            self.pwm.set_pwm(1, 0, y_pwm)
            time.sleep(wait_time)            
        except IOError as e:
            self.get_logger().error('IOError I2C occurred: %s' % str(e))
        
        self.lock = False

def main(args=None):
    relay_pin = 17
    rclpy.init(args=args)
    speed_control = SpeedControlNode()
    rclpy.spin(speed_control)
    speed_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
