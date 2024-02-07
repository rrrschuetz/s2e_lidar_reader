import time
import rclpy
from rclpy.node import Node
import board
import busio
import adafruit_vl53l1x
import RPi.GPIO as GPIO
from std_msgs.msg import Float32

class DistanceSensorNode(Node):  # Corrected class name
    def __init__(self):
        super().__init__('distance_sensor_node')
        self.publisher = self.create_publisher(Float32, 'distance_sensor', 10)  # Corrected message type

        self.sensor_pin = 17  # Change as per your GPIO connection
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.sensor_pin, GPIO.OUT, initial=GPIO.HIGH)

        # Initialize I2C bus and sensor.
        i2c = busio.I2C(board.SCL, board.SDA)
        self.tof = adafruit_vl53l1x.VL53L1X(i2c)

        self.timer = self.create_timer(0.1, self.timer_callback)  # Check every 0.1 second

    #def __del__(self):
        GPIO.cleanup()

    def timer_callback(self):  # Removed unused parameter
        msg = Float32()
        msg.data = self.tof.distance
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: Distance in mm "%f"' % msg.data)  # Corrected logging format

def main(args=None):
    rclpy.init(args=args)
    distance_sensor_node = DistanceSensorNode()

    try:
        rclpy.spin(distance_sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        distance_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
