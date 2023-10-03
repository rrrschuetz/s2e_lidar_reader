import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, time

class openmvH7Node(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'openmv_topic', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust the timer callback rate as needed
        self.get_logger().info('OpenMV H7 connected' )
    def timer_callback(self):
        try:
            msg = String()
            if self.serial_port.in_waiting:
                msg.data = self.serial_port.read(self.serial_port.in_waiting).decode()
                #msg.data = self.serial_port.readline().decode('utf-8').strip()  # Read a line and strip it
                self.publisher_.publish(msg)
                self.get_logger().info('blob published %s' % msg.data )
        except serial.SerialException as e:
            self.get_logger().error(f"Serial Exception: {e}")
        except OSError as e:
            self.get_logger().error(f"OS Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = openmvH7Node()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

