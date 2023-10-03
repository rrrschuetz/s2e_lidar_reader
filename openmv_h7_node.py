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
        self._counter = 0
    def timer_callback(self):
        try:
            msg = String()
            if self.serial_port.in_waiting:
                header = self.serial_port.readline().decode().strip()
                _, str_len, _, jpg_len = map(int, header.split(','))
                msg.data = self.serial_port.read(str_len).decode()
                self.get_logger().info('blob published %s' % msg.data )
                with open("/home/rrrschuetz/test/saved_images/image_{}.jpg".format(counter),'wb') as f:
                    f.write(self.serial_read.read(jpg_len))
                    self._counter += 1
                    if self._counter > 9999: self._counter = 0
                self.publisher_.publish(msg)
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

