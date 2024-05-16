import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, time

class openmvH7Node(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'openmv_topic2', 10)
        self.publisher_log_ = self.create_publisher(String, 'main_logger', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=5)  # 115200
        self.get_logger().info('OpenMV H7 2 connected' )
        with open("/home/rrrschuetz/ros2_ws4/src/s2e_lidar_reader/s2e_lidar_reader/h7_cam_exec.py", 'rb') as file:
            script_data = file.read()
            self.serial_port.write(script_data)
            self.get_logger().info('OpenMV H7 2 script sent' )
        #time.sleep(10)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()
        self.timer = self.create_timer(0.02, self.timer_callback)  # Adjust the timer callback rate as needed
        self._counter = 0

        #msg = String()
        #msg.data = "CAM2 online"
        #self.publisher_log_.publish(msg)

    def timer_callback(self):
        try:
            msg = String()
            if self.serial_port.in_waiting:
                msg.data = self.serial_port.readline().decode().strip()
                self.publisher_.publish(msg)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial Exception: {e}")
        except OSError as e:
            self.get_logger().error(f"OS Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected Error: {e}")
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

def main(args=None):
    rclpy.init(args=args)
    node = openmvH7Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

