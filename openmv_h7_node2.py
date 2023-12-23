import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, time

class openmvH7Node(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'openmv_topic2', 10)
        self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=5)  # 115200
        self.get_logger().info('OpenMV H7 2 connected' )
        with open("/home/rrrschuetz/ros2_ws4/src/s2e_lidar_reader/s2e_lidar_reader/h7_cam_exec.py", 'rb') as file:
            script_data = file.read()
            self.serial_port.write(script_data)
            self.get_logger().info('OpenMV H7 2 script sent' )
        time.sleep(10)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

        self.timer = self.create_timer(0.05, self.timer_callback)  # Adjust the timer callback rate as needed
        self._counter = 0

    def timer_callback(self):
        try:
            msg = String()
            if self.serial_port.in_waiting:
                header = self.serial_port.readline().decode().strip()

                # Log the received header
                self.get_logger().info(f'header {header}')

                parts = header.split(',')
                if len(parts) >= 4:
                    str_len = int(parts[1])
                    jpg_len = int(parts[3])
                    msg.data = self.serial_port.read(str_len).decode()

                    # Additional code for processing the data
                    with open("/home/rrrschuetz/test/saved_images/image_2_{}.jpg".format(self._counter),'wb') as f:
                        f.write(self.serial_port.read(jpg_len))
                        self._counter += 1
                        if self._counter > 9999: self._counter = 0
                    self.publisher_.publish(msg)

                else:
                    self.get_logger().warning(f"Invalid header format: {header}")

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

if __name__ == '__main__':
    main()

