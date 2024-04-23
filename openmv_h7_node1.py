import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, time

class openmvH7Node(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'openmv_topic1', 10)
        self.publisher_log_ = self.create_publisher(String, 'main_logger', 10)
        self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=5)   #115200
        self.get_logger().info('OpenMV H7 1 connected' )
        with open("/home/rrrschuetz/ros2_ws4/src/s2e_lidar_reader/s2e_lidar_reader/h7_cam_exec.py", 'rb') as file:
            script_data = file.read()
            self.serial_port.write(script_data)
            self.get_logger().info('OpenMV H7 1 script sent' )
        time.sleep(10)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()
        self.timer = self.create_timer(0.04, self.timer_callback)  # Adjust the timer callback rate as needed
        self._counter = 0

        msg = String()
        msg.data = "CAM1 online"
        self.publisher_log_.publish(msg)

    def timer_callback(self):
        try:
            msg = String()
            if self.serial_port.in_waiting:
                header = self.serial_port.readline().decode().strip()

                # Log the received header
                #self.get_logger().info(f'header {header}')

                parts = header.split(',')
                if len(parts) == 2:   # >=5
                    cam_id = parts[0]
                    str_len = int(parts[1])
                    #jpg_len = int(parts[5])
                    if cam_id   == '240024001951333039373338': msg.data = '1,'   # 33001c000851303436373730
                    elif cam_id == '2d0024001951333039373338': msg.data = '2,'   # 340046000e51303434373339
                    msg.data += self.serial_port.read(str_len).decode()

                    # Additional code for processing the data
                    #with open("/home/rrrschuetz/test/saved_images/image_1_{}.jpg".format(self._counter),'wb') as f:
                    #    f.write(self.serial_port.read(jpg_len))
                    #    self._counter += 1
                    #    if self._counter > 9999: self._counter = 0

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
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

