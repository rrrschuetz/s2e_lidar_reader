import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, time
import configparser

class openmvH7Node(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'openmv_topic1', 10)
        self.publisher_log_ = self.create_publisher(String, 'main_logger', 10)
        self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=5)   #115200
        self.get_logger().info('OpenMV H7 1 connected' )

        config = configparser.ConfigParser()
        config.read('/home/rrrschuetz/ros2_ws4/config.ini')
        db_gain = float(config['Camera']['db_gain'])
        gamma_corr = float(config['Camera']['gamma_corr'])
        self.get_logger().info(f"Settings: db_gain {db_gain}, gamma_corr {gamma_corr}")

        with open("/home/rrrschuetz/ros2_ws4/src/s2e_lidar_reader/s2e_lidar_reader/h7_cam_exec.py", 'rb') as file:
            script_data = file.read()
            header_data = f"{db_gain}\n{gamma_corr}\n{len(script_data)}\n".encode('utf-8')
            self.serial_port.write(header_data + script_data)
            self.get_logger().info(f"OpenMV H7 1 script sent: {header_data}" )
        #time.sleep(10)
        #self.serial_port.reset_input_buffer()
        #self.serial_port.reset_output_buffer()
        self.timer = self.create_timer(0.01, self.timer_callback)  # Adjust the timer callback rate as needed
        self._counter = 0

        #msg = String()
        #msg.data = "CAM1 online"
        #self.publisher_log_.publish(msg)

    def timer_callback(self):
        try:
            msg = String()
            if self.serial_port.in_waiting:
                msg.data = self.serial_port.readline().decode().strip()
                self.publisher_.publish(msg)
            else:
                self.get_logger().info("No cam data available!")

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

