import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, time, threading, configparser

class openmvH7Node(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'openmv_topic2', 10)
        self.publisher_log_ = self.create_publisher(String, 'main_logger', 10)

        config = configparser.ConfigParser()
        config.read('/home/rrrschuetz/ros2_ws4/config.ini')
        self.interval = float(config['Hardware']['camera_timer'])
        self.get_logger().info(f"Camera 2: timer interval: {self.interval}")

        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=5)   #115200
        self.get_logger().info('OpenMV H7 2 connected' )
        with open("/home/rrrschuetz/ros2_ws4/src/s2e_lidar_reader/s2e_lidar_reader/h7_cam_exec.py", 'rb') as file:
            script_data = file.read()
            self.serial_port.write(script_data)
            self.get_logger().info('OpenMV H7 2 script sent' )
        #time.sleep(10)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

        self.latest_message = String()
        self.lock = threading.Lock()
        read_thread = threading.Thread(target=self.read_from_port)
        read_thread.daemon = True
        read_thread.start()

        self.timer = self.create_timer(self.interval, self.timer_callback)  # Adjust the timer callback rate as needed
        self.get_logger().info('Camera 2 ready.')

    def read_from_port(self):
        while True:
            if self.serial_port.in_waiting > 0:
                with self.lock:
                    try:
                        self.latest_message.data = self.serial_port.readline().decode().strip()
                    except Exception as e:
                        self.get_logger().error(f"Unexpected Error: {e}")
                        self.serial_port.reset_input_buffer()
                        self.serial_port.reset_output_buffer()
            time.sleep(0.01)  # Small sleep to prevent excessive CPU usage

    def timer_callback(self):
        self.publisher_.publish(self.latest_message)

def main(args=None):
    rclpy.init(args=args)
    node = openmvH7Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
