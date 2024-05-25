import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, time, threading, configparser

class openmvH7Node(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'openmv_topic1', 10)
        self.publisher_log_ = self.create_publisher(String, 'main_logger', 10)

        config = configparser.ConfigParser()
        config.read('/home/rrrschuetz/ros2_ws4/config.ini')
        self.interval = float(config['Hardware']['camera_timer'])
        self.get_logger().info(f"Camera 1: timer interval: {self.interval}")

        self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=5)   #115200
        self.get_logger().info('OpenMV H7 1 connected' )
        with open("/home/rrrschuetz/ros2_ws4/src/s2e_lidar_reader/s2e_lidar_reader/h7_cam_exec.py", 'rb') as file:
            script_data = file.read()
            self.serial_port.write(script_data)
            self.get_logger().info('OpenMV H7 1 script sent' )
        #time.sleep(10)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

        self.consolidated_data = {1: [], 2: [], 4:[]}
        self.latest_message = String()
        self.lock = threading.Lock()
        read_thread = threading.Thread(target=self.read_from_port)
        read_thread.daemon = True
        read_thread.start()

        self.timer = self.create_timer(self.interval, self.timer_callback)  # Adjust the timer callback rate as needed
        self.get_logger().info('Camera 1 ready.')

    def read_from_port(self):
        while True:
            if self.serial_port.in_waiting > 0:
                with self.lock:
                    try:
                        self.latest_message.data = self.serial_port.readline().decode().strip()
                        data = self.latest_message.data.split(',')
                        blobs = [(int(data[i]), int(data[i+1]), int(data[i+2])) for i in range(1, len(data), 3)]
                        for blob in blobs:
                            color_id, x1_new, x2_new = blob
                            merged = False
                            for existing_blob in self.consolidated_data[color_id]:
                                x1_exist, x2_exist = existing_blob
                                x1_exist = int(x1_exist)
                                x2_exist = int(x2_exist)1
                                if not (x2_new < x1_exist or x1_new > x2_exist):
                                    existing_blob[1] = min(x1_exist, x1_new)
                                    existing_blob[2] = max(x2_exist, x2_new)
                                    merged = True
                                    break
                            if not merged:
                                self.consolidated_data[color_id].append([x1_new, x2_new])

                    except Exception as e:
                        self.get_logger().error(f"Unexpected Error: {e}")
                        self.serial_port.reset_input_buffer()
                        self.serial_port.reset_output_buffer()
            time.sleep(0.01)  # Small sleep to prevent excessive CPU usage

    def timer_callback(self):
        msg = String()
        blob_data = []
        color_ids = [1, 2, 4]

        for color_id in color_ids:
            if color_id in self.consolidated_data:
                for blob in self.consolidated_data[color_id]:
                    x1, x2 = blob
                    blob_data.append(f"{color_id},{x1},{x2}")

        msg.data = ",".join(blob_data)
        self.publisher_.publish(msg)
        self.consolidated_data = {color_id: [] for color_id in color_ids}

def main(args=None):
    rclpy.init(args=args)
    node = openmvH7Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

