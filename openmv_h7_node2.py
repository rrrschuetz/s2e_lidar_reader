import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, time, threading, configparser, os

class openmvH7Node(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'openmv_topic2', 10)
        self.publisher_log_ = self.create_publisher(String, 'main_logger', 10)

        config = configparser.ConfigParser()
        config.read(os.path.expanduser('~/ros2_ws4/config.ini'))
        self.cam_software_path = config['Hardware']['cam_software_path']
        self.interval = float(config['Hardware']['camera_timer'])
        self.get_logger().info(f"Camera 2: Software path: {self.cam_software_path}")
        self.get_logger().info(f"Camera 2: timer interval: {self.interval}")

        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=5)   #115200
        self.get_logger().info('OpenMV H7 2 connected' )
        with open(os.path.expanduser(self.cam_software_path), 'rb') as file:
            script_data = file.read()
            self.serial_port.write(script_data)
            self.get_logger().info('OpenMV H7 2 script sent' )
        #time.sleep(10)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

        self.cam_id = ""
        self.consolidated_data = {1: [], 2: [], 4:[], 8:[], 16:[]}
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
                        #self.get_logger().info(f"{self.latest_message.data}")
                        data = self.latest_message.data.split(',')
                        self.cam_id = data[0]
                        blobs = [(int(data[i]), int(data[i+1]), int(data[i+2])) for i in range(1, len(data), 3)]
                        #self.get_logger().info(f"{blobs}")
                        for blob in blobs:
                            color_id, x1_new, x2_new = blob
                            merged = False
                            for existing_blob in self.consolidated_data[color_id]:
                                x1_exist, x2_exist = existing_blob
                                x1_exist = int(x1_exist)
                                x2_exist = int(x2_exist)
                                if not (x2_new < x1_exist or x1_new > x2_exist):
                                    existing_blob[0] = min(x1_exist, x1_new)
                                    existing_blob[1] = max(x2_exist, x2_new)
                                    merged = True
                                    break
                            if not merged:
                                self.consolidated_data[color_id].append([x1_new, x2_new])

                        #self.get_logger().info(f"{self.consolidated_data}")

                    except Exception as e:
                        self.get_logger().error(f"Unexpected Error: {e}")
                        self.serial_port.reset_input_buffer()
                        self.serial_port.reset_output_buffer()
            time.sleep(0.01)  # Small sleep to prevent excessive CPU usage


    def timer_callback(self):
        msg = String()
        blob_data = []
        color_ids = [1, 2, 4, 8, 16]

        for color_id in color_ids:
            if color_id in self.consolidated_data:
                for blob in self.consolidated_data[color_id]:
                    x1, x2 = blob
                    blob_data.append(f"{color_id},{x1},{x2}")

        if blob_data:
            msg.data = self.cam_id + ","+",".join(blob_data)
            #self.get_logger().info(f"*2* {msg.data}")
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
