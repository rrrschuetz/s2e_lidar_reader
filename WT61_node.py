#!/usr/bin/env python3
import serial
import struct
import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from std_msgs.msg import String

def quaternion_from_euler(roll, pitch, yaw):
    # Convert Euler angles (given in degrees) to quaternion
    # Assuming roll, pitch, yaw are in degrees for this conversion
    c1 = math.cos(math.radians(yaw / 2))
    s1 = math.sin(math.radians(yaw / 2))
    c2 = math.cos(math.radians(pitch / 2))
    s2 = math.sin(math.radians(pitch / 2))
    c3 = math.cos(math.radians(roll / 2))
    s3 = math.sin(math.radians(roll / 2))
    c1c2 = c1 * c2
    s1s2 = s1 * s2
    w = c1c2 * c3 - s1s2 * s3
    x = c1c2 * s3 + s1s2 * c3
    y = s1 * c2 * c3 + c1 * s2 * s3
    z = c1 * s2 * c3 - s1 * c2 * s3
    return Quaternion(x=x, y=y, z=z, w=w)

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyS0'),
                ('baud', 115200),
            ]
        )  #115200

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.serial_port = serial.Serial(port=port, baudrate=baud, timeout=10)
        self.imu_pub = self.create_publisher(Imu, 'wit/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'wit/mag', 10)
        self.location_pub = self.create_publisher(NavSatFix, 'wit/location', 10)
        self.cali_sub = self.create_subscription(String, 'wit/cali', self.handle_command, 10)

        self.buff = bytearray()

        self.last_received_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_imu_data_timeout)
        self.timer = self.create_timer(0.1, self.read_serial_data)
        self.get_logger().info('WT61 configuration ready.')

    def read_serial_data(self):
        if self.serial_port.in_waiting:
            data = self.serial_port.read(self.serial_port.in_waiting)
            self.handle_serial_data(data)
            self.last_received_time = self.get_clock().now()

    def handle_serial_data(self, data):
        for byte in data:
            self.buff.append(byte)
            if len(self.buff) >= 11:  # Assuming each packet length is 11 for now
                if self.buff[0] == 0x55:  # Start of packet as per device protocol
                    self.process_packet(self.buff[:11])
                self.buff = self.buff[11:]

    def process_packet(self, packet):
        if not self.check_sum(packet):
            self.get_logger().warn('Checksum failed')
            #self.reset_device()
            return

        data_type = packet[1]
        if data_type == 0x53:  # Angle data example
            angle_data = struct.unpack('<hhh', packet[2:8])
            yaw, pitch, roll = [x / 32768.0 * 180 for x in angle_data]  # Convert to degrees
            quat = quaternion_from_euler(roll, pitch, yaw)
            self.publish_imu_orientation(quat)
            self.get_logger().info(f"Quaternion published: yaw: {yaw}, pitch: {pitch}, roll: {roll}")

    def check_sum(self, packet):
        return sum(packet[:-1]) & 0xff == packet[-1]

    def publish_imu_orientation(self, quat):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.orientation = quat
        self.imu_pub.publish(imu_msg)

    def check_imu_data_timeout(self):
        if (self.get_clock().now() - self.last_received_time).nanoseconds > 1e9:
            self.reset_device()

    def reset_device(self):
        self.get_logger().info('No IMU data for 1 second, resetting device...')
        if self.serial_port.is_open:
            self.serial_port.close()
        time.sleep(1)
        self.serial_port.open()

    def handle_command(self, msg):
        command = msg.data
        if "start_calib" in command:
            self.start_calibration()
        elif "stop_calib" in command:
            self.stop_calibration()

    def start_calibration(self):
        self.get_logger().info('Starting calibration...')

    def stop_calibration(self):
        self.get_logger().info('Stopping calibration...')

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(imu_node)
    executor.spin()
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
