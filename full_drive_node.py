import time, configparser, os, logging, subprocess
import rclpy, math
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_msgs.msg import String
import numpy as np
import tensorflow as tf
import pickle
from Adafruit_PCA9685 import PCA9685
import RPi.GPIO as GPIO
import usb.core
import usb.util

G_handler = None

G_LEFT_CAM_ID = ""
G_RIGHT_CAM_ID = ""

HPIX = 320
G_color1_g = np.zeros(HPIX, dtype=int)
G_color1_r = np.zeros(HPIX, dtype=int)
G_color2_g = np.zeros(HPIX, dtype=int)
G_color2_r = np.zeros(HPIX, dtype=int)
G_color1_m = np.zeros(HPIX, dtype=int)
G_color2_m = np.zeros(HPIX, dtype=int)

G_tf_control = False
G_clockwise = False
G_parking_lot = 0
G_cam_updates = 0

G_roll = 0.0
G_pitch = 0.0
G_yaw = 0.0
G_front_dist = 0.0
G_collision_detect = 0.0
G_pwm = None
G_speed_publisher = None
G_FWD_SPEED = ""
G_FWD_SPEEDU = ""
G_servo_neutral = 0
G_servo_ctl_fwd = 0

class fullDriveNode(Node):

    OBSTACLE_RACE_PATH_CC = "~/test/model.tflite"
    OBSTACLE_RACE_PATH_CW = "~/test/modelu.tflite"
    INITIAL_RACE_PATH_CC = "~/test/model_i.tflite"
    INITIAL_RACE_PATH_CW = "~/test/model_iu.tflite"

    OBSTACLE_SCALER_PATH_CC = "~/test/scaler.pkl"
    OBSTACLE_SCALER_PATH_CW = "~/test/scaleru.pkl"
    INITIAL_SCALER_PATH_CC = "~/test/scaler_i.pkl"
    INITIAL_SCALER_PATH_CW = "~/test/scaler_iu.pkl"

    initial_race = False
    VPIX = 200
    HFOV = 70.8
    num_scan = 1620
    num_scan2 = 810
    scan_max_dist = 2.8
    motor_ctl = -20
    relay_pin = 17
    gpio_pin = 5

    def __init__(self):
        global G_color1_r,G_color1_g,G_color2_r,G_color2_g,G_color1_m,G_color2_m
        global G_tf_control,G_parking_lot,G_clockwise,G_cam_updates
        global G_LEFT_CAM_ID, G_RIGHT_CAM_ID
        global G_speed_publisher, G_pwm, G_servo_neutral, G_servo_ctl_fwd, G_FWD_SPEED, G_FWD_SPEEDU
        global G_collision_detect, G_handler

        super().__init__('full_drive_node')
        self.publisher_ = self.create_publisher(String, 'main_logger', 10)
        G_speed_publisher = self.create_publisher(String, 'set_speed', 10)

        qos_profile = QoSProfile(
                depth=1, 
                history=QoSHistoryPolicy.KEEP_LAST, 
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE)

        servo_min = 260  # Min pulse length out of 4096
        servo_max = 380  # Max pulse length out of 4096
        G_servo_neutral = int((servo_max+servo_min)/2)
        G_servo_ctl_fwd = int(-(servo_max-servo_min)/2 * 1.1)
        self.servo_ctl_rev = int(-(servo_max-servo_min)/2 * 1.0)
        G_FWD_SPEED = "5"
        G_FWD_SPEEDU = "5"
        self.REV_SPEED = "-5"

        self._state = 'IDLE'
        self._processing = False
        G_tf_control = False
        G_clockwise = False
        self._clockwise_def = False
        self._obstacle_chk = False
        G_parking_lot = 0
        G_cam_updates = 0

        self._X = 0.0 
        self._Y = 0.0
        HPIX = 320
        G_color1_g = np.zeros(HPIX, dtype=int)
        G_color1_r = np.zeros(HPIX, dtype=int)
        G_color2_g = np.zeros(HPIX, dtype=int)
        G_color2_r = np.zeros(HPIX, dtype=int)
        G_color1_m = np.zeros(HPIX, dtype=int)
        G_color2_m = np.zeros(HPIX, dtype=int)

        self.section_means = []
        self._backward = False

        config = configparser.ConfigParser()
        config.read(os.path.expanduser('~/ros2_ws4/config.ini'))

        FWD_SPEED_initial = str(config['Speed']['forward_initial_counterclockwise'])
        FWD_SPEEDU_initial = str(config['Speed']['forward_initial_clockwise'])
        FWD_SPEED_obstacle = str(config['Speed']['forward_obstacle_counterclockwise'])
        FWD_SPEEDU_obstacle = str(config['Speed']['forward_obstacle_clockwise'])
        self.get_logger().info(f"Speed settings initial race: {FWD_SPEEDU_initial}/{FWD_SPEEDU_initial}")
        self.get_logger().info(f"Speed settings obstacle race: {FWD_SPEED_obstacle}/{FWD_SPEEDU_obstacle}")

        self.MIN_DETECTIONS_SPOT = int(config['Parking']['min_detections_spot'])
        self.MIN_DETECTIONS_TRIGGER = int(config['Parking']['min_detections_trigger'])
        self.RACE_SECTIONS = int(config['Parking']['race_sections'])
        self.STOP_DISTANCE_MAX_TURN = float(config['Parking']['stop_distance_max_turn'])
        self.STOP_DISTANCE_MIN_TURN = float(config['Parking']['stop_distance_min_turn'])
        self.STOP_DISTANCE_DELTA = float(config['Parking']['stop_distance_delta'])
        self.STOP_DISTANCE_PARK = float(config['Parking']['stop_distance_park'])
        self.STOP_DISTANCE_MAX_FINAL = float(config['Parking']['stop_distance_max_final'])
        self.STOP_DISTANCE_MIN_FINAL = float(config['Parking']['stop_distance_min_final'])
        self.get_logger().info(f"Parking detection spot / trigger: {self.MIN_DETECTIONS_SPOT} / {self.MIN_DETECTIONS_TRIGGER}")
        self.get_logger().info(f"Number of race half rounds: {self.RACE_SECTIONS}")
        self.get_logger().info(f"Stop distances min / max / park: {self.STOP_DISTANCE_MIN_TURN} / {self.STOP_DISTANCE_MAX_TURN} / {self.STOP_DISTANCE_PARK}")
        self.get_logger().info(f"Stop position final min / max : {self.STOP_DISTANCE_MIN_FINAL} / {self.STOP_DISTANCE_MAX_FINAL}")

        G_collision_detect = float(config['Parking']['collision_detect'])
        self.get_logger().info(f"Collision detection distance: {G_collision_detect}")

        G_LEFT_CAM_ID = str(config['Hardware']['left_cam_id'])
        G_RIGHT_CAM_ID = str(config['Hardware']['right_cam_id'])
        self.DONGLE_ID1 = str(config['Hardware']['dongle_id1'])
        self.DONGLE_ID2 = str(config['Hardware']['dongle_id2'])
        self.CALIBRATION_GYRO =  float(config['Hardware']['calibration_gyro'])
        self.CALIBRATION_DISTANCE =  float(config['Hardware']['calibration_distance'])
        self.WAIT_FOR_HEALTH_CHECK =  float(config['Hardware']['wait_for_health_check'])
        self.get_logger().info(f"Left / right camera IDs: {G_LEFT_CAM_ID} / {G_RIGHT_CAM_ID}")
        self.get_logger().info(f"Dongle ID: {self.DONGLE_ID1}:{self.DONGLE_ID2}")
        self.get_logger().info(f"Calibration accuracy: {self.CALIBRATION_GYRO}")
        self.get_logger().info(f"Calibration accuracy: {self.CALIBRATION_DISTANCE}")
        self.get_logger().info(f"Wait time for health check: {self.WAIT_FOR_HEALTH_CHECK} sec")

        # Initialize PCA9685
        G_pwm = PCA9685()
        G_pwm.set_pwm_freq(50)  # Set frequency to 50Hz
        G_pwm.set_pwm(0, 0, int(G_servo_neutral))
        self.get_logger().info('Steering unit initialized ...')

        self._total_heading_change = 0
        self._round_start_time = self.get_clock().now()
        self._button_time = self.get_clock().now()

        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )

        self.get_logger().info('Messages subscribed ...')

        tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

        # Look for initial race dongle
        # ID 2357:012e TP-Link 802.11ac NIC
        self.initial_race = self.check_usb_device(self.DONGLE_ID1, self.DONGLE_ID2)

        if self.initial_race:
            self.get_logger().info('Initial race mode activated ...')
            self.RACE_PATH_CC = self.INITIAL_RACE_PATH_CC
            self.RACE_PATH_CW = self.INITIAL_RACE_PATH_CW
            self.SCALER_PATH_CC = self.INITIAL_SCALER_PATH_CC
            self.SCALER_PATH_CW = self.INITIAL_SCALER_PATH_CW
            G_FWD_SPEED = FWD_SPEED_initial
            G_FWD_SPEEDU = FWD_SPEEDU_initial
        else:
            self.get_logger().info('Obstacle race mode activated ...')
            self.RACE_PATH_CC = self.OBSTACLE_RACE_PATH_CC
            self.RACE_PATH_CW = self.OBSTACLE_RACE_PATH_CW
            self.SCALER_PATH_CC = self.OBSTACLE_SCALER_PATH_CC
            self.SCALER_PATH_CW = self.OBSTACLE_SCALER_PATH_CW
            G_FWD_SPEED = FWD_SPEED_obstacle
            G_FWD_SPEEDU = FWD_SPEEDU_obstacle

        # Load the trained racing model and the scaler counterclockwise and clockwise
        with open(os.path.expanduser(self.SCALER_PATH_CC), 'rb') as f:
            self._scaler = pickle.load(f)
        self._interpreter = tf.lite.Interpreter(model_path = os.path.expanduser(self.RACE_PATH_CC))
        self._interpreter.allocate_tensors()
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        self.get_logger().info('counterclockwise prediction model loaded')

        with open(os.path.expanduser(self.SCALER_PATH_CW), 'rb') as f:
            self._scaleru = pickle.load(f)
        self._interpreteru = tf.lite.Interpreter(model_path = os.path.expanduser(self.RACE_PATH_CW))
        self._interpreteru.allocate_tensors()
        self._input_detailsu = self._interpreter.get_input_details()
        self._output_detailsu = self._interpreter.get_output_details()
        self.get_logger().info('clockwise prediction model loaded')

        # touch button
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.gpio_pin, GPIO.FALLING, callback=self.gpio_callback)
        self._button_time = 0

        self.log_timer = self.create_timer(10, self.log_timer_callback)

        #logging.error("Test error")

        # health self check
        time.sleep(self.WAIT_FOR_HEALTH_CHECK)
        if not self.health_check():
            self.prompt("PANIC. Not ready.")
            self.get_logger().error('PANIC. Not ready.')
            self.motor_off()
            self.get_logger().info(f"ROS2 shutdown requested")
            rclpy.shutdown()
        else:
            self.prompt("Ready!")
            self.get_logger().info('Ready.')


    def __del__(self):
        self.get_logger().info('Switch off ESC')
        self.motor_off()


    def health_check(self):
        global G_handler
        healthy = True
        self.get_logger().info("Performing final health self check")

        self.node_list = {'/WT61_node',
                     '/distance_sensor_node',
                     '/full_drive_node',
                     '/sllidar_node',
                     '/openmv_h7_node1',
                     '/openmv_h7_node2',
                     '/display_node',
                     '/speed_control_node'
                     }
        for i in range(10):
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
            #self.get_logger().info(f"Subprocess result: {result}")
            current_nodes = set(result.stdout.split('\n'))
            missing_nodes = self.node_list - current_nodes
            if not missing_nodes: break
            self.get_logger().info(f"Missing nodes at {i+1}. check: {missing_nodes}")
            time.sleep(5)
        if missing_nodes:
            healthy = False

        if G_handler.error_occurred:
            self.get_logger().info(f"Error message: {G_handler.error_message}")
            healthy = False

        return healthy

    def log_timer_callback(self):
        self.get_logger().info(f"Heading change: {self._total_heading_change}, parking lot spotted: {G_parking_lot}")

    def prompt(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)

    @classmethod
    def motor_off(self):
        #self.get_logger().info("motor_off called.")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.LOW)
        GPIO.cleanup()

    @classmethod
    def boost(self):
        msg = String()
        msg.data = "BOOST"
        G_speed_publisher.publish(msg)

    @classmethod
    def reset(self):
        global G_clockwise, G_speed_publisher, G_FWD_SPEED, G_FWD_SPEEDU
        msg = String()
        msg.data = "RESET"
        G_speed_publisher.publish(msg)
        # lower speed for clockwise race, speed measurement at inner wheel !
        msg.data = G_FWD_SPEED if not G_clockwise else G_FWD_SPEEDU
        G_speed_publisher.publish(msg)

    @classmethod
    def steer(self,X,sleep):
        global G_pwm, G_servo_neutral, G_servo_ctl_fwd
        XX = int(G_servo_neutral+X*G_servo_ctl_fwd)
        G_pwm.set_pwm(0, 0, XX)
        if sleep: time.sleep(1.0)

    @classmethod
    def move(self, dist):
        global G_speed_publisher
        msg = String()
        msg.data = dist
        G_speed_publisher.publish(msg)
        time.sleep(5.0)

    def move_m(self, dist):
        if np.isnan(dist): return
        dir = "F" if dist >= 0 else "R"
        self.move(dir+str(abs(int(dist*self.CALIBRATION_DISTANCE))))

    def start_race(self):
        global G_tf_control,G_parking_lot, G_roll
        self._state = "RACE"
        G_tf_control = True
        G_parking_lot = 0

        self._initial_heading = G_roll
        self._start_heading = self._initial_heading
        self._last_heading = self._initial_heading
        self._total_heading_change = 0
        self.get_logger().info(f"Initial heading: {self._initial_heading} degrees")
        self._round_start_time = self.get_clock().now()

    @classmethod
    def stop(self):
        global G_speed_publisher
        msg = String()
        msg.data = "STOP"
        G_speed_publisher.publish(msg)

    def stop_race(self):
        global G_tf_control, G_pwm
        self._state = "IDLE"
        G_tf_control = False
        self._processing = False
        self.steer(0,False)
        self.stop()
        self.motor_off()
        self.get_logger().info(f"ROS2 shutdown requested")
        rclpy.shutdown()

    def calculate_heading_change(self, start_heading, current_heading):
        # Calculate the raw difference
        raw_diff = current_heading - start_heading
        # Adjust the difference to find the shortest path and preserve the turn direction
        if raw_diff > 180:
            return raw_diff - 360
        elif raw_diff < -180:
            return raw_diff + 360
        else:
            return raw_diff

    def check_usb_device(self, vendor_id, product_id):
        # Convert vendor_id and product_id from hexadecimal string to integer
        vendor_id = int(vendor_id, 16)
        product_id = int(product_id, 16)
        # Find USB device with specific Vendor ID and Product ID
        device = usb.core.find(idVendor=vendor_id, idProduct=product_id)
        # Return True if device is found, else False
        if device is not None:
            self.get_logger().info("Device Found: ID {0}:{1}".format(vendor_id, product_id))
            return True
        else:
            return False

    def race_report(self):
        duration_in_seconds = (self.get_clock().now() - self._round_start_time).nanoseconds * 1e-9
        self.get_logger().info(f"Race in {duration_in_seconds} sec completed!")
        self.get_logger().info(f"Heading change: {self._total_heading_change} Distance: {self.front_dist()}")
        self.get_logger().info(f"Parking lot detections {G_parking_lot}")

    def front_dist(self):
        global G_front_dist
        dist = 0
        #self.get_logger().info(f"Distances: {self.section_means}")
        if len(self.section_means) > 0:
            dist = max(self.section_means[78:83])
            if np.isnan(dist): dist = 0
        if np.isnan(G_front_dist): G_front_dist = 0
        return max(dist,G_front_dist)

    def lidar_callback(self, msg):
        global G_color1_r,G_color1_g,G_color2_r,G_color2_g,G_color1_m,G_color2_m
        global G_tf_control,G_parking_lot,G_clockwise,G_cam_updates, G_roll

        if self._processing:
            self.get_logger().warn('Scan skipped')
            return
        else:
            self._processing = True

            #self.get_logger().info(f"Cam updates per lidar callback {G_cam_updates}")
            G_cam_updates = 0

            scan = np.array(msg.ranges[self.num_scan+self.num_scan2:]+msg.ranges[:self.num_scan2])
            scan[scan == np.inf] = np.nan
            scan[scan > self.scan_max_dist] = np.nan

            num_sections = 161
            section_data = np.array_split(scan, num_sections)
            self.section_means = [np.mean(section) for section in section_data]
            min_far_dist = min(self.section_means[60:101])
            min_near_dist = min(self.section_means[40:121])
            side_dist_left = min(self.section_means[0:11])
            side_dist_right = min(self.section_means[150:161])
            fwd_side_dist_left = min(self.section_means[30:41])
            fwd_side_dist_right = min(self.section_means[120:131])
            self._cal_left = self.section_means[70]+self.section_means[60]
            self._cal_right = self.section_means[91]+self.section_means[101]

            ########################
            # RACE
            ########################
            if self._state == 'RACE' and G_tf_control:

                #self.get_logger().info(f"Total heading change: {self._total_heading_change}")
                self._current_heading = G_roll
                heading_change = self.calculate_heading_change(self._last_heading, self._current_heading)
                self._total_heading_change += heading_change
                self._last_heading = self._current_heading

                if abs(self._cal_left/self._cal_right -1) < self.CALIBRATION_GYRO and self._cal_left+self._cal_right < 5.0:
                    self.get_logger().info(f"Calibration pre: {self._total_heading_change}")
                    add = -45 if G_clockwise else 45
                    self._total_heading_change = int((self._total_heading_change+add)/90)*90.0
                    self.get_logger().info(f"Calibration aft: {self._total_heading_change}")

                if G_parking_lot > self.MIN_DETECTIONS_SPOT and abs(self._total_heading_change) >= (self.RACE_SECTIONS*360-10):
                    if ((not G_clockwise and sum(G_color2_m) > self.MIN_DETECTIONS_TRIGGER) or
                            (G_clockwise and sum(G_color1_m) > self.MIN_DETECTIONS_TRIGGER))\
                            and self.STOP_DISTANCE_MIN_FINAL < self.front_dist() < self.STOP_DISTANCE_MAX_TURN:
                        self.race_report()
                        self.prompt("Parking ...")
                        self._state = "PARK"
                        self._processing = False
                        self._park_phase = 0
                        return

                elif G_parking_lot <= self.MIN_DETECTIONS_SPOT and \
                        abs(self._total_heading_change) >= (self.RACE_SECTIONS*360-10) and \
                        self.front_dist() < 1.6:
                    self.race_report()
                    self.prompt("Stopping ...")
                    self.stop()
                    self._state = "STOP"
                    self._processing = False
                    self._stop_phase = 0
                    self.steer(0,True)
                    return

                try:
                    if not self._obstacle_chk:
                        self._obstacle_chk = True

                        self.get_logger().info(f"Obstacle: {min(side_dist_left,side_dist_right)}, {min_far_dist}, {min_near_dist}")
                        self.get_logger().info(f"Front distance: {self.front_dist()}")
                        if not self.initial_race and (np.isnan(min_far_dist) or min_far_dist < 0.8 or min_near_dist < 0.2):
                            self._backward = True
                            self.move_m(self.front_dist() - 2.3 )
                        elif self.front_dist() > 1.3:
                            self.move_m(self.front_dist() - 1.3 ) 
                        self._processing = False
                        return

                    elif not self._clockwise_def:
                        self._clockwise_def = True

                        if min(side_dist_left,side_dist_right) < 0.15:
                            G_clockwise = side_dist_left > side_dist_right
                            self.get_logger().info(f"Decide clockwise mode 1: {side_dist_left}, {side_dist_right}")
                        else:
                            sum_left_half = np.nansum(scan[0:self.num_scan2])
                            sum_right_half = np.nansum(scan[self.num_scan2+1:self.num_scan])
                            G_clockwise = (sum_left_half <= sum_right_half)
                            self.get_logger().info(f"Decide clockwise mode 2: {sum_left_half}, {sum_right_half}")
                            #self.get_logger().info(f"Left side: {scan[0:self.num_scan2]}")
                            #self.get_logger().info(f"Right side: {scan[self.num_scan2+1:self.num_scan]}")

                        self.get_logger().info(f"Clockwise: {G_clockwise}")
                        if self._backward:
                            self._backward = False
                            self.steer(-1.0 if G_clockwise else 1.0,True)
                            self.move_m(0.4)
                            time.sleep(5)

                        self.reset()

                    x = np.arange(len(scan))
                    finite_vals = np.isfinite(scan)
                    scan_interpolated = np.interp(x, x[finite_vals], scan[finite_vals])
                    scan_interpolated = [1/value if value != 0 else 0 for value in scan_interpolated]
                    scan_interpolated = list(scan_interpolated)
                    color_data = list(G_color1_g) + list(G_color2_g) + list(G_color1_r) + list(G_color2_r)

                    lidar_data = np.reshape(scan_interpolated, (1, -1))               # Reshape LIDAR data
                    color_data_standardized = np.reshape(color_data, (1, -1))         # Reshape COLOR data
                    # Reshape color_data to (1, 1, 1) to match dimensions
                    color_data_standardized = np.reshape(color_data_standardized, (1, color_data_standardized.shape[1], 1)).astype(np.float32)

                    if not G_clockwise:
                        lidar_data_standardized = self._scaler.transform(lidar_data)
                        # Reshape for TFLite model input
                        lidar_data_standardized = np.reshape(lidar_data_standardized, (1, lidar_data_standardized.shape[1], 1)).astype(np.float32)
                        # Combine LIDAR and color data for the model input (concatenation, as required by your model)
                        self._interpreter.set_tensor(self._input_details[0]['index'], lidar_data_standardized)
                        self._interpreter.set_tensor(self._input_details[1]['index'], color_data_standardized)
                        # Run inference
                        self._interpreter.invoke()
                        # Retrieve the output of the model
                        predictions = self._interpreter.get_tensor(self._output_details[0]['index'])
                    else:
                        lidar_data_standardized = self._scaleru.transform(lidar_data)
                        lidar_data_standardized = np.reshape(lidar_data_standardized, (1, lidar_data_standardized.shape[1], 1)).astype(np.float32)
                        self._interpreteru.set_tensor(self._input_detailsu[0]['index'], lidar_data_standardized)
                        self._interpreteru.set_tensor(self._input_detailsu[1]['index'], color_data_standardized)
                        self._interpreteru.invoke()
                        predictions = self._interpreteru.get_tensor(self._output_detailsu[0]['index'])

                    self._X = predictions[0, 0]
                    self._Y = predictions[0, 1]
                    #self.get_logger().info('Predicted axes: "%s"' % predictions)
                    self.steer(self._X,False)

                except ValueError as e:
                    self.get_logger().error('Model rendered nan: %s' % str(e))
                    logging.error('Model rendered nan: %s' % str(e))

                except IOError as e:
                    self.get_logger().error('IOError I2C occurred: %s' % str(e))
                    logging.error('IOError I2C occurred: %s' % str(e))

            ########################
            # PARK
            ########################
            elif self._state == 'PARK':

                if self._park_phase == 0:
                    self.stop()
                    self.steer(0,True)
                    dist = self.front_dist()
                    delta = self.STOP_DISTANCE_DELTA if (G_clockwise and fwd_side_dist_left < fwd_side_dist_right) or \
                        (not G_clockwise and fwd_side_dist_left > fwd_side_dist_right) else 0.00
                    self.get_logger().info(f"Front distance, delta: {dist}, {delta}")
                    if self.STOP_DISTANCE_MIN_TURN < dist+delta < self.STOP_DISTANCE_MAX_TURN: # 1.45 < dist < 1.55
                        self._park_phase = 2
                    else:
                        self.move_m(dist+delta-(self.STOP_DISTANCE_MIN_TURN+self.STOP_DISTANCE_MIN_TURN)/2)

                elif self._park_phase == 2:
                    X = -1.0 if G_clockwise else 1.0
                    self.steer(X,True)
                    self.boost()
                    self.reset()
                    self._park_phase = 3

                elif self._park_phase == 3:
                    #self.get_logger().info(f"Side Distance: {min_near_dist}")
                    if self.front_dist() < self.STOP_DISTANCE_PARK and min_near_dist < self.STOP_DISTANCE_PARK:
                        self.stop_race()
                        self._state = "IDLE"

            ########################
            # STOP
            ########################
            elif self._state == 'STOP':

                if self._stop_phase == 0:
                    dist = self.front_dist()
                    #self.get_logger().info(f"Front distance: {dist}")
                    if self.STOP_DISTANCE_MIN_FINAL < dist < self.STOP_DISTANCE_MAX_FINAL: # 1.2 < dist < 1.6
                        self._stop_phase = 1
                    else:
                        self.move_m(dist-(self.STOP_DISTANCE_MIN_FINAL+self.STOP_DISTANCE_MAX_FINAL)/2)

                else:
                    self.stop_race()
                    self._state = "IDLE"


            ########################
            # IDLE
            ########################
            elif self._state == 'IDLE':
                pass

            self._processing = False

    def gpio_callback(self, channel):
        global G_tf_control
        if not G_tf_control:
            self._button_time = self.get_clock().now()
            self.get_logger().info('Start button pressed!')
            self.start_race()
        else:
            duration_in_seconds = (self.get_clock().now() - self._button_time).nanoseconds * 1e-9
            if duration_in_seconds > 5:
                self.get_logger().info('Stop button pressed!')
                self.stop_race()

class cameraNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(String, 'main_logger', 10)
        self._busy = False

        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.subscription = self.create_subscription(
            String,
            name,
            self.openmv_h7_callback,
            qos_profile
        )

    def prompt(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)

    def openmv_h7_callback(self, msg):
        global G_color1_r,G_color1_g,G_color2_r,G_color2_g,G_color1_m,G_color2_m
        global G_tf_control,G_parking_lot,G_clockwise, G_cam_updates
        global G_LEFT_CAM_ID, G_RIGHT_CAM_ID

        if self._busy: return
        self._busy = True

        HPIX = 320
        WEIGHT = 1

        try:
            data = msg.data.split(',')
            if data[0] == G_LEFT_CAM_ID: cam = 1      # 33001c000851303436373730 / 240024001951333039373338
            elif data[0] == G_RIGHT_CAM_ID: cam = 2   # 2d0024001951333039373338 / 340046000e51303434373339
            else:
                self._busy = False
                return

            if cam == 1:
                G_color1_g = np.zeros(HPIX, dtype=int)
                G_color1_r = np.zeros(HPIX, dtype=int)
                G_color1_m = np.zeros(HPIX, dtype=int)
            elif cam == 2:
                G_color2_g = np.zeros(HPIX, dtype=int)
                G_color2_r = np.zeros(HPIX, dtype=int)
                G_color2_m = np.zeros(HPIX, dtype=int)

            blobs = ((data[i],data[i+1],data[i+2]) for i in range (1,len(data),3))
            for blob in blobs:
                color, x1, x2 = blob
                color = int(color)
                ix1 = int(x1)
                ix2 = int(x2)

                if color == 1 or color == 2:
                    if cam == 1 and not G_clockwise:
                        G_cam_updates += 1
                        G_color1_g[ix1:ix2] = WEIGHT
                        self.prompt('*,'+x1+','+x2+',G')
                    if cam == 2 and G_clockwise:
                        G_cam_updates += 1
                        G_color2_g[ix1:ix2] = WEIGHT
                        self.prompt('*,'+x1+','+x2+',G')
                if color == 4 or color == 8:
                    if cam == 1 and not G_clockwise:
                        G_cam_updates += 1
                        G_color1_r[ix1:ix2] = WEIGHT
                        self.prompt('*,'+x1+','+x2+',R')
                    if cam == 2 and G_clockwise:
                        G_cam_updates += 1
                        G_color2_r[ix1:ix2] = WEIGHT
                        self.prompt('*,'+x1+','+x2+',R')
                if color == 16:
                    G_cam_updates += 1
                    if cam == 1: G_color1_m[ix1:ix2] = WEIGHT
                    if cam == 2: G_color2_m[ix1:ix2] = WEIGHT
                    G_parking_lot += 1

        except Exception as e:
            self.get_logger().error(f"Faulty cam msg received: {msg.data} {e}")
            logging.error(f"Faulty cam msg received: {msg.data} {e}")
        finally:
            self._busy = False


class imuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        self.get_logger().info("imu_node initialized")

        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.subscription_imu = self.create_subscription(
            Imu,
            'wit/imu',
            self.imu_callback,
            qos_profile
        )

        self.subscription_imu = self.create_subscription(
            String,
            'imu_warnings',
            self.imu_warning_callback,
            qos_profile
        )

        #logging.error("IMU test error message")


    def quaternion_to_euler(self,quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def imu_callback(self, msg):
        global G_roll, G_pitch, G_yaw
        quaternion = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(quaternion)
        G_roll = math.degrees(roll)
        G_pitch = math.degrees(pitch)
        G_yaw = math.degrees(yaw)
        #self.get_logger().info(f"Roll: {G_roll}, Pitch: {G_pitch}, Yaw: {G_yaw}")

    def imu_warning_callback(self, msg):
        self.get_logger().error(f"IMU problem: {msg.data}")
        logging.error(f"IMU problem: {msg.data}")

class distanceNode(Node):
    def __init__(self):
        super().__init__("distance_node")

        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.subscription_speed = self.create_subscription(
            Float32,
            'distance_sensor',
            self.distance_sensor_callback,
            qos_profile
        )

    def distance_sensor_callback(self, msg):
        global G_front_dist, G_tf_control, G_collision_detect
        G_front_dist = msg.data
        #self.get_logger().info(f"Distance: {msg.data}")

        if G_tf_control and G_collision_detect > 0 and G_front_dist < G_collision_detect:
            self.get_logger().info(f"Collision detected: {G_front_dist}, push back")
            G_tf_control = False
            fullDriveNode.stop()
            for i in range(40):
                fullDriveNode.steer(0.0,False)
                time.sleep(0.05)
            fullDriveNode.move("R30")
            fullDriveNode.reset()
            G_tf_control = True
            return

class errorLogHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.error_occurred = False
        self.error_message = None

    def emit(self, record):
        if record.levelno == logging.ERROR:
            self.error_occurred = True
            self.error_message = record.getMessage()
            print(f"Error logged: {record.getMessage()}")

def main(args=None):
    global G_handler

    rclpy.init(args=args)
    G_handler = errorLogHandler()
    G_handler.setLevel(logging.ERROR)
    logging.getLogger().addHandler(G_handler)
    logging.getLogger().setLevel(logging.DEBUG)  # Set a low threshold to capture all log levels

    imu_node = imuNode()
    cam1_node = cameraNode('openmv_topic1')
    cam2_node = cameraNode('openmv_topic2')
    distance_node = distanceNode()
    full_drive_node = fullDriveNode()

    # Use MultiThreadedExecutor to allow parallel callback execution
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(full_drive_node)
    executor.add_node(cam1_node)
    executor.add_node(cam2_node)
    executor.add_node(imu_node)
    executor.add_node(distance_node)

    try:
        while rclpy.ok():
            executor.spin_once()
    finally:
        executor.shutdown()
        full_drive_node.destroy_node()
        cam1_node.destroy_node()
        cam2_node.destroy_node()
        imu_node.destroy_node()
        distance_node.destroy_node()
        #rclpy.shutdown()

    try:
        fullDriveNode.motor_off()
        with open('/tmp/ros2_pipe', 'w') as pipe:
            pipe.write('shutdown\n')
    except:
        pass

if __name__ == '__main__':
    main()
