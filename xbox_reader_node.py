import rclpy
from rclpy.node import Node
from evdev import InputDevice, ecodes

from sensor_msgs.msg import Joy

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller_node')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        try:
            self.device = InputDevice('/dev/input/event0')
            self.get_logger().info('Device initialized: %s' % self.device)
        except Exception as e:
            self.get_logger().error('Failed to initialize device: %s' % str(e))
            raise
        self.axes = {
            ecodes.ABS_X: 0,  # left stick horizontal
            ecodes.ABS_Y: 1,  # left stick vertical
            ecodes.ABS_Z: 2,  # left trigger
            ecodes.ABS_RZ: 3  # right trigger
        }
        self.get_logger().info('Xbox Controller Node is ready.',)

    def run(self):
        self.get_logger().info('Start reading events from the Xbox controller.')
        axes = [0.0]*4

        for event in self.device.read_loop():

            if event.type == ecodes.EV_ABS:
                #self.get_logger().info('Received an EV_ABS event.')
                axis = self.axes.get(event.code)
                if axis is not None:
                    #self.get_logger().info('Received an EV_ABS event.')
                    axes[axis] = float(event.value-32768)/32768
                    self.publish_axes_state(axes)

            #elif event.type == ecodes.EV_KEY:
            #    self.get_logger().info('Received an EV_KEY event.')

    def publish_axes_state(self, axes):
        msg = Joy()
        # Assuming axis is an integer representing the index of the axis
        msg.axes = axes 
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.set_logger_level('xbox_controller_node', rclpy.logging.LoggingSeverity.DEBUG)

    try:
        xbox_controller_node = XboxControllerNode()

        xbox_controller_node.get_logger().info('Calling run method.')
        xbox_controller_node.run()
        xbox_controller_node.get_logger().info('Run method finished.')

    except Exception as e:
        xbox_controller_node.get_logger().error('An error occurred: %s' % str(e))
        raise

    finally:
        xbox_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
