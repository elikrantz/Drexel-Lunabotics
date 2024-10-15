import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy  # message type used by the joy_node
from geometry_msgs.msg import Twist  # message type used for diffbot

# Axes and buttons of 8PowerA Nintendo Switch gamepad
class Axis:
    LEFT_STICK_X = 0
    LEFT_STICK_Y = 1
    RIGHT_STICK_X = 2
    RIGHT_STICK_Y = 3
    D_PAD_Y = 4
    D_PAD_X = 5

class Button:
    Y = 0
    B = 1
    A = 2
    X = 3
    L = 4
    R = 5
    ZL = 6
    ZR = 7
    MINUS = 8
    PLUS = 9
    LEFT_STICK_BTN = 10
    RIGHT_STICK_BTN = 11
    HOME = 12
    CIRCLE = 13


class Joy2Cmd(Node):
    def __init__(self):
        super().__init__('joy2cmd')

        # Create the subscription to the Joy topic
        self.subscription = self.create_subscription(
            Joy, 'joy', self.topic_callback, 10)

        # Publisher for diffbot's velocity command
        self.diff_cmd = self.create_publisher(
            Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)

    def topic_callback(self, msg):
        # Create a Twist message
        cmd_ = Twist()

        # Read joystick inputs
        fwd = msg.axes[Axis.LEFT_STICK_Y]  # Forward/Backward control
        turn = msg.axes[Axis.RIGHT_STICK_X]  # Turn control

        # Assign the linear and angular velocities
        cmd_.linear.x = fwd
        cmd_.angular.z = turn

        # Publish the velocity command to diffbot
        self.diff_cmd.publish(cmd_)


def main(args=None):
    rclpy.init(args=args)

    # Instantiate the Joy2Cmd node
    joy2cmd_node = Joy2Cmd()

    # Spin the node so it keeps processing Joy messages
    rclpy.spin(joy2cmd_node)

    # Clean up after the node is shut down
    joy2cmd_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
