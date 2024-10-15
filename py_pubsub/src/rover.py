import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

# Axis and button definitions for the PowerA Nintendo Switch gamepad
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

        # Create a subscription to the joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Create a publisher for the diffbot
        self.diff_cmd = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)

    def joy_callback(self, msg):
        # Create a new Twist message
        cmd = Twist()

        # Get joystick inputs
        fwd = msg.axes[Axis.LEFT_STICK_Y]
        turn = msg.axes[Axis.RIGHT_STICK_X]

        # Set linear and angular velocity
        cmd.linear.x = fwd
        cmd.angular.z = turn

        # Publish the velocity command
        self.diff_cmd.publish(cmd)

        if msg.buttons[Button.A]:
            print("Got command A: Scooping")
            time.sleep(10)  # Wait for 10 seconds

            for i in range(20):
                cmd.linear.x = -1.0  # Move backward
                cmd.angular.z = 0.0
                self.diff_cmd.publish(cmd)
                time.sleep(0.05)  # Wait for 50 ms

        if msg.buttons[Button.Y]:
            print("Got command Y: Dumping")
            time.sleep(10)  # Wait for 10 seconds

            for i in range(40):
                cmd.linear.x = 1.0  # Move forward
                cmd.angular.z = 0.0
                self.diff_cmd.publish(cmd)
                time.sleep(0.05)  # Wait for 50 ms

def main(args=None):
    rclpy.init(args=args)
    node = Joy2Cmd()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
