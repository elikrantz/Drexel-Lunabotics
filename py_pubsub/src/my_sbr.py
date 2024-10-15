import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy  # Message type used by the joy_node

# Enum for the axes and buttons of the 8BitDo Pro 2
class Axis:
    LEFT_STICK_X = 0
    LEFT_STICK_Y = 1
    LEFT_TRIGGER = 2
    RIGHT_STICK_X = 3
    RIGHT_STICK_Y = 4
    D_PAD_X = 5
    D_PAD_Y = 6

class Button:
    Y = 3
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

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')  # Node name
        self.subscription_ = self.create_subscription(
            Joy,
            'joy',  # Topic name
            self.topic_callback,  # Callback function
            10)  # Queue size

    def topic_callback(self, msg):
        # Construct message with values from the gamepad
        to_print = f"Front: {msg.axes[Axis.LEFT_STICK_Y]}, Turn: {msg.axes[Axis.LEFT_STICK_X]}"
        self.get_logger().info(f"I heard: {to_print}")

def main(args=None):
    rclpy.init(args=args)
    joy_subscriber = JoySubscriber()
    rclpy.spin(joy_subscriber)  # Keep the node running to listen for messages
    joy_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
