import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
#from ros_phoenix.msg import MotorControl
from geometry_msgs.msg import Twist
import serial
import time

# Axes and buttons mapping for the PowerA Nintendo Switch gamepad
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

        # Set up the serial communication
        serial_device = "/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2:1.0"
        self.serial_conn = serial.Serial(serial_device, baudrate=9600, timeout=1)
        self.get_logger().info("Serial connection established")

        # Subscribe to the 'joy' topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # The size of the queue is 10 messages. 10 commands per second
        # Publisher to send velocity commands to the diffbot
        self.diff_cmd_publisher = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)

    def joy_callback(self, msg):
        # Process joystick axes
        fwd = msg.axes[Axis.LEFT_STICK_Y]
        turn = msg.axes[Axis.RIGHT_STICK_X]

        # Create Twist message for diffbot control
        cmd = Twist()
        cmd.linear.x = fwd
        cmd.angular.z = turn

        # Publish velocity command
        self.diff_cmd_publisher.publish(cmd)

        # Send serial commands based on button presses
        if msg.buttons[Button.R] == 1:  # Extend two big actuators
            self.send_serial_command("1")
        elif msg.buttons[Button.ZR] == 1:  # Retract two big actuators
            self.send_serial_command("2")
        elif msg.buttons[Button.L] == 1:  # Extend small actuator
            self.send_serial_command("3")
        elif msg.buttons[Button.ZL] == 1:  # Retract small actuator
            self.send_serial_command("4")
        elif msg.buttons[Button.B] == 1:  # Stop action
            self.send_serial_command("0")
        elif msg.buttons[Button.A] == 1 and msg.buttons[Button.L] == 1 and msg.buttons[Button.R] == 1:  # Reservoir position
            self.send_serial_command("5")
        elif msg.buttons[Button.A] == 1 and msg.buttons[Button.R] == 1:  # Scoop 1
            self.send_serial_command("6")
        elif msg.buttons[Button.A] == 1 and msg.buttons[Button.ZR] == 1:  # Scoop 2
            self.send_serial_command("7")
        elif msg.buttons[Button.A] == 1 and msg.buttons[Button.L] == 1:  # Scoop 3
            self.send_serial_command("8")
        elif msg.buttons[Button.A] == 1 and msg.buttons[Button.ZL] == 1 and msg.buttons[Button.ZR] == 1:  # Dumping
            self.send_serial_command("9")

    # may be missing something here, it is hard too tell it may have been reduntent

    def send_serial_command(self, command):
        # Flush buffers before sending command
        self.serial_conn.reset_input_buffer()
        self.serial_conn.reset_output_buffer()

        # Send the command
        self.serial_conn.write(command.encode())
        self.get_logger().info(f"Sent serial command: {command}")

        # Add a small delay between commands
        time.sleep(0.002)

def main(args=None):
    rclpy.init(args=args)
    node = Joy2Cmd()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
