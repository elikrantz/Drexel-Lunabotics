import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Example for built-in message types (String)
from geometry_msgs.msg import Twist  # Example for built-in message types (Twist)
# Placeholder for custom message type
from ros_phoenix.msg import MotorControl  # Replace with your custom message type

class CmdPublisher(Node):
    def __init__(self):
        super().__init__('cmd_publisher')  # Node name
        self.publisher_ = self.create_publisher(String, '/talon_right/set', 10)  # Replace 'String' with custom type
        self.timer = self.create_timer(0.05, self.timer_callback)  # 50ms timer
        self.count_ = 0

    def timer_callback(self):
        # Create the message object
        message = MotorControl()  # Use this if you have the custom message
        # message = String()  # Example using a simple String message
        
        message.mode = 0
        message.value = 1.0
        # For demonstration, hardcoding some values
        # Replace these with values based on your custom message structure
        message.data = f'mode: {message.mode}, value: {message.value}'  # Customize according to the message type
        
        self.get_logger().info(f"Publishing: {message.data}")
        self.publisher_.publish(message)  # Publish the message

def main(args=None):
    rclpy.init(args=args)
    cmd_publisher = CmdPublisher()
    rclpy.spin(cmd_publisher)  # Keep the node running and handle callbacks
    cmd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
