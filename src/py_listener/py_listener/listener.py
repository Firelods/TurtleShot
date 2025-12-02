import rclpy # the ros2 python client library
from rclpy.node import Node # A base class from rclpy that represents a ROS2 node
from std_msgs.msg import String # The message type we are subscribing to. In this case, the publisher (talker) is sending messages of type std_msgs/String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')

        self.declare_parameter('message_limit', 5)
        self.message_count = 0

        # Create a subscriber on topic 'chatter'
        self.subscription = self.create_subscription(
            String, # Message type
            'chatter', # Topic name
            self.listener_callback, # Callback function
            10 # QoS history depth
        )
        self.add_on_set_parameters_callback(self.parameter_callback)

    def listener_callback(self, msg):
        self.message_count += 1
        message_limit = self.get_parameter('message_limit').value
        self.get_logger().info(f"I heard: '{msg.data}'")

        if self.message_count >= message_limit:
            self.get_logger().info(f"Message limit reached: {message_limit}. Shutting down.")
            rclpy.shutdown()
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'message_limit' and param.type == Param.Type.INTEGER:
                self.get_logger().info(f"Parameter 'message_limit' changed to: {param.value}")
        return rclpy.parameter.SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS2 Python system
    node = Listener() # Create an instance of the Listener node
    rclpy.spin(node) # Keep the node running, listening for messages
    node.destroy_node() # Cleanup when the node is stopped
    rclpy.shutdown() # It cleans up all ROS2 resources used by the node

if __name__ == '__main__':
    main()