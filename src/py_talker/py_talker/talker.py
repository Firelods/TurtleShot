import rclpy # the ros2 python client library
from rclpy.node import Node # A base class from rclpy that represents a ROS2 node
from std_msgs.msg import String # The message type we are subscribing to. In this case, the publisher (talker) is sending messages of type std_msgs/String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, world! {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
        
def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()