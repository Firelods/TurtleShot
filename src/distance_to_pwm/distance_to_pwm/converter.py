import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import time
import math

class CommandToPWM(Node):
    def __init__(self):
        super().__init__('command_to_pwm_converter')
        
        # Parameters
        self.declare_parameter('input_topic', '/motor_commands')
        self.declare_parameter('left_motor_topic', '/left_motor_pwm')
        self.declare_parameter('right_motor_topic', '/right_motor_pwm')
        self.declare_parameter('pwm_speed', 150) # Standard speed PWM
        self.declare_parameter('linear_speed', 0.5) # m/s at pwm_speed (calibration needed)
        self.declare_parameter('angular_speed', 1.0) # rad/s at pwm_speed (calibration needed)
        
        # Get params
        self.input_topic = self.get_parameter('input_topic').value
        self.left_topic = self.get_parameter('left_motor_topic').value
        self.right_topic = self.get_parameter('right_motor_topic').value
        self.pwm_speed = self.get_parameter('pwm_speed').value
        self.linear_k = self.get_parameter('linear_speed').value
        self.angular_k = self.get_parameter('angular_speed').value

        # Publishers / Subscribers
        self.subscription = self.create_subscription(
            String,
            self.input_topic,
            self.command_callback,
            10)
            
        self.left_pub = self.create_publisher(Int32, self.left_topic, 10)
        self.right_pub = self.create_publisher(Int32, self.right_topic, 10)
        
        # State
        self.current_pwm_left = 0
        self.current_pwm_right = 0
        self.stop_time = 0.0
        
        # Control Loop (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Command to PWM converter ready. Waiting for S{dist} or T{angle}...')

    def command_callback(self, msg):
        command = msg.data.strip()
        if not command:
            return
            
        cmd_type = command[0].upper()
        try:
            value = float(command[1:])
        except ValueError:
            self.get_logger().error(f'Invalid value in command: {command}')
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if cmd_type == 'S':
            # Straight: value is distance (meters)
            # time = dist / speed
            duration = abs(value) / self.linear_k
            
            # Direction
            direction = 1 if value >= 0 else -1
            
            self.current_pwm_left = int(self.pwm_speed * direction)
            self.current_pwm_right = int(self.pwm_speed * direction)
            self.stop_time = now + duration
            self.get_logger().info(f'Driving Straight: {value}m ({duration:.2f}s)')
            
        elif cmd_type == 'T':
            # time = rad_angle / ang_speed
            rads = math.radians(value)
            duration = abs(rads) / self.angular_k
            
            if value > 0: # Turn Left
                self.current_pwm_left = -int(self.pwm_speed)
                self.current_pwm_right = int(self.pwm_speed)
            else: # Turn Right
                self.current_pwm_left = int(self.pwm_speed)
                self.current_pwm_right = -int(self.pwm_speed)
                
            self.stop_time = now + duration
            self.get_logger().info(f'Turning: {value} deg ({duration:.2f}s)')
            
        else:
            self.get_logger().warn(f'Unknown command type: {cmd_type}')

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        
        if now >= self.stop_time:
            self.current_pwm_left = 0
            self.current_pwm_right = 0
            
        # Publish current state
        l_msg = Int32()
        l_msg.data = self.current_pwm_left
        self.left_pub.publish(l_msg)
        
        r_msg = Int32()
        r_msg.data = self.current_pwm_right
        self.right_pub.publish(r_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommandToPWM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
