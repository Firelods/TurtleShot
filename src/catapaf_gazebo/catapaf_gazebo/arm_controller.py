#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.srv = self.create_service(Trigger, 'trigger_arm', self.trigger_arm_callback)
        self.get_logger().info('Arm Controller Service Ready.')

    def trigger_arm_callback(self, request, response):
        self.get_logger().info('Trigger received. Activating arm...')
        
        # Simulate arm operation time
        time.sleep(2.0)
        
        self.get_logger().info('Arm action completed.')
        response.success = True
        response.message = "Arm triggered successfully"
        return response

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
