#!/usr/bin/env python3
"""
Catapaf Arm Controller Node (Gazebo JointController – Force Mode)

Controls the catapult arm by sending a velocity command
to a Gazebo JointController running in force mode.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class CatapafArmController(Node):

    def __init__(self):
        super().__init__('catapaf_arm_controller')

        # Parameters
        self.declare_parameter('launch_velocity', -40.0)
        self.declare_parameter('impulse_duration', 0.3)
        self.declare_parameter('return_velocity', 15.0)
        self.declare_parameter('return_duration', 0.2)
        self.declare_parameter('return_delay', 2.0)

        # Optional feedback-based stop thresholds (radians)
        self.declare_parameter('joint_name', 'catapaf_joint')
        self.declare_parameter('launch_position', -1.45)
        self.declare_parameter('rest_position', -0.05)
        self.declare_parameter('control_rate', 100.0)

        self.launch_velocity = self.get_parameter('launch_velocity').value
        self.impulse_duration = self.get_parameter('impulse_duration').value
        self.return_velocity = self.get_parameter('return_velocity').value
        self.return_duration = self.get_parameter('return_duration').value
        self.return_delay = self.get_parameter('return_delay').value

        self.joint_name = self.get_parameter('joint_name').value
        self.launch_position = self.get_parameter('launch_position').value
        self.rest_position = self.get_parameter('rest_position').value
        self.control_rate = float(self.get_parameter('control_rate').value)

        # Publisher → JointController (velocity command)
        self.cmd_pub = self.create_publisher(
            Float64,
            'catapaf_arm/cmd_vel',
            10
        )

        # Services
        self.launch_srv = self.create_service(
            Trigger,
            'catapaf_arm/launch',
            self.launch_callback
        )

        self.reset_srv = self.create_service(
            Trigger,
            'catapaf_arm/reset',
            self.reset_callback
        )

        self.is_launching = False

        self._joint_position = None
        self._joint_velocity = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self._joint_state_callback,
            10,
        )

        self._impulse_timer = None
        self._return_delay_timer = None
        self._return_timer = None

        self._impulse_tick_timer = None
        self._return_tick_timer = None
        self._phase_start_time = None

        self.get_logger().info('Catapaf Arm Controller (Force Mode) ready')
        self.get_logger().info(
            'Call: ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger'
        )

    # ----------------------------------------------------
    # Helpers
    # ----------------------------------------------------

    def publish_velocity(self, value: float):
        msg = Float64()
        msg.data = value
        self.cmd_pub.publish(msg)

    def _cancel_timer(self, timer):
        if timer is None:
            return None
        try:
            timer.cancel()
        except Exception:
            pass
        return None

    def _create_one_shot_timer(self, delay_sec: float, cb):
        timer_holder = {'timer': None}

        def _wrapper():
            timer = timer_holder['timer']
            if timer is not None:
                timer.cancel()
            cb()

        timer_holder['timer'] = self.create_timer(delay_sec, _wrapper)
        return timer_holder['timer']

    # ----------------------------------------------------
    # Callbacks
    # ----------------------------------------------------

    def _joint_state_callback(self, msg: JointState):
        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            return

        if idx < len(msg.position):
            self._joint_position = msg.position[idx]
        if idx < len(msg.velocity):
            self._joint_velocity = msg.velocity[idx]

    def launch_callback(self, request, response):
        if self.is_launching:
            response.success = False
            response.message = 'Launch already in progress'
            return response

        self._impulse_timer = self._cancel_timer(self._impulse_timer)
        self._return_delay_timer = self._cancel_timer(self._return_delay_timer)
        self._return_timer = self._cancel_timer(self._return_timer)
        self._impulse_tick_timer = self._cancel_timer(self._impulse_tick_timer)
        self._return_tick_timer = self._cancel_timer(self._return_tick_timer)

        self.is_launching = True
        self.get_logger().info('🚀 Launching catapult')

        # Step 1: impulsion
        self.publish_velocity(self.launch_velocity)

        self._phase_start_time = self.get_clock().now()
        tick_period = 1.0 / max(self.control_rate, 1.0)
        self._impulse_tick_timer = self.create_timer(tick_period, self._impulse_tick)

        self._impulse_timer = self._create_one_shot_timer(
            self.impulse_duration,
            self.stop_impulse
        )

        response.success = True
        response.message = 'Launch triggered'
        return response

    def _impulse_tick(self):
        if not self.is_launching:
            self._impulse_tick_timer = self._cancel_timer(self._impulse_tick_timer)
            return

        if self._joint_position is not None and self._joint_position <= float(self.launch_position):
            self.stop_impulse()

    def stop_impulse(self):
        if not self.is_launching:
            return

        self.publish_velocity(0.0)

        self._impulse_timer = None
        self._impulse_tick_timer = self._cancel_timer(self._impulse_tick_timer)

        # Step 2: return to rest after delay
        self._return_delay_timer = self._create_one_shot_timer(
            self.return_delay,
            self.return_to_rest
        )

    def return_to_rest(self):
        if not self.is_launching:
            return

        self._return_delay_timer = None
        self.get_logger().info('Returning arm to rest position')
        self.publish_velocity(self.return_velocity)

        self._phase_start_time = self.get_clock().now()
        tick_period = 1.0 / max(self.control_rate, 1.0)
        self._return_tick_timer = self.create_timer(tick_period, self._return_tick)

        self._return_timer = self._create_one_shot_timer(
            self.return_duration,
            self.finish_return
        )

    def _return_tick(self):
        if not self.is_launching:
            self._return_tick_timer = self._cancel_timer(self._return_tick_timer)
            return

        if self._joint_position is not None and self._joint_position >= float(self.rest_position):
            self.finish_return()

    def finish_return(self):
        if not self.is_launching:
            return

        self.publish_velocity(0.0)
        self._return_timer = None
        self._return_tick_timer = self._cancel_timer(self._return_tick_timer)
        self.is_launching = False

    def reset_callback(self, request, response):
        self.get_logger().info('Resetting arm')
        self._impulse_timer = self._cancel_timer(self._impulse_timer)
        self._return_delay_timer = self._cancel_timer(self._return_delay_timer)
        self._return_timer = self._cancel_timer(self._return_timer)
        self._impulse_tick_timer = self._cancel_timer(self._impulse_tick_timer)
        self._return_tick_timer = self._cancel_timer(self._return_tick_timer)
        self.publish_velocity(0.0)
        self.is_launching = False

        response.success = True
        response.message = 'Arm reset'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CatapafArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
