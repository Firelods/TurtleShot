#!/usr/bin/env python3
"""
Ball Spawner Node

This node spawns projectile balls in the catapaf arm for testing physics.
It automatically spawns a ball when the simulation starts and can respawn
balls after each launch.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
import os
import subprocess
import tempfile
import time
from ament_index_python.packages import get_package_share_directory


class BallSpawner(Node):
    """Spawns and manages projectile balls for the catapaf."""

    def __init__(self):
        super().__init__('ball_spawner')

        # Parameters
        self.declare_parameter('auto_respawn', False)
        self.declare_parameter('spawn_delay', 2.0)  # Delay before initial spawn
        self.declare_parameter('ball_x', 0.40)  # Relative to robot base
        self.declare_parameter('ball_y', -0.098)
        self.declare_parameter('ball_z', 0.20)  # Above the catapaf arm

        self.auto_respawn = self.get_parameter('auto_respawn').value
        self.spawn_delay = self.get_parameter('spawn_delay').value
        self.ball_x = self.get_parameter('ball_x').value
        self.ball_y = self.get_parameter('ball_y').value
        self.ball_z = self.get_parameter('ball_z').value

        self.ball_counter = 0
        self.spawn_in_progress = False

        # Service to manually spawn a ball
        self.spawn_srv = self.create_service(
            Trigger,
            'ball_spawner/spawn',
            self.spawn_callback
        )

        # Get the ball model SDF file path
        pkg_catapaf_gazebo = get_package_share_directory('catapaf_gazebo')
        self.ball_sdf_path = os.path.join(pkg_catapaf_gazebo, 'models', 'ball', 'model.sdf')

        # Read the SDF file content
        with open(self.ball_sdf_path, 'r') as f:
            self.ball_sdf = f.read()

        # Timer for initial spawn
        self.create_timer(self.spawn_delay, self.initial_spawn_callback)

        self.get_logger().info('Ball Spawner initialized')
        self.get_logger().info(f'Auto-respawn: {self.auto_respawn}')
        self.get_logger().info(f'Ball position: x={self.ball_x}, y={self.ball_y}, z={self.ball_z}')
        self.get_logger().info('Call service: ros2 service call /ball_spawner/spawn std_srvs/srv/Trigger')

    def initial_spawn_callback(self):
        """Spawn the first ball after a delay."""
        self.get_logger().info('Spawning initial ball...')
        self.spawn_ball()
        # Cancel the timer so it only runs once
        self.destroy_timer(self.create_timer(0.0, lambda: None))

    def spawn_ball(self):
        """Spawn a ball using gz service command."""
        if self.spawn_in_progress:
            self.get_logger().warn('Spawn already in progress, skipping')
            return False

        self.spawn_in_progress = True
        self.ball_counter += 1
        ball_name = f'ball_{self.ball_counter}'

        temp_file = None
        try:
            # Prepare the SDF with unique name and position
            ball_sdf = self.ball_sdf.replace('name="catapaf_ball"', f'name="{ball_name}"')
            # Update the pose in the SDF
            ball_sdf = ball_sdf.replace(
                '<pose>0 0 0.15 0 0 0</pose>',
                f'<pose>{self.ball_x} {self.ball_y} {self.ball_z} 0 0 0</pose>'
            )

            # Write SDF to a temporary file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as temp_file:
                temp_file.write(ball_sdf)
                temp_sdf_path = temp_file.name

            self.get_logger().info(f'Spawning {ball_name} at ({self.ball_x}, {self.ball_y}, {self.ball_z})')
            self.get_logger().debug(f'Temporary SDF file: {temp_sdf_path}')

            # Build the gz service request with sdf_filename instead of sdf content
            # Note: fields are at root level, not wrapped in "entity_factory"
            gz_request = (
                f'name: "{ball_name}" '
                f'allow_renaming: true '
                f'sdf_filename: "{temp_sdf_path}"'
            )

            # Call gz service directly
            cmd = [
                'gz', 'service',
                '-s', '/world/default/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', gz_request
            ]

            self.get_logger().debug(f'Running command: {" ".join(cmd)}')

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=10.0
            )

            # Wait a bit for Gazebo to read the file before cleaning up
            # Gazebo needs time to process the file asynchronously
            time.sleep(0.5)

            # Check if the command succeeded
            stdout = result.stdout.strip()
            stderr = result.stderr.strip()

            self.get_logger().info(f'Return code: {result.returncode}')
            self.get_logger().info(f'Stdout length: {len(stdout)} bytes')
            self.get_logger().info(f'Stdout: [{stdout}]')
            self.get_logger().info(f'Stderr: [{stderr}]')

            # Check for common error patterns
            error_patterns = [
                'Unable to find service',
                'Service call timed out',
                'Error',
                'error',
                'failed',
                'Failed',
                'not available'
            ]

            has_error = any(pattern in stdout or pattern in stderr for pattern in error_patterns)

            # gz service returns "data: true\n" on success (with newline)
            # Be very strict: must contain "data: true" specifically
            has_success = 'data: true' in stdout.lower()

            self.get_logger().info(f'Has error: {has_error}, Has success: {has_success}')

            # Clean up temp file after Gazebo has processed it
            try:
                if os.path.exists(temp_sdf_path):
                    os.remove(temp_sdf_path)
                    self.get_logger().debug(f'Cleaned up temp file: {temp_sdf_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to clean up temp file: {e}')

            if result.returncode == 0 and has_success and not has_error and len(stdout) > 0:
                self.get_logger().info(f'✓ Successfully spawned {ball_name}')
                self.spawn_in_progress = False
                return True
            else:
                self.get_logger().error(f'Failed to spawn ball')
                if stderr:
                    self.get_logger().error(f'Error: {stderr}')
                if not has_success:
                    self.get_logger().error('No success confirmation (data: true) received from Gazebo')
                if len(stdout) == 0:
                    self.get_logger().error('Empty response from gz service - is Gazebo running?')
                self.spawn_in_progress = False
                return False

        except subprocess.TimeoutExpired:
            self.get_logger().error('Spawn command timed out')
            self.spawn_in_progress = False
            return False
        except Exception as e:
            self.get_logger().error(f'Exception during spawn: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.spawn_in_progress = False
            return False
        finally:
            # Ensure temp file is cleaned up in case of exception
            if temp_file and hasattr(temp_file, 'name') and os.path.exists(temp_file.name):
                try:
                    os.remove(temp_file.name)
                    self.get_logger().debug(f'Cleaned up temp file in finally: {temp_file.name}')
                except Exception as e:
                    self.get_logger().debug(f'Could not clean up temp file in finally: {e}')

    def spawn_callback(self, request, response):
        """Service callback to manually spawn a ball."""
        self.get_logger().info('Manual ball spawn requested')

        success = self.spawn_ball()

        if success:
            response.success = True
            response.message = f'Ball {self.ball_counter} spawned successfully'
        else:
            response.success = False
            response.message = 'Failed to spawn ball'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = BallSpawner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
