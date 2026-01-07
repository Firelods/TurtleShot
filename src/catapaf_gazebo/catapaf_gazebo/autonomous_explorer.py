#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import numpy as np
import scipy.ndimage
import math
import time

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.navigator = BasicNavigator()
        
        # Subscribe to map
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
            
        self.latest_map = None
        
        # Timer for the main control loop
        self.timer = self.create_timer(1.0, self.control_loop)
        
        # Exploration variables
        self.min_frontier_size = 10 # Minimum number of cells to be considered a valid frontier
        self.goal_handle = None
        
        self.get_logger().info('Frontier Explorer Node Started. Waiting for Nav2...')

        # Wait for Nav2 to be fully active
        # We do this in the loop to avoid blocking init
        self.nav2_active = False

    def map_callback(self, msg):
        self.latest_map = msg

    def get_frontiers(self, map_msg):
        """
        Find frontiers in the occupancy grid.
        Returns a list of (x, y) coordinates in world frame.
        """
        # Map metadata
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        
        # Convert to numpy array
        # Data is row-major 1D array. Reshape to (height, width)
        grid = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
        
        # Constants
        UNKNOWN = -1
        FREE = 0
        OCCUPIED = 100
        
        # 1. Find free cells (value 0)
        free_mask = (grid == FREE)
        
        # 2. Find unknown cells (value -1)
        unknown_mask = (grid == UNKNOWN)
        
        # 3. Find edges: Free cells adjacent to Unknown cells
        # We can dilate the unknown mask by 1 pixel (connectivity 4 or 8)
        # If a free cell overlaps with the dilated unknown mask, it's a frontier.
        
        structure = scipy.ndimage.generate_binary_structure(2, 2) # 8-connectivity
        dilated_unknown = scipy.ndimage.binary_dilation(unknown_mask, structure=structure)
        
        frontier_mask = free_mask & dilated_unknown
        
        # 4. Cluster frontiers to find distinct regions
        labeled_array, num_features = scipy.ndimage.label(frontier_mask, structure=structure)
        
        if num_features == 0:
            return None
            
        # 5. Extract centroids of valid (large enough) clusters
        frontiers = []
        
        # Get sizes of components
        # bincount works on 1D, so flatten
        sizes = np.bincount(labeled_array.ravel())
        
        # Labels start from 1. 0 is background.
        for label_idx in range(1, num_features + 1):
            if sizes[label_idx] < self.min_frontier_size:
                continue
                
            # Find indices of this cluster
            # Note: where returns (row_indices, col_indices) -> (y, x)
            y_indices, x_indices = np.where(labeled_array == label_idx)
            
            # Compute centroid
            centroid_y = np.mean(y_indices)
            centroid_x = np.mean(x_indices)
            
            # Convert to world coordinates
            world_x = origin_x + (centroid_x + 0.5) * resolution
            world_y = origin_y + (centroid_y + 0.5) * resolution
            
            frontiers.append((world_x, world_y))
            
        return frontiers

    def select_best_frontier(self, frontiers):
        """
        Select the best frontier from the list.
        Currently: Closest Euclidean distance to current robot pose (if known) 
        or just the first one.
        """
        if not frontiers:
            return None
            
        # For simplicity, currently just picking the first one found (top-left ish usually)
        # or random to ensure coverage variance
        # Better: use BasicNavigator to get robot pose and find closest
        
        # Let's try to get feedback from navigator if possible, else just pick random for coverage
        import random
        return random.choice(frontiers)

    def control_loop(self):
        # 0. Ensure Nav2 is active
        if not self.nav2_active:
            # Check if lifecycle nodes are active. 
            # BasicNavigator doesn't expose a simple "is_active" check that is non-blocking 
            # but we can try getting the clock or assuming user launched it.
            # We'll rely on the user having launched nav2.
            # Ideally: self.navigator.waitUntilNav2Active() 
            # But that blocks. We want to be async.
            pass
        
        # 1. Check current task status
        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                 # self.get_logger().info('Moving to frontier...')
                 pass
            return

        # 2. Check execution result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Frontier reached!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Task canceled')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Task failed - blocked?')
            
        # 3. Plan next move
        if self.latest_map is None:
            self.get_logger().info('Waiting for map data...')
            return
            
        self.get_logger().info('Processing map for new frontiers...')
        
        try:
            frontiers = self.get_frontiers(self.latest_map)
        except Exception as e:
            self.get_logger().error(f'Error finding frontiers: {e}')
            return

        if not frontiers:
            self.get_logger().info('No new frontiers found. Exploration complete?')
            return
            
        self.get_logger().info(f'Found {len(frontiers)} potential frontiers.')
        
        target = self.select_best_frontier(frontiers)
        
        if target:
            tx, ty = target
            self.get_logger().info(f'Navigating to frontier at ({tx:.2f}, {ty:.2f})')
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = tx
            goal_pose.pose.position.y = ty
            goal_pose.pose.orientation.w = 1.0 # Forward orientation
            
            self.navigator.goToPose(goal_pose)


def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
