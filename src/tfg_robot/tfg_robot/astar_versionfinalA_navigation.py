import math
import time
import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tfg_robot.sdf_to_grid import create_grid_from_world
from tfg_robot.astar_planner import astar
from tfg_robot.path_utils import path_to_world

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid

from datetime import datetime


def world_to_grid_node(x, y, world_size=30.0, resolution=0.2):
    origin_x = -world_size / 2.0
    origin_y = -world_size / 2.0

    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)

    return col, row


class AStarNavigationNode(Node):

    def __init__(self):
        super().__init__("astar_navigation_node")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/astar_path", 10)

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            "/astar_map",
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.spawn_x = -9.0
        self.spawn_y = -9.0

        self.world_path = []
        self.path_generated = False
        self.current_waypoint_index = 0
        self.goal_reached = False
        self.navigation_start_time = None
        self.navigation_end_time = None

        self.distance_tolerance = 0.1
        self.angle_tolerance = 0.08

        self.linear_speed = 1.2
        self.angular_speed = 1.5

        self.world_size = 30.0
        self.resolution = 0.2

        self.world_file = "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_medium.world"

        self.grid = create_grid_from_world(
           world_file=self.world_file,
           resolution=self.resolution,
           world_size=self.world_size
        )

        # Keep original grid for RViz2 visualization
        self.visual_grid = self.grid

# Use inflated grid only for A* planning
        self.planning_grid = self.inflate_obstacles(
           self.grid,
           inflation_radius=2
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.map_timer = self.create_timer(
            1.0,
            self.publish_map
        )

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x + self.spawn_x

        self.current_y = msg.pose.pose.position.y + self.spawn_y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def generate_path_from_robot_position(self):
        start = world_to_grid_node(
            self.current_x,
            self.current_y,
            self.world_size,
            self.resolution
        )

        goal = world_to_grid_node(
            9.0,
            9.0,
            self.world_size,
            self.resolution
        )

        self.get_logger().info(f"Robot world position: ({self.current_x:.2f}, {self.current_y:.2f})")
        self.get_logger().info(f"Grid start: {start}")
        self.get_logger().info(f"Grid goal: {goal}")
        self.get_logger().info("Path planning started.")

        self.get_logger().info("=================================")
        self.get_logger().info("Experiment ID: M3_ASTAR_VERSIONFinalA_RUN3")
        self.get_logger().info(f"Timestamp: {datetime.now().strftime('%Y-%m-%d')}")
        self.get_logger().info("Map: Medium")
        self.get_logger().info("Algorithm: A*")
        self.get_logger().info("Version: finalA")
        self.get_logger().info("=================================")

        start_time = time.perf_counter()

        path, visited_nodes = astar(self.planning_grid, start, goal)

        end_time = time.perf_counter()

        execution_time = end_time - start_time

        self.get_logger().info(f"Execution time: {execution_time:.6f} seconds")

        if path is None:
            self.get_logger().error("No path found.")
            self.world_path = []
            return
        
        self.get_logger().info("Path found.")

        self.world_path = path_to_world(
            path,
            world_size=self.world_size,
            resolution=self.resolution
        )
        
        


         # Reduce waypoints to make motion smoother
        self.world_path = self.reduce_waypoints_by_angle(
        self.world_path,
        angle_threshold=0.35,
        step=5
        )

        self.path_generated = True
        self.navigation_start_time = time.perf_counter()
        self.get_logger().info("Navigation started.")

        self.get_logger().info("A* path generated from real robot position.")
        self.get_logger().info(f"Visited nodes: {visited_nodes}")
        self.get_logger().info(f"Total waypoints: {len(self.world_path)}")
        self.publish_path()


    def publish_map(self):

        map_msg = OccupancyGrid()

        map_msg.header.frame_id = "map"
        map_msg.header.stamp = self.get_clock().now().to_msg()

        map_msg.info.resolution = self.resolution

        map_msg.info.width = self.grid.shape[1]
        map_msg.info.height = self.grid.shape[0]

        map_msg.info.origin.position.x = -self.world_size / 2.0
        map_msg.info.origin.position.y = -self.world_size / 2.0
        map_msg.info.origin.orientation.w = 1.0

        data = []

        rviz_grid = self.grid
        for row in rviz_grid:
                for cell in row:

                    if cell == 1:
                       data.append(100)
                    else:
                       data.append(0)

        map_msg.data = data

        self.map_pub.publish(map_msg)    


    def inflate_obstacles(self, grid, inflation_radius=0.005):
        inflated_grid = np.copy(grid)

        rows, cols = grid.shape

        for row in range(rows):
          for col in range(cols):

           if grid[row][col] == 1:

              for dr in range(-inflation_radius, inflation_radius + 1):
               for dc in range(-inflation_radius, inflation_radius + 1):

                new_row = row + dr
                new_col = col + dc

                if 0 <= new_row < rows and 0 <= new_col < cols:
                    distance = math.sqrt(dr * dr + dc * dc)

                    if distance <= inflation_radius:
                        inflated_grid[new_row][new_col] = 1

        return inflated_grid    
    
    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.world_path:
           pose = PoseStamped()
           pose.header.frame_id = "map"
           pose.header.stamp = self.get_clock().now().to_msg()

           pose.pose.position.x = x
           pose.pose.position.y = y
           pose.pose.position.z = 0.0
           pose.pose.orientation.w = 1.0

           path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


    def reduce_waypoints_by_angle(self, path, angle_threshold=0.35, step=5):
      if len(path) <= 2:
        return path

      reduced_path = [path[0]]

      counter = 0

      for i in range(1, len(path) - 1):
        x1, y1 = path[i - 1]
        x2, y2 = path[i]
        x3, y3 = path[i + 1]

        angle1 = math.atan2(y2 - y1, x2 - x1)
        angle2 = math.atan2(y3 - y2, x3 - x2)

        angle_change = abs(self.normalize_angle(angle2 - angle1))

        counter += 1

        if angle_change > angle_threshold:
            reduced_path.append(path[i])
            counter = 0
        elif counter >= step:
            reduced_path.append(path[i])
            counter = 0

      reduced_path.append(path[-1])

      return reduced_path


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def control_loop(self):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        if not self.path_generated:
            self.generate_path_from_robot_position()
            return

        if len(self.world_path) == 0:
            self.stop_robot()
            return

        if self.current_waypoint_index >= len(self.world_path):

            if not self.goal_reached:
               self.goal_reached = True
               self.navigation_end_time = time.perf_counter()

               navigation_time = self.navigation_end_time - self.navigation_start_time

               self.stop_robot()

               self.get_logger().info("=================================")
               self.get_logger().info("RESULT: SUCCESS")
               self.get_logger().info("Robot moved successfully: YES")
               self.get_logger().info(f"Navigation time: {navigation_time:.3f} seconds")
               self.get_logger().info("Goal reached.")
               self.get_logger().info("=================================")

            return   

        target_x, target_y = self.world_path[self.current_waypoint_index]

        dx = target_x - self.current_x
        dy = target_y - self.current_y

        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_yaw)

        cmd = Twist()

        if distance < self.distance_tolerance:
            self.current_waypoint_index += 1
            return

        if abs(angle_error) > self.angle_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.9 * angle_error

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = AStarNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()