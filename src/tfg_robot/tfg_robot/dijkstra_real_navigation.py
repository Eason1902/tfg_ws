import math
import time
import rclpy
import numpy as np
from PIL import Image

from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from datetime import datetime
from tfg_robot.dijkstra_planner import dijkstra


def world_to_grid_node(x, y, origin_x=-3.343, origin_y=-1.609, resolution=0.05):
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)

    return col, row


def grid_to_world_node(col, row, origin_x=-3.343, origin_y=-1.609, resolution=0.05):
    x = col * resolution + origin_x
    y = row * resolution + origin_y

    return x, y


class DijkstraNavigationNode(Node):

    def __init__(self):
        super().__init__("dijkstra_navigation_node")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/dijkstra_path", 10)

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            "/dijkstra_map",
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

        self.spawn_x = 0.0
        self.spawn_y = 0.0

        self.world_path = []
        self.path_generated = False
        self.current_waypoint_index = 0
        self.goal_reached = False
        self.navigation_start_time = None
        self.navigation_end_time = None

        self.distance_tolerance = 0.15
        self.angle_tolerance = 0.15

        self.max_linear_speed = 0.15
        self.medium_linear_speed = 0.08
        self.min_linear_speed = 0.04

        self.angular_speed = 0.2

        self.map_width = 135
        self.map_height = 113

        self.origin_x = -3.343
        self.origin_y = -1.609
        self.resolution = 0.05
        self.safety_distance = 0.4

        self.map_file = "/home/yilun/tfg_ws/src/tfg_robot/maps/mymap.pgm"

        img = Image.open(self.map_file).convert("L")
        map_array = np.array(img)

        self.grid = np.zeros_like(map_array)

        self.grid[map_array < 50] = 1
        self.grid[map_array >= 50] = 0

       # Keep original grid for RViz2 visualization
        self.visual_grid = self.grid

        # Use inflated grid only for Dijkstra planning
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
           self.origin_x,
           self.origin_y,
           self.resolution
        )

    
        goal = world_to_grid_node(
            -2.2,
            3.0,
            self.origin_x,
            self.origin_y,
            self.resolution
        )

        self.get_logger().info(f"Robot world position: ({self.current_x:.2f}, {self.current_y:.2f})")
        self.get_logger().info(f"Grid start: {start}")
        self.get_logger().info(f"Grid goal: {goal}")

        self.get_logger().info("Path planning started.")
     
        self.get_logger().info("=================================")
        self.get_logger().info("Experiment ID: M3_DIJKSTRA_VersionFinal_RUN1")
        self.get_logger().info(f"Timestamp: {datetime.now().strftime('%Y-%m-%d')}")
        self.get_logger().info("Map: Simple")
        self.get_logger().info("Algorithm: Dijkstra")
        self.get_logger().info("Version: VersionFinal")
        self.get_logger().info("=================================")

        start_time = time.perf_counter()

        path, visited_nodes = dijkstra(self.planning_grid, start, goal)

        end_time = time.perf_counter()
        execution_time = end_time - start_time

        self.get_logger().info(f"Execution time: {execution_time:.6f} seconds")

        if path is None:
            self.get_logger().error("No path found.")
            self.world_path = []
            return

        self.world_path = [
            grid_to_world_node(
                col,
                row,
                self.origin_x,
                self.origin_y,
                self.resolution
            )
            for col, row in path
        ]

        # Reduce waypoints to make motion smoother
        self.world_path = self.reduce_waypoints_dynamic(self.world_path)
        
        self.get_logger().info("Path found.")
        self.path_generated = True
        self.navigation_start_time = time.perf_counter()
        self.get_logger().info("Navigation started.")
        self.get_logger().info("Dijkstra path generated from real robot position.")
        self.get_logger().info(f"Visited nodes: {visited_nodes}")
        self.get_logger().info(f"Total waypoints: {len(self.world_path)}")

        self.publish_path()

    def publish_map(self):
        map_msg = OccupancyGrid()

        map_msg.header.frame_id = "odom"
        map_msg.header.stamp = self.get_clock().now().to_msg()

        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.grid.shape[1]
        map_msg.info.height = self.grid.shape[0]

        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
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

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.world_path:
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def inflate_obstacles(self, grid, inflation_radius=2):
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

    def reduce_waypoints_dynamic(self, path):
      if len(path) <= 2:
        return path

      reduced_path = [path[0]]

      for i in range(1, len(path) - 1):
        prev_x, prev_y = path[i - 1]
        curr_x, curr_y = path[i]
        next_x, next_y = path[i + 1]

        angle1 = math.atan2(curr_y - prev_y, curr_x - prev_x)
        angle2 = math.atan2(next_y - curr_y, next_x - curr_x)

        angle_diff = abs(self.normalize_angle(angle2 - angle1))

        if angle_diff > 0.20:
            reduced_path.append(path[i])
        elif i % 5 == 0:
            reduced_path.append(path[i])

      reduced_path.append(path[-1])

      return reduced_path

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def get_adaptive_speed(self, angle_error):
      abs_error = abs(angle_error)

      if abs_error > 0.50:
        return self.min_linear_speed
      elif abs_error > 0.25:
        return self.medium_linear_speed
      else:
        return self.max_linear_speed

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

        if abs(angle_error) > 0.80:
           cmd.linear.x = 0.0
           cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
        else:
           adaptive_speed = self.get_adaptive_speed(angle_error)

           cmd.linear.x = adaptive_speed
           cmd.angular.z = 0.5 * angle_error

        self.cmd_pub.publish(cmd)
    
    


    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = DijkstraNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()