import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

import numpy as np
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose   # reuse nav2 action type

from .astar_path import AStarPlanner
from .cmd_vel import PurePursuitController
from rclpy.time import Time
from rclpy.duration import Duration

class MyNavigator(Node):
    def __init__(self):
        super().__init__('my_navigator')

        # TF listener — to get robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Map subscriber
        self.map = None
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)

        # cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Path publisher (so you can visualize in RViz)
        self.path_pub = self.create_publisher(Path, '/my_path', 10)

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

        self.controller = PurePursuitController(
            lookahead_dist=0.5,
            max_linear_vel=0.2,
            max_angular_vel=1.0
        )

        self.get_logger().info('MyNavigator ready!')

    def map_callback(self, msg):
        # Convert flat OccupancyGrid data → 2D numpy array
        data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map = data
        self.map_info = msg.info

    def get_robot_pose(self):
        """Returns (x, y, yaw) in map frame"""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                    Time(),                        # time(0) = latest
                    timeout=Duration(seconds=1.0))

            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return x, y, yaw

        except Exception as e:
            self.get_logger().warn(f'TF error: {e}')
            return None

    def publish_path(self, path_coords):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in path_coords:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = wx
            p.pose.position.y = wy
            msg.poses.append(p)
        self.path_pub.publish(msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Goal received!')

        goal_pose = goal_handle.request.pose.pose
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y

        # --- Wait for map ---
        while self.map is None:
            self.get_logger().info('Waiting for map...')
            rclpy.spin_once(self, timeout_sec=0.5)

        # --- Get robot pose ---
        pose = self.get_robot_pose()
        if pose is None:
            goal_handle.abort()
            return NavigateToPose.Result()

        robot_x, robot_y, robot_yaw = pose

        # --- Plan path with A* ---
        self.get_logger().info(f'Planning from ({robot_x:.2f},{robot_y:.2f}) to ({goal_x:.2f},{goal_y:.2f})')

        planner = AStarPlanner(
            occupancy_grid=self.map,
            resolution=self.map_info.resolution,
            origin=(self.map_info.origin.position.x,
                    self.map_info.origin.position.y)
        )

        path = planner.plan(
            start_world=(robot_x, robot_y),
            goal_world=(goal_x, goal_y)
        )

        if path is None:
            self.get_logger().error('No path found!')
            goal_handle.abort()
            return NavigateToPose.Result()

        self.get_logger().info(f'Path found! {len(path)} waypoints')
        self.publish_path(path)

        # --- Follow path with Pure Pursuit ---
        rate = self.create_rate(20)  # 20Hz control loop

        while rclpy.ok():
            pose = self.get_robot_pose()
            if pose is None:
                continue

            robot_x, robot_y, robot_yaw = pose

            # Check goal reached
            dist = np.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
            if dist < 0.1:
                self.get_logger().info('Goal reached!')
                break

            # Compute velocity
            linear, angular = self.controller.compute_velocity(
                robot_x, robot_y, robot_yaw, path
            )

            # Publish cmd_vel
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.cmd_pub.publish(twist)

            # Send feedback
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = dist
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        # Stop robot
        self.cmd_pub.publish(Twist())
        goal_handle.succeed()
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = MyNavigator()
    rclpy.spin(node)
    rclpy.shutdown()    