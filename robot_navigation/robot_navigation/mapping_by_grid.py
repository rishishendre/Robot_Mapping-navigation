import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import math
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
class Mapper(Node):
    def __init__(self):
        super().__init__('mapper')
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 30)
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 30)
        self.publisher_occupancy_grid = self.create_publisher(OccupancyGrid, '/map', 30)

        self.width = 70*2
        self.height = 70*2
        self.resolution = 0.04

        self.grid = np.full((self.width, self.height), -1)
        self.count_grid = np.full((self.width, self.height), 0)
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0
        self.count = 0
        self.count_threshold = 8  # number of hits required to mark a cell as occupied
        self.wx=0
        self.wy=0
        self.gx=0
        self.gy=0
        self.rx=0
        self.ry=0
        self.tf_broadcaster = TransformBroadcaster(self)

        # publish map→odom at 20Hz
        self.create_timer(0.05, self.publish_map_to_odom)

    def publish_map_to_odom(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'       # parent
        tf.child_frame_id = 'odom'       # child

        # where is odom origin relative to map?
        # if your robot starts at (0,0) and map origin is (0,0)
        # then this is just identity
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0   # no rotation

        self.tf_broadcaster.sendTransform(tf)    

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2 * (q.w*q.z + q.x*q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.robot_theta = math.atan2(siny, cosy)
    
    def xy_to_grid(self,x,y):
        self.wx = self.robot_x + x * math.cos(self.robot_theta) - y * math.sin(self.robot_theta)
        self.wy = self.robot_y + x * math.sin(self.robot_theta) + y * math.cos(self.robot_theta)
        
        self.gy = round(self.wx / self.resolution) + self.width // 2
        self.gx = round(self.wy / self.resolution) + self.height // 2


        self.ry = round(self.robot_x / self.resolution) + self.width // 2
        self.rx = round(self.robot_y / self.resolution) + self.height // 2
         
        if abs(self.gx) < self.width and abs(self.gy) < self.height and abs(self.rx) < self.width and abs(self.ry) < self.height: #grid andar ani chaiye
            self.count_grid[self.gx][self.gy] += 1
            self.grid[self.gx][self.gy]=100 if self.count_grid[self.gx][self.gy]>self.count_threshold else -1
            self.bresenham(self.rx,self.ry,self.gx,self.gy)
                   
    def bresenham(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy  
            self.count_grid[x0][y0] += 1   
            if self.count_grid[x0][y0]>self.count_threshold:   
                self.grid[x0][y0]=0 if self.grid[x0][y0]!=100 else 100      
        

    def scan_callback(self, msg):
        angle = msg.angle_min
        for r in msg.ranges:
            if r == float('inf'):
                angle += msg.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)
            self.xy_to_grid(x,y)
            angle += msg.angle_increment

        self.publish_map()

    def publish_map(self):
        msg = OccupancyGrid()
        msg.info.origin.position.x = -self.width * self.resolution / 2
        msg.info.origin.position.y = -self.height * self.resolution / 2
        msg.info.width = self.width
        msg.info.height = self.height
        msg.header.frame_id = "odom"
        msg.info.resolution = self.resolution

        msg.data = self.grid.flatten().tolist()
        self.publisher_occupancy_grid.publish(msg)


def main():
    rclpy.init()
    node = Mapper()
    rclpy.spin(node)
    rclpy.shutdown()