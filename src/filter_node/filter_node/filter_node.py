#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pointcloud_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)
        self.get_logger().info('FilterNode with distance filter started.')

    def pointcloud_callback(self, msg):
        # Convert raw PointCloud2 to list of points with all available fields
        cloud_points = list(point_cloud2.read_points(msg, skip_nans=True))

        # Filter: only keep points within a certain distance range
        min_range = 0.5  # meters
        max_range = 20.0  # meters

        filtered_points = []
        for pt in cloud_points:
            x, y, z = pt[0], pt[1], pt[2]
            distance = (x**2 + y**2 + z**2)**0.5
            if min_range < distance < max_range:
                filtered_points.append(pt)  # Keep the full point with all fields

        # Create new PointCloud2 message with preserved fields
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        filtered_msg = point_cloud2.create_cloud(header, msg.fields, filtered_points)
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

