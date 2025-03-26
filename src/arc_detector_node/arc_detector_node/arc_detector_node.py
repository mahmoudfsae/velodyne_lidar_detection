#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import math

class ArcDetectorNode(Node):
    def __init__(self):
        super().__init__('arc_detector_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/filtered_points',
            self.pointcloud_callback,
            10)
        self.get_logger().info('ArcDetectorNode has started.')

    def pointcloud_callback(self, msg):
        points = list(point_cloud2.read_points(msg, skip_nans=True))
        fields = [f.name for f in msg.fields]
        ring_index = fields.index('ring')

        # Group points into 16 vertical sheets by ring
        sheets = {i: [] for i in range(16)}
        for pt in points:
            ring = int(pt[ring_index])
            sheets[ring].append(pt)

        # Slide over each sheet and check for arcs
        window_size = 5
        min_radius = 0.1
        max_error = 0.05

        for ring_id, sheet in sheets.items():
            if len(sheet) < window_size:
                continue

            for i in range(len(sheet) - window_size + 1):
                window = sheet[i:i+window_size]
                arc_points = [(pt[0], pt[1]) for pt in window]

                center, radius, error = self.fit_circle(arc_points)

                if error < max_error and radius >= min_radius:
                    self.get_logger().info(
                        f'✅ Ring {ring_id} arc detected! '
                        f'Center: ({center[0]:.2f}, {center[1]:.2f}), '
                        f'Radius: {radius:.2f}, Error: {error:.5f}'
                    )

    def fit_circle(self, points):
        x = np.array([p[0] for p in points])
        y = np.array([p[1] for p in points])

        A = np.c_[2*x, 2*y, np.ones(len(points))]
        b = x**2 + y**2

        try:
            sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            cx, cy, c = sol
            radius = math.sqrt(c + cx**2 + cy**2)
            center = (cx, cy)

            errors = [abs(math.hypot(px - cx, py - cy) - radius) for px, py in points]
            avg_error = sum(errors) / len(errors)

            return center, radius, avg_error

        except Exception as e:
            self.get_logger().warn(f'Circle fit failed: {e}')
            return (0, 0), 0, float('inf')

def main(args=None):
    rclpy.init(args=args)
    node = ArcDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

