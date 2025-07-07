#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from scipy.optimize import least_squares

class CircleEnhancerNode:
    def __init__(self):
        rospy.init_node('circle_enhancer_node')

        self.sub = rospy.Subscriber("/flattened_slice", PointCloud2, self.pointcloud_callback)
        self.pub_strong = rospy.Publisher("/circle_strong", PointCloud2, queue_size=1)
        self.pub_line = rospy.Publisher("/circle_line", PointCloud2, queue_size=1)

        # 固定参数
        self.radius = 0.2285  # 圆半径（457mm / 2）
        self.tolerance = 0.02  # 半径误差（±2cm）
        self.min_support = 30  # 最少支持点数量
        self.resolution = 120  # 拟合圆输出点数

    def pointcloud_callback(self, msg):
        # 1. 提取xy点（z ≈ 0）
        raw_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points_xy = np.array([[x, y] for x, y, z in raw_points if abs(z) < 0.05])
        if len(points_xy) < 10:
            rospy.logwarn("点太少，跳过帧")
            return

        best_center = None
        best_support = []

        # 2. 滑窗查找圆形区域（固定半径）
        for i, p in enumerate(points_xy):
            dists = np.linalg.norm(points_xy - p, axis=1)
            support = points_xy[(dists > self.radius - self.tolerance) & (dists < self.radius + self.tolerance)]

            if len(support) > len(best_support):
                best_support = support
                best_center = p

        if best_center is None or len(best_support) < self.min_support:
            rospy.logwarn("未找到足够支持点的圆，跳过帧")
            return

        # 3. 拟合圆心（保持半径固定）
        def residuals(c, points):
            return np.linalg.norm(points - c, axis=1) - self.radius

        res = least_squares(residuals, x0=best_center, args=(best_support,))
        refined_center = res.x

        # 4. 生成圆形点
        theta = np.linspace(0, 2 * np.pi, self.resolution)
        x = refined_center[0] + self.radius * np.cos(theta)
        y = refined_center[1] + self.radius * np.sin(theta)
        z = np.zeros_like(x)
        enhanced_circle = list(zip(x, y, z))

        # 5. 提取非圆点（原始直线点）
        full_points_xy = np.array([[x, y] for x, y, z in raw_points])
        dists_to_circle = np.linalg.norm(full_points_xy - refined_center, axis=1)
        line_points = [
            (x, y, 0.0) for (x, y), d in zip(full_points_xy, dists_to_circle)
            if not (self.radius - self.tolerance < d < self.radius + self.tolerance)
        ]

        # 6. 发布两个话题
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id

        cloud_circle = pc2.create_cloud_xyz32(header, enhanced_circle)
        cloud_combined = pc2.create_cloud_xyz32(header, enhanced_circle + line_points)

        self.pub_strong.publish(cloud_circle)
        self.pub_line.publish(cloud_combined)


if __name__ == '__main__':
    try:
        node = CircleEnhancerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
