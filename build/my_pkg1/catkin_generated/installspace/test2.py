#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def pointcloud2_to_xyz_array(cloud_msg):
    points = []
    for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])
    return np.array(points, dtype=np.float32)

def xyz_array_to_pointcloud2(points, header):
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    return pc2.create_cloud(header, fields, points)

def fit_circle_ransac_2d(points, max_iterations=1000, distance_threshold=0.01):
    best_inliers = []
    best_circle = None

    xy = points[:, :2]

    for _ in range(max_iterations):
        sample = xy[np.random.choice(xy.shape[0], 3, replace=False)]
        A, B, C = sample

        def calc_circle(a, b, c):
            temp = b - a
            temp2 = c - a
            d = 2 * (temp[0] * temp2[1] - temp2[0] * temp[1])
            if abs(d) < 1e-6:
                return None, None
            ux = ((temp2[1]*(np.dot(temp, temp)) - temp[1]*(np.dot(temp2, temp2))) / d)
            uy = ((temp[0]*(np.dot(temp2, temp2)) - temp2[0]*(np.dot(temp, temp))) / d)
            center = a + np.array([ux, uy])
            radius = np.linalg.norm(center - a)
            return center, radius

        center, radius = calc_circle(A, B, C)
        if center is None or radius > 10.0 or radius < 0.01:
            continue

        dists = np.abs(np.linalg.norm(xy - center, axis=1) - radius)
        inliers = np.where(dists < distance_threshold)[0]

        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_circle = (center, radius)

    if best_circle is None:
        return None, None, None

    circle_points = points[best_inliers]
    return circle_points, best_circle[0], best_circle[1]

class CircleDetector:
    def __init__(self):
        rospy.init_node('circle_detector')
        self.circle_pub = rospy.Publisher('/object', PointCloud2, queue_size=1)
        self.center_pub = rospy.Publisher('/object_center', Point, queue_size=1)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        rospy.Subscriber('/flattened_slice', PointCloud2, self.cloud_cb)
        rospy.loginfo("Circle detector node started.")
        rospy.spin()

    def cloud_cb(self, msg):
        points = pointcloud2_to_xyz_array(msg)
        if points.shape[0] < 10:
            rospy.logwarn("点云数量太少，无法拟合圆")
            return

        circle_points, center, radius = fit_circle_ransac_2d(points)
        if circle_points is None:
            rospy.logwarn("未检测到圆")
            return

        pc2_msg = xyz_array_to_pointcloud2(circle_points, msg.header)
        self.circle_pub.publish(pc2_msg)

        pt = Point()
        pt.x, pt.y, pt.z = center[0], center[1], 0.0
        self.center_pub.publish(pt)

        # 发布 Marker 用于 RViz 可视化圆心
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "circle_center"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = pt
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        rospy.loginfo("圆心: (%.3f, %.3f), 半径: %.3f, 点数: %d" % (pt.x, pt.y, radius, len(circle_points)))

if __name__ == '__main__':
    try:
        CircleDetector()
    except rospy.ROSInterruptException:
        pass
