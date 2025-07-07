#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
import numpy as np
from sklearn.cluster import DBSCAN

class HoopDetector:
    def __init__(self):
        rospy.init_node('hoop_detector_2d')
        self.sub = rospy.Subscriber('/flattened_slice', PointCloud2, self.cloud_cb)
        self.pub = rospy.Publisher('/hoop_detection', Point, queue_size=1)

    def cloud_cb(self, msg):
        # 读取点云为 numpy 数组
        points = np.array([p[:2] for p in pc2.read_points(msg, skip_nans=True)])  # 只取 x, y

        if len(points) == 0:
            return

        # DBSCAN 聚类找团簇
        clustering = DBSCAN(eps=0.2, min_samples=10).fit(points)
        labels = clustering.labels_

        for label in set(labels):
            if label == -1:
                continue  # 噪声
            cluster = points[labels == label]

            if len(cluster) < 30:
                continue  # 忽略太小的点

            # 可选：用拟合圆、拟合椭圆等方法检测“圆形”
            center = np.mean(cluster, axis=0)
            result = Point(x=center[0], y=center[1], z=0.0)
            self.pub.publish(result)
            rospy.loginfo(f"Hoop detected at: ({center[0]:.2f}, {center[1]:.2f})")
            break  # 只发布一个篮筐

if __name__ == '__main__':
    try:
        HoopDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
