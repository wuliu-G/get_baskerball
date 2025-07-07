#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import numpy as np
from sklearn.cluster import DBSCAN

class HoopDetector:
    def __init__(self):
        rospy.init_node('hoop_detector_2d')
        self.sub = rospy.Subscriber('/flattened_slice', PointCloud2, self.cloud_cb)
        self.pub = rospy.Publisher('/hoop_detection', PointStamped, queue_size=1)

        self.last_center = None
        self.last_header = None  # 保存 header 以便发布时设置坐标系

    def cloud_cb(self, msg):
        points = np.array([p[:2] for p in pc2.read_points(msg, skip_nans=True)])  # 只取 x, y

        if len(points) == 0:
            return

        # 聚类
        clustering = DBSCAN(eps=0.2, min_samples=10).fit(points)
        labels = clustering.labels_

        for label in set(labels):
            if label == -1:
                continue
            cluster = points[labels == label]

            if len(cluster) < 30:
                continue

            center = np.mean(cluster, axis=0)
            self.last_center = center
            self.last_header = msg.header  # 保存 header（包含 frame_id）
            rospy.loginfo(f"Hoop detected at: ({center[0]:.2f}, {center[1]:.2f})")
            break  # 只处理一个篮筐

        # 无论有没有新的检测，只要有记录，就持续发布
        if self.last_center is not None and self.last_header is not None:
            point_msg = PointStamped()
            point_msg.header = self.last_header
            point_msg.header.stamp = rospy.Time.now()
            point_msg.point.x = self.last_center[0]
            point_msg.point.y = self.last_center[1]
            point_msg.point.z = 0.0
            self.pub.publish(point_msg)

if __name__ == '__main__':
    try:
        HoopDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
