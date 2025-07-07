#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped

class BasketDetector:
    def __init__(self):
        rospy.init_node('basket_detector', anonymous=True)

        # 订阅你的压缩切片点云
        rospy.Subscriber("/flattened_slice", PointCloud2, self.pointcloud_callback)

        # 发布检测到的篮筐点
        self.basket_pub = rospy.Publisher("/detected_basket", PointStamped, queue_size=1)

        rospy.loginfo("Basket detector node started and listening to /flattened_slice.")
        rospy.spin()

    def pointcloud_callback(self, msg):
        min_x = float('inf')
        best_point = None

        # 遍历点云
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            if x > 0 and x < min_x:  # 只看前方点，取最小 x
                min_x = x
                best_point = (x, y, z)

        if best_point:
            # 构造并发布 PointStamped
            basket_point = PointStamped()
            basket_point.header = msg.header  # 使用同样的时间戳和坐标系
            basket_point.point.x = best_point[0]
            basket_point.point.y = best_point[1]
            basket_point.point.z = best_point[2]

            self.basket_pub.publish(basket_point)
            rospy.loginfo("Basket point published: x=%.2f, y=%.2f", best_point[0], best_point[1])

if __name__ == '__main__':
    try:
        BasketDetector()
    except rospy.ROSInterruptException:
        pass
