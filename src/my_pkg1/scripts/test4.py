#!/usr/bin/env python
#  通过反射强度，确定篮筐
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

import math

class HighlightStrongPoints:
    def __init__(self):
        rospy.init_node('highlight_strong_points')

        self.sub = rospy.Subscriber('/point_strong', PointCloud2, self.callback)
        self.pub = rospy.Publisher('/highlight_points', PointCloud2, queue_size=1)

        self.max_range = rospy.get_param('~max_range', 5.0)  # 5米范围

    def callback(self, msg):
        field_names = [f.name for f in msg.fields]

        # 保留距离原点小于 max_range 的点
        nearby_points = []
        for pt in pc2.read_points(msg, field_names=field_names, skip_nans=True):
            x, y, z = pt[:3]
            distance = math.sqrt(x**2 + y**2 + z**2)
            if distance <= self.max_range:
                nearby_points.append(pt)

        # 确保有足够的点
        if len(nearby_points) < 10:
            rospy.logwarn("可用点太少，跳过本帧")
            return

        # 找强度最大/最小的5个点
        intensity_idx = field_names.index('intensity')
        sorted_by_intensity = sorted(nearby_points, key=lambda p: p[intensity_idx])

        lowest_5 = sorted_by_intensity[-5:]
        # highest_5 = sorted_by_intensity[-5:]

        # selected_points = lowest_5 + highest_5
        selected_points = lowest_5

        # 创建并发布新点云
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id

        cloud_out = pc2.create_cloud(header, msg.fields, selected_points)
        self.pub.publish(cloud_out)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = HighlightStrongPoints()
    node.run()
