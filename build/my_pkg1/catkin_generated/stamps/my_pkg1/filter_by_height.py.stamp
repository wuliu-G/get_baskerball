#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import PointField

class PointCloudFilter:
    def __init__(self):
        rospy.init_node('pointcloud_height_filter')

        self.sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.callback)
        self.pub = rospy.Publisher('/filtered_points', PointCloud2, queue_size=1)

        # 设置高度过滤范围（单位：米） 高度差是175

        self.min_z = rospy.get_param('~min_z', 1.72)
        self.max_z = rospy.get_param('~max_z',101.80)
    def callback(self, msg):
        # 读取点云数据并筛选 z 值
        filtered_points = []

        for point in pc2.read_points(msg, skip_nans=True):
            x, y, z = point[:3]
            if self.min_z <= z <= self.max_z:
                filtered_points.append(point)

        # 发布新的点云
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id  # 保持原坐标系一致

        cloud_filtered = pc2.create_cloud(msg.header, msg.fields, filtered_points)
        self.pub.publish(cloud_filtered)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PointCloudFilter()
    node.run()
