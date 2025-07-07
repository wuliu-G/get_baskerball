#!/usr/bin/env python3
# 每6帧点云数据，累积并过滤高度范围内的点云
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from copy import deepcopy

class PointCloudAggregatorByCount:
    def __init__(self):
        rospy.init_node('pointcloud_aggregator_by_count')

        # 参数设置
        self.min_z = rospy.get_param('~min_z', 1.72)
        self.max_z = rospy.get_param('~max_z', 1.80)
        self.max_frames = rospy.get_param('~max_frames', 10)

        # 订阅和发布器
        self.sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.callback)
        self.pub = rospy.Publisher('/point_strong', PointCloud2, queue_size=1)

        # 缓冲区
        self.point_buffer = []
        self.frame_count = 0
        self.fields = None
        self.frame_id = None

    def callback(self, msg):
        # 第一次记录字段和坐标系
        if self.fields is None:
            self.fields = deepcopy(msg.fields)
            self.frame_id = msg.header.frame_id

        # 累积点云
        points = list(pc2.read_points(msg, skip_nans=True))
        self.point_buffer.extend(points)
        self.frame_count += 1

        # 达到6帧就处理
        if self.frame_count >= self.max_frames:
            # z值过滤
            filtered_points = [pt for pt in self.point_buffer if self.min_z <= pt[2] <= self.max_z]

            # 构建新点云消息
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.frame_id

            cloud_filtered = pc2.create_cloud(header, self.fields, filtered_points)
            self.pub.publish(cloud_filtered)

            # 重置缓存
            self.point_buffer = []
            self.frame_count = 0

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PointCloudAggregatorByCount()
    node.run()
