#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class PointCloudSlicer:
    def __init__(self):
        rospy.init_node('pointcloud_flatten_slice')

        self.sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/flattened_slice', PointCloud2, queue_size=1)

        # 可调参数：切片范围
        self.min_z = rospy.get_param('~min_z', 0.5)
        self.max_z = rospy.get_param('~max_z', 0.6)
        self.flatten_z = rospy.get_param('~flatten_z', 0.0)  # 压平到的高度

    def callback(self, msg):
        # 保留全部字段，确保结构一致
        fields = msg.fields
        header = msg.header

        points = []
        for p in pc2.read_points(msg, field_names=None, skip_nans=True):
            x, y, z = p[:3]
            if self.min_z <= z <= self.max_z:
                # 改 z 值为 flatten_z，其余字段保留
                new_point = list(p)
                new_point[2] = self.flatten_z
                points.append(tuple(new_point))

        if points:
            cloud_out = pc2.create_cloud(header, fields, points)
            self.pub.publish(cloud_out)
        else:
            rospy.logwarn_throttle(5.0, "No points in specified slice range.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    PointCloudSlicer().run()
