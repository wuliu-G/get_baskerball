#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

class TargetCenterPublisher:
    def __init__(self):
        rospy.init_node('target_center_publisher')

        self.sub = rospy.Subscriber('/highlight_points', PointCloud2, self.callback)
        self.pub = rospy.Publisher('/object', PointStamped, queue_size=1)

    def callback(self, msg):
        field_names = [f.name for f in msg.fields]

        if 'intensity' not in field_names:
            rospy.logwarn("点云中不包含 intensity 字段，跳过本帧")
            return

        intensity_idx = field_names.index('intensity')

        all_points = list(pc2.read_points(msg, field_names=field_names, skip_nans=True))

        if len(all_points) < 5:
            rospy.logwarn("点数不足 5 个，跳过")
            return



        # 正确做法：按 intensity 值排序



        sorted_by_intensity = sorted(all_points, key=lambda pt: pt[intensity_idx])
        target_points = all_points  # 最低强度的5个点；删除了最强点，所以所有点都是最弱点

        # 计算中心点
        avg_x = sum(pt[0] for pt in target_points) / 5.0
        avg_y = sum(pt[1] for pt in target_points) / 5.0
        avg_z = sum(pt[2] for pt in target_points) / 5.0

        rospy.loginfo("目标中心坐标: (%.3f, %.3f, %.3f)", avg_x, avg_y, avg_z)

        # 发布为 PointStamped
        center_point = PointStamped()
        center_point.header = Header()
        center_point.header.stamp = rospy.Time.now()
        center_point.header.frame_id = msg.header.frame_id
        center_point.point.x = avg_x
        center_point.point.y = avg_y
        center_point.point.z = avg_z

        self.pub.publish(center_point)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = TargetCenterPublisher()
    node.run()
