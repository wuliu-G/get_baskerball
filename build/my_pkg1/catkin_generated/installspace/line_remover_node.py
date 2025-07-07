#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

# 导入 scikit-learn 用于 RANSAC 拟合
from sklearn import linear_model

class WallRemover:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('wall_remover', anonymous=True)

        # 订阅 /flattened_slice 话题，接收原始点云
        self.subscriber = rospy.Subscriber('/flattened_slice', PointCloud2, self.pointcloud_callback)
        # 发布过滤后的点云到 /remove_L 话题
        self.publisher = rospy.Publisher('/remove_L', PointCloud2, queue_size=10)

        # --- 重点：RANSAC 模型配置和初始化日志必须正确缩进到 __init__ 方法内 ---
        # 配置 RANSAC 拟合器
        # 我们要拟合的是直线 (y = mx + c)，所以使用线性回归作为基础模型
        # min_samples: 拟合模型所需的最小样本数，直线至少需要2个点
        # residual_threshold: 内点到模型的最大距离，单位是米。
        #                     这是非常关键的参数，请根据你的激光雷达精度和墙体厚度进行调整。
        #                     如果墙体较厚或点云有噪声，可能需要适当增大。
        # max_trials: RANSAC 迭代的最大次数，更多次数可以提高找到最佳模型的概率
        self.ransac_model = linear_model.RANSACRegressor(
            linear_model.LinearRegression(),
            min_samples=2,
            residual_threshold=0.05,  # 0.05 米 = 5 厘米。请根据实际情况调整！
            max_trials=100
        )

        rospy.loginfo("Wall Remover Node Initialized. Subscribing to /flattened_slice and publishing to /remove_L (using scikit-learn RANSAC).")

    def pointcloud_callback(self, msg):
        rospy.loginfo("Received PointCloud2 message.")

        # 将 ROS PointCloud2 消息转换为 NumPy 数组 (只提取 x, y 坐标用于2D拟合)
        # 我们同时也会保留 Z 坐标，以便后续重新构建3D点云
        all_original_points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            all_original_points.append([p[0], p[1], p[2]])

        if not all_original_points:
            rospy.logwarn("Received empty point cloud or no valid points, skipping processing.")
            # 如果点云为空，直接发布原始消息 (或空消息，取决于你的需求)
            self.publisher.publish(msg)
            return

        all_original_points_np = np.array(all_original_points, dtype=np.float32)

        # 如果点少于 RANSAC 拟合所需的最小样本数，则无法进行拟合
        if all_original_points_np.shape[0] < self.ransac_model.min_samples:
            rospy.loginfo(f"Not enough points ({all_original_points_np.shape[0]}) to perform RANSAC. Publishing original cloud.")
            self.publisher.publish(msg)
            return

        # 提取 X 和 Y 坐标进行 2D 直线拟合
        X = all_original_points_np[:, 0].reshape(-1, 1) # X 坐标作为特征 (需要二维数组)
        y = all_original_points_np[:, 1]               # Y 坐标作为目标

        try:
            # 执行 RANSAC 直线拟合
            self.ransac_model.fit(X, y)
        except ValueError as e:
            # 捕获 RANSAC 拟合可能出现的错误，例如所有点都是异常值
            rospy.logwarn(f"RANSAC fitting failed (e.g., all points are outliers or insufficient data): {e}. Publishing original cloud.")
            self.publisher.publish(msg)
            return

        # 获取 RANSAC 算法识别出的内点和外点掩码
        inlier_mask = self.ransac_model.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask) # 墙体以外的点即是我们想保留的点

        # 根据掩码从原始点云中提取墙体点和非墙体点
        wall_points_xyz = all_original_points_np[inlier_mask]
        filtered_points_xyz = all_original_points_np[outlier_mask]

        rospy.loginfo(f"Found {len(wall_points_xyz)} wall points, retaining {len(filtered_points_xyz)} non-wall points.")

        # 准备要发布的新点云数据
        # pc2.create_cloud_xyz32 需要一个列表而不是生成器，且每个点是 (x, y, z) 元组
        points_to_publish = []
        if len(filtered_points_xyz) > 0:
            for p in filtered_points_xyz:
                # 确保每个坐标都是 float 类型，以防万一
                points_to_publish.append((float(p[0]), float(p[1]), float(p[2])))
        else:
            rospy.logwarn("No points remained after filtering. Publishing empty point cloud.")

        # 创建新的 PointCloud2 消息
        # 使用原始消息的 header，确保时间戳和坐标系一致
        filtered_ros_cloud = pc2.create_cloud_xyz32(msg.header, points_to_publish)
        
        # 发布过滤后的点云
        self.publisher.publish(filtered_ros_cloud)
        rospy.loginfo("Filtered point cloud published to /remove_L.")

    # --- 重点：添加 run() 方法，包含 rospy.spin() ---
    def run(self):
        # 保持节点运行直到被关闭
        rospy.spin()

# --- 重点：添加主执行块，创建 WallRemover 实例并调用其 run() 方法 ---
if __name__ == '__main__':
    try:
        remover = WallRemover()
        remover.run() # 调用 run 方法，其中包含 rospy.spin()
    except rospy.ROSInterruptException:
        # 捕获 ROS 中断异常 (例如 Ctrl+C)
        rospy.loginfo("Wall Remover Node Shutting Down.")
        pass