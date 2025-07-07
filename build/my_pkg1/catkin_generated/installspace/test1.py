import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ImprovedHoughCircleDetector:
    def __init__(self):
        rospy.init_node('improved_hough_circle_detector', anonymous=True)

        self.slice_sub = rospy.Subscriber('/flattened_slice', PointCloud2, self.point_cloud_callback)
        self.marker_pub = rospy.Publisher('/detected_circles_marker', Marker, queue_size=10)

        # 图像处理参数
        self.resolution = 0.05  # 栅格化分辨率，每个像素代表的物理尺寸 (米/像素)
        self.image_size_m = 5.0 # 图像覆盖的物理范围 (米)，例如从中心向外2.5米
        self.image_pixels = int(self.image_size_m / self.resolution) # 图像的像素尺寸

        # Canny边缘检测参数
        self.canny_thresh1 = 50
        self.canny_thresh2 = 150

        # 霍夫直线检测参数
        self.hough_line_rho = 1      # 像素分辨率
        self.hough_line_theta = np.pi / 180 # 角度分辨率 (弧度)
        self.hough_line_threshold = 100 # 累加器阈值
        self.hough_line_min_length = 50 # 最小线段长度
        self.hough_line_max_gap = 10    # 最大线段间隙

        # 霍夫圆检测参数
        # dp: 累加器分辨率与图像分辨率之比，dp=1表示相同分辨率，dp=2表示累加器分辨率是图像的一半
        self.hough_circle_dp = 1
        # minDist: 检测到的圆心之间的最小距离，防止检测到多个重叠的圆
        self.hough_circle_min_dist = 5
        # param1: Canny边缘检测的高阈值，低阈值是它的一半
        self.hough_circle_param1 = 200
        # param2: 圆心累加器阈值，越小越容易检测到圆，但误检率可能增加
        self.hough_circle_param2 = 10
        self.hough_circle_min_radius = 4 # 最小半径 (像素)
        self.hough_circle_max_radius = 20   # 最大半径 (像素)

        rospy.loginfo("Improved Hough Circle Detector Node Initialized.")

    def point_to_pixel(self, x, y):
        """将物理坐标转换为图像像素坐标"""
        # 将物理坐标原点移到图像中心
        px = int((x + self.image_size_m / 2) / self.resolution)
        py = int((y + self.image_size_m / 2) / self.resolution)
        return px, py

    def pixel_to_point(self, px, py):
        """将图像像素坐标转换回物理坐标"""
        x = (px * self.resolution) - (self.image_size_m / 2)
        y = (py * self.resolution) - (self.image_size_m / 2)
        return x, y

    def point_cloud_callback(self, msg):
        # 创建空白图像
        img = np.zeros((self.image_pixels, self.image_pixels), dtype=np.uint8)

        # 遍历点云并填充图像
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p[0], p[1], p[2]
            
            # 由于我们只关注 z=0 的切片，这里不检查 z
            
            px, py = self.point_to_pixel(x, y)

            # 确保像素坐标在图像范围内
            if 0 <= px < self.image_pixels and 0 <= py < self.image_pixels:
                # 简单地增加像素值，表示点密度。也可以直接设置为255表示有占用的区域。
                img[py, px] = min(255, img[py, px] + 10) # 增加像素值，使其更亮

        # 如果图像全黑，则没有有效点云
        if np.sum(img) == 0:
            #rospy.logwarn("Received empty or all-zero point cloud slice.")
            return

        # 对图像进行平滑处理，减少噪声
        blurred_img = cv2.GaussianBlur(img, (5, 5), 0)

        # --- 改进的霍夫检测策略 ---

        # 1. Canny边缘检测
        edges = cv2.Canny(blurred_img, self.canny_thresh1, self.canny_thresh2)

        # 2. 霍夫直线检测 (识别墙壁)
        lines = cv2.HoughLinesP(edges, self.hough_line_rho, self.hough_line_theta, 
                                self.hough_line_threshold, 
                                minLineLength=self.hough_line_min_length, 
                                maxLineGap=self.hough_line_max_gap)

        # 创建一个掩膜，用于“移除”直线区域
        processed_edges = edges.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # 在掩膜上绘制粗直线，将其区域“抹去”
                cv2.line(processed_edges, (x1, y1), (x2, y2), 0, 10) # 宽度为10像素的黑色线条

        # 3. 霍夫圆检测 (在去除直线后的图像上进行)
        # 这里的 min_radius 和 max_radius 需要根据篮筐的实际尺寸和分辨率进行调整
        # 例如，如果篮筐直径0.45米，分辨率0.05米/像素，则半径为 (0.45/2) / 0.05 = 4.5 像素
        # 可以设置 min_radius=4, max_radius=6
        circles = cv2.HoughCircles(processed_edges, cv2.HOUGH_GRADIENT,
                                   self.hough_circle_dp,
                                   self.hough_circle_min_dist,
                                   param1=self.hough_circle_param1,
                                   param2=self.hough_circle_param2,
                                   minRadius=self.hough_circle_min_radius,
                                   maxRadius=self.hough_circle_max_radius)

        if circles is None:
            rospy.logwarn("HoughCircles did not detect any circles.")
        else:
            rospy.loginfo(f"HoughCircles detected {len(circles[0])} circles.")
            # 打印检测到的原始像素圆数据
            for circle in circles[0, :]:
                rospy.loginfo(f"Detected circle (px): x={circle[0]:.2f}, y={circle[1]:.2f}, r={circle[2]:.2f}")

        # 发布可视化Marker
        self.publish_circles_marker(circles, msg.header.frame_id)

        # 可以在此处显示处理过程中的图像 (仅用于调试)
        cv2.imshow("Original Density Image", img)
        cv2.imshow("Edges", edges)
        # cv2.imshow("Processed Edges (Lines Removed)", processed_edges)
        # cv2.waitKey(1)

    def publish_circles_marker(self, circles, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "detected_circles"
        marker.id = 0
        marker.type = Marker.CYLINDER # 用圆柱体表示2D圆
        marker.action = Marker.ADD

        marker.color.a = 0.7 # 透明度
        marker.color.r = 1.0 # 红色
        marker.color.g = 0.0
        marker.color.b = 0.0

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i, (x, y, r) in enumerate(circles[0, :]):
                center_x_m, center_y_m = self.pixel_to_point(x, y)
                radius_m = r * self.resolution

                rospy.loginfo(f"Marker {i}: Calculated radius_m={radius_m:.4f}") # <--- 新增打印

                marker.id = i
                marker.pose.position.x = center_x_m
                marker.pose.position.y = center_y_m
                marker.pose.position.z = 0.01

                marker.scale.x = radius_m * 2
                marker.scale.y = radius_m * 2
                marker.scale.z = 0.02

                # 确保四元数有效 (如果之前被意外修改，通常不需要设置)
                marker.pose.orientation.w = 1.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0

                self.marker_pub.publish(marker)
                rospy.loginfo(f"Published Circle {i}: Center=({center_x_m:.2f}, {center_y_m:.2f}), Radius={radius_m:.2f} m")
        else:
            marker.action = Marker.DELETEALL
            self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        detector = ImprovedHoughCircleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()