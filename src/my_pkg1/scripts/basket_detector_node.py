#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
from geometry_msgs.msg import PointStamped

# 霍夫变换参数
IMAGE_RESOLUTION = 0.01
MIN_RADIUS_M = 0.15
MAX_RADIUS_M = 0.25
HOUGH_DP = 1
HOUGH_MIN_DIST_M = 0.5
HOUGH_PARAM1 = 100
HOUGH_PARAM2 = 20

class BasketDetector:
    def __init__(self):
        rospy.init_node('basket_detector_node', anonymous=True)

        self.filtered_points_sub = rospy.Subscriber('/filtered_flatten', PointCloud2, self.filtered_points_callback)
        self.basket_center_pub = rospy.Publisher('/basket_center', PointStamped, queue_size=1)

        rospy.loginfo("Basket Detector Node initialized. Waiting for filtered point clouds on /filtered_flatten.")

    def filtered_points_callback(self, msg):
        points_list = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([p[0], p[1]])  # 忽略 z，只用 x 和 y

        if not points_list:
            rospy.logwarn("Received empty filtered point cloud for basket detection.")
            return

        points_np = np.array(points_list, dtype=np.float32)

        min_x, min_y = np.min(points_np, axis=0)
        max_x, max_y = np.max(points_np, axis=0)

        margin_m = 0.5 
        img_width_m = (max_x - min_x) + 2 * margin_m
        img_height_m = (max_y - min_y) + 2 * margin_m

        img_width_px = int(img_width_m / IMAGE_RESOLUTION)
        img_height_px = int(img_height_m / IMAGE_RESOLUTION)

        if img_width_px == 0 or img_height_px == 0:
            rospy.logwarn("Image dimensions are zero, likely too few points or points are too close.")
            return

        image = np.zeros((img_height_px, img_width_px), dtype=np.uint8)

        points_in_img_coords = np.copy(points_np)
        points_in_img_coords[:, 0] = (points_np[:, 0] - min_x + margin_m) / IMAGE_RESOLUTION
        points_in_img_coords[:, 1] = (points_np[:, 1] - min_y + margin_m) / IMAGE_RESOLUTION

        for p in points_in_img_coords:
            px = int(p[0])
            py = int(p[1])
            if 0 <= px < img_width_px and 0 <= py < img_height_px:
                image[py, px] = 255

        # blurred_image = cv2.GaussianBlur(image, (5, 5), 0)

        min_radius_px = int(MIN_RADIUS_M / IMAGE_RESOLUTION)
        max_radius_px = int(MAX_RADIUS_M / IMAGE_RESOLUTION)
        min_dist_px = int(HOUGH_MIN_DIST_M / IMAGE_RESOLUTION)

        circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 
                                   dp=HOUGH_DP, 
                                   minDist=min_dist_px,
                                   param1=HOUGH_PARAM1, 
                                   param2=HOUGH_PARAM2, 
                                   minRadius=min_radius_px, 
                                   maxRadius=max_radius_px)

        if circles is not None:
            circles = np.uint16(np.round(circles))
            rospy.loginfo(f"Detected {len(circles[0])} circle(s).")

            best_circle = circles[0][0]

            x_px, y_px, r_px = best_circle

            basket_center_x = x_px * IMAGE_RESOLUTION + min_x - margin_m
            basket_center_y = y_px * IMAGE_RESOLUTION + min_y - margin_m

            rospy.loginfo(f"Detected basket center at (X: {basket_center_x:.2f}m, Y: {basket_center_y:.2f}m) with radius {r_px * IMAGE_RESOLUTION:.2f}m")

            basket_point_msg = PointStamped()
            basket_point_msg.header = msg.header
            basket_point_msg.point.x = basket_center_x
            basket_point_msg.point.y = basket_center_y
            basket_point_msg.point.z = 0.0  # 保持为2D处理

            self.basket_center_pub.publish(basket_point_msg)
        else:
            rospy.logwarn("No circles detected by Hough Transform.")

if __name__ == '__main__':
    try:
        BasketDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
