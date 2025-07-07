# -*- coding: utf-8 -*-
# 原始功能：不断获取当前角度 + 现在增加 ROS 发布

from ultralytics import YOLO
import numpy as np
import cv2
import pyrealsense2 as rs
import math
import time

# ✅ ROS 支持
import rospy
from std_msgs.msg import Float64

# 计算俯仰角
def calculate_pitch(accel):
    az = accel.z
    ay = accel.y
    pitch_rad = math.atan2(az, -ay)
    return math.degrees(pitch_rad), pitch_rad

# 获取角度
def get_angle(pipeline):
    sum_pitch_deg = 0.0
    n = 3
    try:
        for _ in range(n):
            frames = pipeline.wait_for_frames()
            accel_frame = frames.first_or_default(rs.stream.accel)
            if not accel_frame:
                print("加速度数据缺失")
                continue
            accel_data = accel_frame.as_motion_frame().get_motion_data()
            current_degree, _ = calculate_pitch(accel_data)
            sum_pitch_deg += current_degree
        return sum_pitch_deg / n
    except Exception as e:
        print(f"角度获取失败: {str(e)}")
        return None

# ✅ 初始化 ROS 节点
rospy.init_node('angle_publisher', anonymous=True)
angle_pub = rospy.Publisher('/current_angle', Float64, queue_size=10)
rate = rospy.Rate(10)

# ✅ Realsense 初始化 —— 保持你原来能正常工作的顺序
pipeline = rs.pipeline()
config = rs.config()

try:
    pipeline_profile = pipeline.start(config)
except Exception as e:
    print(f"设备连接失败: {str(e)}")
    exit(1)

# 添加流（即使不需要用图像帧）
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)

# 主循环
while not rospy.is_shutdown():
    angle = get_angle(pipeline)
    if angle is not None:
        print(f"当前角度为 {angle:.2f}")
        angle_pub.publish(Float64(angle))
    rate.sleep()
