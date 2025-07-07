import pyrealsense2 as rs
import numpy as np
import time
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3Stamped
from ahrs.filters import Madgwick # 推荐使用 Madgwick 滤波器

# --- 配置参数 ---
# RealSense 相机配置
IMU_FPS = 200 # IMU 采样率，确保与 enable_stream 中的频率一致

# ROS 话题配置
TOPIC_NAME = "/current_angle"

# --- 全局变量 ---
# IMU 滤波器实例
# 0.1 表示滤波器增益，可以根据实际效果调整。数值越小，滤波器对加速度计的依赖越大。
# sample_period 用于 Madgwick 滤波器，通常是 1/IMU_FPS
madgwick_filter = Madgwick(sample_period=(1.0 / IMU_FPS), beta=0.1)
# 初始四元数，表示无旋转（单位四元数）
current_quaternion = np.array([1.0, 0.0, 0.0, 0.0])

# ROS 发布器声明
angle_pub = None

# --- RealSense 初始化函数 ---
def initialize_realsense_pipeline():
    """初始化 RealSense 管线并启用必要的流。"""
    pipeline = rs.pipeline()
    config = rs.config()

    try:
        print("正在配置 RealSense 流...")
        # 启用彩色和深度流 (可选，但通常用于 RealSense 应用)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # 启用加速度计和陀螺仪流 (IMU)
        # 确保您的 RealSense 型号（例如 D435i, D455）具有 IMU。
        # D415 没有 IMU。
        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, IMU_FPS)
        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, IMU_FPS)

        # 启动管线
        pipeline_profile = pipeline.start(config)
        print("RealSense 管线已成功启动。")

        # 创建一个对齐对象，将深度帧对齐到彩色帧 (可选)
        # align = rs.align(rs.stream.color)

        # 获取摄像头内参 (可选，但推荐保留)
        # rgb_profile = pipeline_profile.get_stream(rs.stream.color)
        # rgb_intrinsics = rgb_profile.as_video_stream_profile().get_intrinsics()
        # c_x, c_y = rgb_intrinsics.ppx, rgb_intrinsics.ppy
        # print(f"彩色摄像头中心: ({c_x}, {c_y})")

        return pipeline
        # 如果需要 align 对象，可以 return pipeline, align
        # 但在这个 IMU 姿态发布场景中，align 并非必需
    except Exception as e:
        print(f"错误：启动 RealSense 管线或启用流失败: {str(e)}")
        print("请确保摄像头已连接并请求了支持的流。")
        return None

# --- 辅助函数：四元数转欧拉角 (Z-Y-X 顺序: 偏航, 俯仰, 滚动) ---
def quaternion_to_euler(q):
    """
    将四元数转换为欧拉角 (Z-Y-X 顺序，即 偏航(yaw), 俯仰(pitch), 滚动(roll))
    输入: q = [w, x, y, z]
    输出: [yaw, pitch, roll] (单位: 弧度)
    """
    w, x, y, z = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp) # 如果超出范围，使用 90 度
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([yaw, pitch, roll])

# --- 初始化 ROS 节点和发布器 ---
def initialize_ros_publisher():
    """初始化 ROS 节点和发布器。"""
    global angle_pub
    try:
        # 初始化 ROS 节点
        # anonymous=True 会在节点名后添加一个随机数字，避免命名冲突
        rospy.init_node('realsense_imu_angle_publisher', anonymous=True)
        print("ROS 节点 'realsense_imu_angle_publisher' 已初始化。")

        # 创建发布器
        # topic_name: /current_angle
        # message_type: geometry_msgs/Vector3Stamped - 包含时间戳、帧ID和3D向量
        # queue_size: 10 - 消息队列大小
        angle_pub = rospy.Publisher(TOPIC_NAME, Vector3Stamped, queue_size=10)
        print(f"ROS 发布器已创建，话题: {TOPIC_NAME}")

    except rospy.ROSInitException as e:
        print(f"错误：ROS 节点初始化失败: {str(e)}")
        angle_pub = None
    except Exception as e:
        print(f"错误：创建 ROS 发布器失败: {str(e)}")
        angle_pub = None

# --- 主函数：数据流和处理 ---
def main():
    global current_quaternion, angle_pub

    # 初始化 RealSense
    pipeline = initialize_realsense_pipeline()
    if pipeline is None:
        return

    # 初始化 ROS 发布器
    initialize_ros_publisher()
    if angle_pub is None:
        pipeline.stop() # 如果 ROS 发布器未能初始化，也要停止 RealSense 管线
        return

    # 定义 ROS 发布频率
    # 我们希望以 IMU 的采样率来更新和发布姿态
    rate = rospy.Rate(IMU_FPS)

    print("进入主循环，开始处理 RealSense IMU 数据并发布到 ROS 话题...")
    try:
        while not rospy.is_shutdown(): # 只要 ROS 节点没有关闭就一直运行
            frames = pipeline.wait_for_frames()

            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)

            if accel_frame and gyro_frame:
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()

                # 将加速度计和陀螺仪数据转换为 numpy 数组
                # RealSense IMU 的轴向可能需要根据实际情况调整或校准
                # 这里假设数据顺序为 [x, y, z]
                accel = np.array([accel_data.x, accel_data.y, accel_data.z])
                gyro = np.array([gyro_data.x, gyro_data.y, gyro_data.z]) # 陀螺仪数据通常已经是弧度/秒

                # 使用 Madgwick 滤波器更新四元数
                current_quaternion = madgwick_filter.updateIMU(
                    Q=current_quaternion, gyr=gyro, acc=accel
                )

                # 将四元数转换为欧拉角 (偏航, 俯仰, 滚动)
                # 转换为度数以便于理解和调试
                yaw_rad, pitch_rad, roll_rad = quaternion_to_euler(current_quaternion)
                yaw_deg = np.degrees(yaw_rad)
                pitch_deg = np.degrees(pitch_rad)
                roll_deg = np.degrees(roll_rad)

                # 打印当前角度到控制台
                print(f"姿态 (度): 偏航={yaw_deg:.2f}, 俯仰={pitch_deg:.2f}, 滚动={roll_deg:.2f}")

                # --- 发布数据到 ROS 话题 ---
                if angle_pub:
                    # 创建 ROS 消息 (geometry_msgs/Vector3Stamped)
                    angle_msg = Vector3Stamped()
                    angle_msg.header = Header()
                    angle_msg.header.stamp = rospy.Time.now() # 设置时间戳
                    angle_msg.header.frame_id = "imu_link" # 设置坐标系 ID (可以根据你的机器人URDF调整)

                    # 将欧拉角数据填充到 Vector3 字段
                    # 注意：roll, pitch, yaw 对应 Vector3 的 x, y, z 是一个常见的约定，
                    # 但请根据你的机器人定义和应用需求进行确认。
                    angle_msg.vector.x = roll_deg  # Roll (绕X轴旋转)
                    angle_msg.vector.y = pitch_deg # Pitch (绕Y轴旋转)
                    angle_msg.vector.z = yaw_deg   # Yaw (绕Z轴旋转)

                    try:
                        angle_pub.publish(angle_msg)
                        # print(f"ROS 话题 '{TOPIC_NAME}' 已发布角度。")
                    except rospy.ROSException as e:
                        # 捕获 ROS 发布错误，例如当节点关闭时
                        if not rospy.is_shutdown(): # 避免在正常关闭时打印错误
                            print(f"ROS 发布失败: {str(e)}")
            else:
                # 即使没有 IMU 数据，ROS 循环也应继续，以便可以正常关机
                # print("未获取到加速度计或陀螺仪数据。请确保您的 RealSense 摄像头有 IMU。")
                pass

            rate.sleep() # 按照设定的频率休眠，确保发布频率稳定

    except KeyboardInterrupt:
        print("用户终止脚本 (Ctrl+C)。")
    except Exception as e:
        print(f"主循环中发生错误: {str(e)}")
    finally:
        print("停止 RealSense 管线...")
        pipeline.stop()
        print("RealSense 管线已停止。")
        # rospy.signal_shutdown() 会在 ROS 节点终止时自动调用，此处无需显式调用
        print("程序退出。")

if __name__ == "__main__":
    main()