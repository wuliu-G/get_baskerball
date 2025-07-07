#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import serial
import time
import threading
from geometry_msgs.msg import PointStamped
import glob

def find_usb_serial_port():
    ports = glob.glob("/dev/ttyUSB*")
    if not ports:
        rospy.logerr("没有找到任何 /dev/ttyUSB* 设备")
        return None
    return ports[0]  # 只取第一个


class UARTSenderReceiver:
    def __init__(self, baud=115200):
        self.baud = baud
        self.ser = None
        self.running = True
        self.port_in_use = None # 记录当前使用的串口端口
        self.reconnect_attempt_delay = 1.0 # 重新连接尝试的延迟时间

        self.connect_serial() # 首次连接

        self.thread = threading.Thread(target=self.recv_thread)
        self.thread.daemon = True
        self.thread.start()

    def connect_serial(self):
        """尝试查找并连接串口"""
        if self.ser and self.ser.isOpen():
            self.ser.close() # 如果已打开，先关闭
            self.ser = None

        rospy.loginfo("[UART] 尝试连接串口...")
        new_port = find_usb_serial_port()

        if new_port is None:
            rospy.logwarn("[UART] 未找到 /dev/ttyUSB* 设备，将稍后重试。")
            self.ser = None # 确保ser为None
            self.port_in_use = None
            return False

        if new_port != self.port_in_use:
            rospy.loginfo(f"[UART] 发现新串口设备：{new_port}")
            self.port_in_use = new_port
        else:
            rospy.loginfo(f"[UART] 尝试重新连接到 {new_port}。")

        try:
            self.ser = serial.Serial(self.port_in_use, self.baud, timeout=0.1)
            time.sleep(2) # 给串口一点时间初始化
            if self.ser.isOpen():
                rospy.loginfo("[UART] 串口已成功打开: %s", self.port_in_use)
                return True
        except serial.SerialException as e:
            rospy.logerr(f"[UART] 串口 {self.port_in_use} 打开失败: {e}，将稍后重试。")
            self.ser = None
            self.port_in_use = None
            return False
        return False # 如果代码运行到这里，说明没有成功打开

    def send_angle_hex(self, angle):
        if self.ser is None or not self.ser.isOpen():
            rospy.logwarn("[UART] 串口未打开或已关闭，尝试重新连接...")
            if not self.connect_serial(): # 尝试重新连接
                rospy.logerr("[UART] 重新连接串口失败，无法发送数据。")
                return # 如果重连失败，则直接返回

        # 1. 方向：左=00，右=01角
        dir_x = 0 if angle >= 0 else 1
        angle_value = int(abs(angle))*10  # 度取整数

        # 2. 保留字段（上下方向）
        dir_y = 0
        v_y_sub = 0

        # 3. 拼接 hex 字符串
        data = ''.join([
            f"{dir_x:02x}",             # 1字节 方向
            f"{angle_value:04x}",       # 2字节 角度值
            f"{dir_y:02x}",             # 1字节 保留
            f"{v_y_sub:04x}",           # 2字节 保留
            "0d",                       # 回车
            "0a"                        # 换行
        ])

        try:
            byte_data = bytes.fromhex(data)
            self.ser.write(byte_data)
            time.sleep(0.5)
            self.ser.flush()
            rospy.loginfo("[UART] 发送十六进制: %s", data.upper())
        except serial.SerialException as e: # 捕获串口通信错误
            rospy.logerr(f"[UART] 发送失败，串口可能已断开: {e}。尝试重新连接...")
            self.ser = None # 强制将串口设为None，以便下次发送时触发重连
            self.port_in_use = None
        except Exception as e:
            rospy.logerr(f"[UART] 发送时发生未知错误: {e}")

    def recv_thread(self):
        while self.running:
            if self.ser is None or not self.ser.isOpen():
                # 如果串口未打开，尝试重新连接
                if not self.connect_serial():
                    # 如果重连失败，等待一段时间再试，避免无限循环占用CPU
                    time.sleep(self.reconnect_attempt_delay)
                    continue # 继续循环，等待下一次重连尝试

            try:
                # readline() 在timeout后会返回空字符串，不会抛异常
                data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    rospy.loginfo("[UART] 接收: %s", data)
            except serial.SerialException as e: # 捕获串口通信错误
                rospy.logwarn(f"[UART] 接收失败，串口可能已断开: {e}。尝试重新连接...")
                self.ser = None # 强制将串口设为None
                self.port_in_use = None
            except Exception as e:
                rospy.logwarn(f"[UART] 接收时发生未知错误: {e}")
            time.sleep(0.01) # 短暂休眠，避免CPU占用过高

    def close(self):
        self.running = False
        time.sleep(0.2) # 给予接收线程一些时间停止
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0) # 等待线程结束
            if self.thread.is_alive():
                rospy.logwarn("[UART] 接收线程未能及时停止。")

        if self.ser and self.ser.isOpen():
            self.ser.close()
            rospy.loginfo("[UART] 串口已关闭")

class AngleProcessor:
    def __init__(self):
        rospy.init_node('object_uart_hex_node', anonymous=True)
        self.uart = UARTSenderReceiver()
        rospy.Subscriber("/object", PointStamped, self.callback)
        rospy.on_shutdown(self.shutdown_hook)

    def callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        if y == 0:
            rospy.logwarn("y=0，角度未定义")
            return
        angle = math.degrees(math.atan2(x, y))
        rospy.loginfo("偏移角度: %.2f°", angle)
        self.uart.send_angle_hex(angle)

    def shutdown_hook(self):
        self.uart.close()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = AngleProcessor()
        node.run()
    except rospy.ROSInterruptException:
        pass