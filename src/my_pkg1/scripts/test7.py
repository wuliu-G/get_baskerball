#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
import math

class QuarticEquationSolver:
    def __init__(self):
        rospy.init_node('quartic_equation_solver')
        
        # 从参数服务器获取常量，如果没有设置则使用默认值
        self.H = rospy.get_param('~H', 1.0)
        self.h2 = rospy.get_param('~h2', 0.5)
        self.L1 = rospy.get_param('~L1', 2.0)
        self.L2 = rospy.get_param('~L2', 1.5)
        self.v = rospy.get_param('~v', 10)
        self.g = rospy.get_param('~g', 10)
        
        rospy.loginfo("常量参数: H=%.3f, h2=%.3f, L1=%.3f, L2=%.3f,v=%.3f, g=%.3f, ", 
                     self.H, self.h2, self.L1, self.L2, self.v, self.g)
        
        # 订阅 /xy_distance 话题
        self.sub = rospy.Subscriber('/xy_distance', Float64, self.callback)

        # 发布 /fire_angle 话题
        self.fire_angle_pub = rospy.Publisher('/fire_angle', Float64, queue_size=10)
        
    def solve_quartic_equation(self, a, b, c, d):
        """
        求解四元一次方程 at^4 + bt^2 + ct + d = 0
        返回最大的正数解
        """
        try:
            # 构造系数数组 [d, c, 0, b, a] 对应 d + ct + 0*t^2 + bt^3 + at^4
            coefficients = [d, c, 0, b, a]
            
            # 使用numpy求解多项式方程
            roots = np.roots(coefficients)
            
            # 过滤出实数解
            real_roots = []
            for root in roots:
                if np.isreal(root):
                    real_roots.append(float(root.real))
            
            # 过滤出正数解
            positive_roots = [root for root in real_roots if root > 0]
            
            if positive_roots:
                max_positive_root = max(positive_roots)
                rospy.loginfo("方程 %.3ft^4 + %.3ft^2 + %.3ft + %.3f = 0", a, b, c, d)
                rospy.loginfo("所有实数根: %s", real_roots)
                rospy.loginfo("正数根: %s", positive_roots)
                rospy.loginfo("最大正数解: %.6f", max_positive_root)
                return max_positive_root
            else:
                rospy.logwarn("方程无正数解")
                rospy.loginfo("方程 %.3ft^4 + %.3ft^2 + %.3ft + %.3f = 0", a, b, c, d)
                rospy.loginfo("所有实数根: %s", real_roots)
                return None
                
        except Exception as e:
            rospy.logerr("求解方程时出错: %s", str(e))
            return None
    
    def calculate_coefficients(self, xy_distance):
        """
        根据常量和输入数据计算方程系数
        这里是示例计算，你可以根据实际需求修改
        """
        # 示例计算方式，你可以根据实际物理模型修改
        if xy_distance is not None:
            # 示例系数计算（根据你的实际需求修改这些公式）
            a = -self.g*self.g*0.25
            b = self.v*self.v- self.g*(self.H-self.h2)
            c = 2*self.v*self.L1
            d = self.L1*self.L1-(self.H-self.h2)*(self.H-self.h2)-(xy_distance+self.L2)*(xy_distance+self.L2)

            return a, b, c, d,(xy_distance+self.L2)
        else:
            rospy.logwarn("xy_distance数据无效")
            return None, None, None, None
    
    def callback(self, msg):
        """
        处理 /xy_distance 话题的回调函数
        """
        rospy.loginfo("收到xy_distance数据: %.6f", msg.data)

        # 根据接收到的数据和常量计算方程系数
        a, b, c, d,x1_L2 = self.calculate_coefficients(msg.data)
        
        if a is not None:
            # 求解四元一次方程
            max_root = self.solve_quartic_equation(a, b, c, d)
            
            if max_root is not None:
                cos_alpha = x1_L2 / (self.L1 + self.v * max_root)

                # 计算alpha角度（弧度）
                alpha = math.acos(cos_alpha)

                # 发布alpha角度到 /fire_angle 话题
                fire_angle_msg = Float64()
                fire_angle_msg.data = alpha
                self.fire_angle_pub.publish(fire_angle_msg)

                print("=" * 50)
                print("最大正数解: {:.6f}".format(max_root))
                print("cos(alpha): {:.6f}".format(cos_alpha))
                print("alpha角度(弧度): {:.6f}".format(alpha))
                print("alpha角度(度): {:.6f}".format(math.degrees(alpha)))
                print("已发布到 /fire_angle 话题")
                print("=" * 50)
    
    def run(self):
        rospy.loginfo("四元方程求解器已启动，等待 /xy_distance 话题数据...")
        rospy.spin()

if __name__ == '__main__':
    try:
        solver = QuarticEquationSolver()
        solver.run()
    except rospy.ROSInterruptException:
        pass
