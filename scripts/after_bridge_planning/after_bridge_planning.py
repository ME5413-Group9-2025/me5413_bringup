#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdomSubscriber:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('odom_listener_node')

        # 订阅小车的里程计数据 /jackal_velocity_controller/odom
        self.odom_sub = rospy.Subscriber('/jackal_velocity_controller/odom', Odometry, self.odom_callback)

        # 存储小车的位置
        self.current_pose = None
        self.current_yaw = None
        
        # 定义目标坐标点数组
        self.target_points = [
            [0.3335, 1.5330, -1.3905],
            [4.2213, 1.9587, -1.2424],
            [8.4716, 1.9654, -1.4668],
            [11.9221, 1.4931, -0.9947]
        ]

    def odom_callback(self, msg):
        # 获取当前的小车位置信息
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # 使用四元数转换为欧拉角（获取yaw角度）
        euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw = euler[2]  # 获取 yaw 角度

        # 打印当前的坐标和yaw角度
        rospy.loginfo("Current position -> x: %f, y: %f, yaw: %f", position.x, position.y, yaw)

        # 存储当前位置信息
        self.current_pose = (position.x, position.y)
        self.current_yaw = yaw

        # 输出目标坐标点数组
        rospy.loginfo("Target Points: %s", self.target_points)

    def get_target_points(self):
        # 返回目标坐标点数组
        return self.target_points

    def start(self):
        # 循环等待并处理 ROS 消息
        rospy.spin()

if __name__ == '__main__':
    try:
        odom_subscriber = OdomSubscriber()
        odom_subscriber.start()
    except rospy.ROSInterruptException:
        pass

