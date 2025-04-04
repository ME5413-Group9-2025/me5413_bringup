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
        
        # 定义新的目标坐标点数组
        self.target_points = [
            [41.548987, -11.875592, 1.032409],
            [39.294108, -8.730418, 0.683180],
            [37.494167, -4.747013, 0.620667],
            [36.226067, -1.109040, 0.439622]
        ]

        # 计算目标点的y坐标的平均值
        self.average_y = sum([point[1] for point in self.target_points]) / len(self.target_points)

        # 用于标记是否已读取过一次小车的位置
        self.has_read_position = False

    def odom_callback(self, msg):
        if self.has_read_position:
            return  # 如果已经读取过小车的位置，直接返回

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

        # 判断小车的y坐标与目标点y坐标的平均值的关系
        if position.y < self.average_y:
            # 小车位于目标点y平均值的左边，目标点从左到右输出
            target_points = self.target_points
        else:
            # 小车位于目标点y平均值的右边，目标点从右到左输出
            target_points = self.target_points[::-1]

        # 输出目标坐标点数组
        rospy.loginfo("Target Points: %s", target_points)

        # 设置标志位，表示已经读取过位置
        self.has_read_position = True

    def start(self):
        # 循环等待并处理 ROS 消息
        rospy.spin()

if __name__ == '__main__':
    try:
        odom_subscriber = OdomSubscriber()
        odom_subscriber.start()
    except rospy.ROSInterruptException:
        pass

