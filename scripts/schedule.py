import subprocess

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import rospy
from std_srvs.srv import SetBool, Empty
from tf.transformations import quaternion_from_euler
import math
import random

bridge_length = 5

waypoints = [
    (22.0, 0.0, -math.pi / 2),
    (19.0, -22, 3 * math.pi / 4)
]
exp_points = [
    (14.0, -19.3, 0),
    (19.0, -16.6, math.pi),
    (14.0, -13.9, 0),
    (19.0, -11.2, math.pi),
    (14.0, -8.5, 0),
    (19.0, -5.8, math.pi),
    (14.5, -3.1, math.pi),

    (13.0, -3.1, math.pi),
    (9.0, -5.8, 0),
    (14.0, -8.5, math.pi),
    (9,-11.2, 0),
    (14,-13.9, math.pi),
    (9,-16.6, 0),
    (14,-19.3, math.pi),
    (9, -22, 0)
]

def random_point_on_circle(x, y, radius):
    # 生成一个随机角度（单位是弧度）
    angle = random.uniform(0, 2 * math.pi)
    
    # 计算圆周上的新点坐标
    random_x = x + radius * math.cos(angle)
    random_y = y + radius * math.sin(angle)
    
    return random_x, random_y

def bound_adj(x, y):
    if y < -22.5:
        y = 22.5
    if y > -2.5:
        y = 2.5
    if x > 19:
        x = 19
    if x < 9:
        x = 9
    return x,y

class Scheduler:
    def __init__(self):
        self.exploration_process = None
        self.move_base_client = SimpleActionClient("move_base", MoveBaseAction)
        self.bridge_open_pub = rospy.Publisher("/cmd_open_bridge", Bool, queue_size=1)
        self.bridge_position_sub = rospy.Subscriber("/bridge_position", Pose2D, self.bridge_position_callback)
        self.enable_perception_client = rospy.ServiceProxy("/enable_perception_signal", SetBool)
        self.clear_costmap_client = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        self.move_base_client.wait_for_server()
        print("Scheduler initialized")

    def schedule(self):
        for x, y, yaw in waypoints:
            self.publish_navigation_goal(x, y, yaw)
            self.clear_costmap_client()
            rospy.sleep(2.)

        print("exiting schedule")

    def exp_sch(self):
        for x, y, yaw in exp_points:
            self.publish_navigation_goal(x, y, yaw)
            self.clear_costmap_client()
            rospy.sleep(2.)

        print("explore schedule")

    def enable_exploration_and_perception(self):
        self.enable_perception_client(True)

        self.exploration_process = subprocess.Popen(["roslaunch", "me5413_bringup", "exploration.launch"])
        # self.exploration_process.wait()
        # self.exploration_process.terminate()

    def open_bridge(self):
        self.bridge_open_pub.publish(True)

    def bridge_position_callback(self, bridge_position: Pose2D):
        x, y, yaw = bridge_position.x, bridge_position.y, bridge_position.theta
        self.publish_navigation_goal(x, y, yaw)
        self.open_bridge()
        self.publish_navigation_goal(x, y - bridge_length, yaw)

    def publish_navigation_goal(self, x, y, yaw):
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = x
        move_base_goal.target_pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, yaw)
        move_base_goal.target_pose.pose.orientation.x = quat[0]
        move_base_goal.target_pose.pose.orientation.y = quat[1]
        move_base_goal.target_pose.pose.orientation.z = quat[2]
        move_base_goal.target_pose.pose.orientation.w = quat[3]
        self.move_base_client.send_goal(move_base_goal)
        print("Publishing new goal")
        result = self.move_base_client.wait_for_result()
        while result == False:
            print("waypoint false, generate new point")
            x,y = random_point_on_circle(x,y,1)
            x,y = bound_adj(x,y)
            move_base_goal.target_pose.pose.position.x = x
            move_base_goal.target_pose.pose.position.y = y
            self.move_base_client.send_goal(move_base_goal)
            result = self.move_base_client.wait_for_result()

        print(f"Goal finished: {result}")
        return result


if __name__ == "__main__":
    rospy.init_node("scheduler", anonymous=True)
    scheduler = Scheduler()
    scheduler.schedule()
    scheduler.exp_sch()
