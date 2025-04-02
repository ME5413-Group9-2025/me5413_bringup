import subprocess

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import rospy
from std_srvs.srv import SetBool, Empty
from tf.transformations import quaternion_from_euler
import math

bridge_length = 5

waypoints = [
    (22.0, 0.0, -math.pi / 2),
    (19.0, -22.0, 3 * math.pi / 4)
]


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
        print(f"Goal finished: {result}")
        return result


if __name__ == "__main__":
    rospy.init_node("scheduler", anonymous=True)
    scheduler = Scheduler()
    scheduler.schedule()
