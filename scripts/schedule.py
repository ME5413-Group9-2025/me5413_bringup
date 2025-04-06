import numpy as np
import math

from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from std_msgs.msg import Bool, Int32MultiArray
import rospy
from std_srvs.srv import SetBool, Empty
from tf.transformations import quaternion_from_euler
from me5413_bringup.srv import SingleDigit
from tf2_ros import Buffer, TransformListener

import sys
from pathlib import Path

package_path = str(Path(__file__).parent.absolute())
sys.path.append(package_path)
from zigzag_sampler import ZigZagSampler

bridge_length = 5


class Scheduler:
    def __init__(self):
        # Global Variable
        self.digit_count = None
        self.exploration_process = None
        self.sample_radius = 1

        # TF tree
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer)

        # ROS Parameter
        self.navigation_waypoints = rospy.get_param("~navigation_waypoints", None)
        self.exploration_waypoints = rospy.get_param("~exploration_waypoints", None)
        self.exploration_bound = rospy.get_param("~exploration_bound", None)
        self.target_waypoints = rospy.get_param("~target_waypoints", None)

        # Sampler
        self.sampler = ZigZagSampler()

        # ROS communication
        self.move_base_client = SimpleActionClient("move_base", MoveBaseAction)
        self.bridge_open_pub = rospy.Publisher("/cmd_open_bridge", Bool, queue_size=10)
        self.bridge_position_sub = rospy.Subscriber("/bridge_pose", PoseStamped, self.bridge_pose_callback)
        self.enable_perception_client = rospy.ServiceProxy("/enable_perception_signal", SetBool)
        self.enable_bridge_client = rospy.ServiceProxy("/enable_bridge_signal", SetBool)
        self.clear_costmap_client = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        self.recognize_digit_client = rospy.ServiceProxy("/recognize_target", SingleDigit)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.digit_count_sub = rospy.Subscriber("/digit_count", Int32MultiArray, self.digit_count_callback)
        self.move_base_client.wait_for_server()
        print("Scheduler initialized")

    def digit_count_callback(self, data):
        self.digit_count = data.data

    def schedule(self):
        rospy.loginfo("Start Navigation")
        for x, y, yaw in self.navigation_waypoints:
            while True:
                result = self.publish_navigation_goal(x, y, yaw)
                rospy.sleep(1.0)
                if result:
                    break
        self.enable_perception_client.call(True)
        rospy.logfatal("Start ZigZag exploration")
        sample_point = []
        cols, rows = 8, 3
        exploration_order = self.sampler.generate_order(cols=cols, rows=rows)
        for block_num in exploration_order:
            if self.sampler.block_not_skip == Falese:
                continue
            x, y = self.sampler.process_single_block(block_num, cols, rows)
            if not x or not y:
                continue
            yaw = math.pi if block_num > cols else 0
            result = self.publish_navigation_goal(x, y, yaw)
            sample_point.append[(x,y)]
            rospy.sleep(1.0)
        self.enable_perception_client.call(False)
        rospy.loginfo("exiting schedule")

    def get_target_waypoints_sequence(self):
        while not self.tf_buffer.can_transform("map", "base_link", rospy.Time()):
            rospy.logwarn("Waiting, can not transform from base_link to map")
            rospy.sleep(0.1)
        robot_pose = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time())
        assert isinstance(robot_pose, TransformStamped)
        y = robot_pose.transform.translation.y
        waypoint_average = sum([point[1] for point in self.target_waypoints]) / len(self.target_waypoints)
        if y >= waypoint_average:
            return self.target_waypoints
        else:
            return self.target_waypoints[::-1]

    def bridge_pose_callback(self, bridge_pose: PoseStamped):
        rospy.loginfo("Receive bridge pose, now navigating to there")
        self.enable_bridge_client.call(False)
        # Navigate to the bridge
        x, y, yaw = bridge_pose.pose.position.x, bridge_pose.pose.position.y, math.pi
        self.publish_navigation_goal(x, y, yaw)
        # Open the bridge
        self.bridge_open_pub.publish(True)
        # Cross the bridge
        cmd_vel = Twist()
        for _ in range(30):
            cmd_vel.linear.x = 1.5
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.sleep(0.1)
        # Planning the waypoints
        target = np.argmax(self.digit_count)
        rospy.logfatal(f"Target digit is {target}")
        waypoints = self.get_target_waypoints_sequence()
        for waypoint in waypoints:
            x, y = waypoint
            self.publish_navigation_goal(x, y, math.pi)
            response = self.recognize_digit_client.call()
            if response.digit == target:
                rospy.logfatal("Found the target, exiting!")
                break
            rospy.sleep(1.0)

    def publish_navigation_goal(self, x, y, yaw, timeout=20):
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
        rospy.loginfo("Publishing new goal")
        finish = self.move_base_client.wait_for_result(timeout=rospy.Duration(timeout))
        state = self.move_base_client.get_state()
        if finish:
            rospy.loginfo(f"Goal finished: {finish}")
            return True
        return False


if __name__ == "__main__":
    rospy.init_node("scheduler", anonymous=True)
    scheduler = Scheduler()
    scheduler.schedule()
    rospy.spin()
