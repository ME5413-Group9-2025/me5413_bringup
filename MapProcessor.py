import random
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D

class MapProcessor:
    def __init__(self):

        self.map_data = None
        self.map_resolution = 0.05  # 默认值，实际从地图获取
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)
        
        while not rospy.is_shutdown() and self.map_data is None:
            rospy.loginfo("Waiting for map data...")
            rospy.sleep(1)

    def map_callback(self, msg):
        self.map_data = msg
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        rospy.loginfo("Map data received")

    def get_block_bounds(self, block_num, cols, rows):

        block_width = 20/cols  # 20/4=5
        block_height = 10/rows # 10/2=5

        row = (block_num - 1) // cols
        col = (block_num - 1) % cols

        y_max = -2.5 - col * block_width
        y_min = y_max - block_width

        x_min = 9 + row * block_height
        x_max = x_min + block_height

        return (x_min, x_max, y_min, y_max)

    def generate_random_point(self, block_num, cols, rows):

        x_min, x_max, y_min, y_max = self.get_block_bounds(block_num, cols, rows)
        return (
            random.uniform(x_min, x_max),
            random.uniform(y_min, y_max)
        )

    def is_obstacle_free(self, x, y, radius=0.5):
        if self.map_data is None:
            return False

        
        mx = int((x - self.map_origin_x) / self.map_resolution)
        my = int((y - self.map_origin_y) / self.map_resolution)

        
        cells = int(radius / self.map_resolution)
        for dx in range(-cells, cells+1):
            for dy in range(-cells, cells+1):
                if (dx*dx + dy*dy) > cells*cells: 
                    continue
                
                check_x = mx + dx
                check_y = my + dy
                
                if 0 <= check_x < self.map_width and 0 <= check_y < self.map_height:
                    index = check_y * self.map_width + check_x
                    if self.map_data.data[index] > 50: 
                        return False
        return True
    
    def generate_order(self, cols, rows):
    # 生成地图
        matrix = [[i + j * cols + 1 for i in range(cols)] for j in range(rows)]
        result = []

        for col in reversed(range(cols)):
            if (cols - col - 1) % 2 == 0:
                # 从上往下取
                for row in range(rows):
                    result.append(matrix[row][col])
            else:
                # 从下往上取
                for row in reversed(range(rows)):
                    result.append(matrix[row][col])
    
        return result


    def process_all_blocks(self, cols, rows):
        result = []
        processing_order = self.generate_order(cols, rows)
        
        for block_num in processing_order:
            rospy.loginfo(f"Processing block {block_num}")
            success = False
            attempts = 0
            
            while not success and attempts < 100:
                x, y = self.generate_random_point(block_num, cols, rows)
                if self.is_obstacle_free(x, y):
                    rospy.loginfo(f"Found valid point: ({x:.2f}, {y:.2f})")
                    # self.publish_goal(x, y)
                    success = True
                attempts += 1
                if success == True:
                    result.append([block_num, (x, y)])
            if not success:
                rospy.logwarn(f"Failed to find valid point in block {block_num}")
        return result

    def process_a_block(self, num, cols, rows):

        processing_order = self.generate_order(cols, rows)
        
        for block_num in processing_order:
            if block_num != num:
                continue
            rospy.loginfo(f"Processing block {block_num}")
            success = False
            attempts = 0
            
            while not success and attempts < 100:
                x, y = self.generate_random_point(block_num, cols, rows)
                if self.is_obstacle_free(x, y):
                    rospy.loginfo(f"Found valid point: ({x:.2f}, {y:.2f})")
                    success = True
                attempts += 1
                if success == True:
                    return x, y
            
            if not success:
                rospy.logwarn(f"Failed to find valid point in block {block_num}")
        return None, None