import rclpy
from rclpy.node import Node
import transforms3d
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from std_msgs.msg import String, Int32

#table1_coords = [-4.5, -2.0] #behind
#table2_coords = [-4.5, 1.5] #behind
#table3_coords = [0.3, -2.5] #left
#table4_coords = [0.3, 2.5] #right
#kitchen_coords = [4.5, 0.0]

coords = {
    1: [-4.5, -2.0], #table1
    2: [-4.5, 1.5], #table2
    3: [0.3, -2.5], #table3
    4: [0.3, 2.5], #table4
    5: [4.5, 0.0] #kitchen
}

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.navigator = BasicNavigator()

        self.subscription_number = self.create_subscription(
            Int32,
            'table_ready_number',
            self.number_listener_callback,
            10
        )

        self.latest_table_number = None # update to queue
    
    def number_listener_callback(self, msg):
        self.get_logger().info('Received table number: "%d"' % msg.data)
        self.latest_table_number = msg.data
        
        self.move_to(coords.get(self.latest_table_number))

    
    def move_to(self, area_coords):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = area_coords[0]
        goal_pose.pose.position.y = area_coords[1]
        goal_pose.pose.position.z = 0.0

        self.navigator.goToPose(goal_pose)
        
        return self.navigator.isTaskComplete()
    


def main(args=None):
    rclpy.init()
    node = RobotController()
    #future = node.move_to(table3_coords)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



    

