import rclpy
from rclpy.node import Node
import transforms3d
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.navigator = BasicNavigator()
    
    def move_to_table(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0

        self.navigator.goToPose(goal_pose)

        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print('Est time: ' + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')
        else:
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
                return
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

def main(args=None):
    rclpy.init()
    node = RobotController()
    future = node.move_to_table()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



    

