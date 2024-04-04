import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import threading
import queue
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult



from simulation_interfaces.action import Move

class MoveActionServer(Node):

    def __init__(self):
        super().__init__('move_action_server')
        self.navigator = BasicNavigator()
        self._action_server = ActionServer(
            self,
            Move,
            'move',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'Executing goal with priority {goal_handle.request.priority}')
        feedback_msg = Move.Feedback()

        self.move_to(goal_handle.request.x, goal_handle.request.y)
        
        # only succeed if the robot is at the goal
        goal_handle.succeed()
        result = Move.Result()
        result.success = True
        self.get_logger().info('Goal succeeded!')
        return result

    
    def move_to(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        # add orientation

        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            self.get_logger().info('task still running')
            time.sleep(1.0)
        return


def main(args=None):
    rclpy.init(args=args)
    move_action_server = MoveActionServer()
    rclpy.spin(move_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
