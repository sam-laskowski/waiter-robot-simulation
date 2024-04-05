import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import threading
import queue
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from simulation_interfaces.action import TakeOrder, DeliverFood


class MoveActionServer(Node):

    def __init__(self):
        super().__init__('move_action_server')
        self.navigator = BasicNavigator()
        self.take_order_action_server = ActionServer(
            self,
            TakeOrder,
            'take_order',
            self.take_order_callback
        )

    async def take_order_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'Executing goal with priority {goal_handle.request.priority}')
        feedback_msg = TakeOrder.Feedback()

        self.move_to(goal_handle.request.x, goal_handle.request.y)
        
        # only succeed if the robot is at the goal
        goal_handle.succeed()
        result = TakeOrder.Result()
        result.result = "Order Sent"
        result.table_number = goal_handle.request.table_number
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
            # self.get_logger().info('task still running')
            time.sleep(1.0)
        return


def main(args=None):
    rclpy.init(args=args)
    move_action_server = MoveActionServer()
    rclpy.spin(move_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
