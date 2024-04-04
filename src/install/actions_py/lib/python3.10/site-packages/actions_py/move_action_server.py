import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time

from simulation_interfaces import Move

class MoveActionServer(Node):

    def __init__(self):
        super().__init__('move_action_server')
        self._action_server = ActionServer(
            self,
            Move,
            'move',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        # Simulate action execution
        feedback_msg = Move.Feedback()
        feedback_msg.feedback = 0.0
        for _ in range(10):
            feedback_msg.feedback += 10.0
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.feedback))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        goal_handle.succeed()

        result = Move.Result()
        result.result = feedback_msg.feedback
        return result

def main(args=None):
    rclpy.init(args=args)
    move_action_server = MoveActionServer()
    rclpy.spin(move_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
