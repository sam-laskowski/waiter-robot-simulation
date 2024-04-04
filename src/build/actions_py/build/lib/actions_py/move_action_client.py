import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from simulation_interfaces import Move

class MoveActionClient(Node):

    def __init__(self):
        super().__init__('move_action_client')
        self._action_client = ActionClient(self, Move, 'move')
        self.subscription = self.create_subscription(
            String,
            'table_ready_number', #topic to subscribe to
            self.topic_callback,
            10)

    def topic_callback(self, msg):
        self.get_logger().info('Heard: "{0}"'.format(msg.data))
        self.send_goal(1.0)  # Example value

    def send_goal(self, position):
        goal_msg = Move.Goal()
        goal_msg.position = position

        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    move_action_client = MoveActionClient()
    rclpy.spin(move_action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
