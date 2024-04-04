import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Int32
from simulation_interfaces.action import Move
import random
import queue

coords = {
    1: [-4.5, -2.0], #table1
    2: [-4.5, 1.5], #table2
    3: [0.3, -2.5], #table3
    4: [0.3, 2.5], #table4
    5: [4.5, 0.0] #kitchen
}

class MoveActionClient(Node):

    def __init__(self):
        super().__init__('move_action_client')
        self._action_client = ActionClient(self, Move, 'move')
        self.subscription = self.create_subscription(
            Int32,
            'table_ready_number', #topic to subscribe to
            self.topic_callback,
            10)
        
        self.goals_queue = queue.PriorityQueue()
        self.robot_busy = False

    def topic_callback(self, msg):
        x, y = coords.get(msg.data)

        # add priority logic
        prio = random.uniform(1.0, 10.0)
        
        self.get_logger().info(f'Publishing goal to move to table {msg.data} at ({x}, {y}) with priority {prio}')
        goal_msg = Move.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.priority = prio

        self.goals_queue.put((prio, goal_msg))

        # attempt to send next goal if robot is not busy
        self.try_send_next_goal()

    def try_send_next_goal(self):
        if not self.robot_busy and not self.goals_queue.empty():
            priority, goal_msg = self.goals_queue.get()
            self.send_goal(goal_msg)


    def send_goal(self, goal_msg):
        self.robot_busy = True # robot is now busy
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.robot_busy = False
            self.try_send_next_goal()
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().success
        self.get_logger().info('Result: {0}'.format(result))
        self.robot_busy = False
        self.try_send_next_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback: {0}'.format(feedback))

def main(args=None):
    rclpy.init(args=args)
    move_action_client = MoveActionClient()
    rclpy.spin(move_action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
