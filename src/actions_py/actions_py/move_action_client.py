import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Int32
from simulation_interfaces.action import TakeOrder, DeliverFood, TakeBill
import random
import queue
from datetime import datetime

coords = {
    1: [0.8, 1.3], #cafe_table1
    2: [0.8, -1.9], #cafe_table2
    3: [-1.0, -3.7], #couch_table
    4: [2.4, 2.8], #sofa_table1
    5: [2.4, -0.4], #sofa_table2
    6: [2.4, -3.6], #sofa_table3
    7: [-1.0, 6.0], #kicthen
}

class MoveActionClient(Node):

    def __init__(self):
        super().__init__('move_action_client')
        self.score = 0

        self.take_order_action_client = ActionClient(self, TakeOrder, 'take_order')
        self.deliver_food_action_client = ActionClient(self, DeliverFood, 'deliver_food')
        self.take_bill_action_client = ActionClient(self, TakeBill, 'take_bill')

        self.order_completed_publisher = self.create_publisher(String, 'order_completed', 10)
        self.food_delivery_completed_publisher = self.create_publisher(String, 'food_delivery_complete', 10)
        self.bill_complete_publisher = self.create_publisher(String, 'bill_completed', 10)

        self.table_ready_subscription = self.create_subscription(
            Int32,
            'table_ready_to_order_number', #topic to subscribe to
            self.table_ready_to_order_callback,
            10)
        
        self.food_delivery_subscription = self.create_subscription(
            String,
            'food_request',
            self.food_request_callback,
            10
        )

        self.bill_request_subscription = self.create_subscription(
            String,
            'bill_request',
            self.bill_request_callback,
            10
        )
        
        self.goals_queue = queue.PriorityQueue()
        self.robot_busy = False


    # all order logic
    def table_ready_to_order_callback(self, msg):
        x, y = coords.get(msg.data)

        # add priority logic
        prio = random.uniform(1.0, 10.0)
        
        self.get_logger().info(f'Queuing an order request at table {msg.data} at ({x}, {y}) with priority {prio}')
        goal_msg = TakeOrder.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.priority = prio
        goal_msg.table_number = msg.data
        goal_msg.action_type = "Take Order"
        goal_msg.start_time = datetime.now().isoformat()

        self.goals_queue.put((-prio, goal_msg)) # negative prio to get highest priority first

        # attempt to send next goal if robot is not busy
        self.try_send_next_goal()

    def food_request_callback(self, msg):
        table_number = int(msg.data)
        self.get_logger().info(f'Food for table {table_number} has been made')
        # add food delivery logic
        x1, y1 = coords.get(7) # kitchen coords
        x2, y2 = coords.get(table_number)
        prio = random.uniform(1.0, 10.0)
        self.get_logger().info(f'Queuing a deliver food request for table {msg.data} at ({x2}, {y2}) with priority {prio}')
        goal_msg = DeliverFood.Goal()
        goal_msg.x1 = x1
        goal_msg.y1 = y1
        goal_msg.x2 = x2
        goal_msg.y2 = y2
        goal_msg.priority = prio
        goal_msg.table_number = table_number
        goal_msg.action_type = "Deliver Food"
        goal_msg.start_time = datetime.now().isoformat()


        self.goals_queue.put((-prio, goal_msg)) # negative prio to get highest priority first

        self.try_send_next_goal()
    
    def bill_request_callback(self, msg):
        table_number = int(msg.data)
        x, y = coords.get(table_number)
        prio = random.uniform(1.0, 10.0)
        self.get_logger().info(f'Queuing a take bill request for table {msg.data} with priority {prio}')
        goal_msg = TakeBill.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.priority = prio
        goal_msg.table_number = table_number
        goal_msg.action_type = "Take Bill"
        goal_msg.start_time = datetime.now().isoformat()


        self.goals_queue.put((-prio, goal_msg))

        self.try_send_next_goal()

    def try_send_next_goal(self):
        if not self.robot_busy and not self.goals_queue.empty():
            priority, goal_msg = self.goals_queue.get()
            self.send_goal(goal_msg)


    def send_goal(self, goal_msg):
        self.robot_busy = True # robot is now busy

        # change logic based on goal type

        if goal_msg.action_type == "Take Order":
            self.take_order_action_client.wait_for_server()            
            self._send_goal_future = self.take_order_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

        elif goal_msg.action_type == "Deliver Food":
            self.deliver_food_action_client.wait_for_server()
            self._send_goal_future = self.deliver_food_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else: # Take Bill
            self.take_bill_action_client.wait_for_server()
            self._send_goal_future = self.take_bill_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.robot_busy = False
            self.try_send_next_goal()
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        table_number = future.result().result.table_number
        result = future.result().result.result
        get_start_time = future.result().result.start_time
        start_time = datetime.fromisoformat(get_start_time)
        
        completion_time = datetime.now()
        time_taken = (completion_time - start_time).total_seconds()
        if result == "Order Sent":
            self.get_logger().info(f'Order complete for table: {table_number}')
            self.order_completed_publisher.publish(String(data=str(table_number)))
            # calculate score
            self.get_logger().info(f'Score recieved: {10/time_taken}')
            self.score += 10/time_taken

        elif result == "Food Delivered":
            self.get_logger().info(f'Food delivered for table: {table_number}')
            self.food_delivery_completed_publisher.publish(String(data=str(table_number)))
            # calculate score
            self.get_logger().info(f'Score recieved: {20/time_taken}')

            self.score += 20/time_taken
        else: # Bill Taken
            self.get_logger().info(f'Bill taken for table: {table_number}')
            self.bill_complete_publisher.publish(String(data=str(table_number)))
            # calculate score
            self.get_logger().info(f'Score recieved: {10/time_taken}')
            self.score += 10/time_taken

        self.get_logger().info(f'Current Score: {self.score}')
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
