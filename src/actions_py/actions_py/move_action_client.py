import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Int32
from simulation_interfaces.action import TakeOrder, DeliverFood, TakeBill
import random
import queue
from datetime import datetime
from nav_msgs.msg import Odometry
import math

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

        # subscribe to /odom topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.goals_queue = queue.PriorityQueue()
        #self.distance_based_goals_queue = queue.PriorityQueue()
        
        self.robot_busy = False

        self.start_time = datetime.now()
        # initial action priorities
        self.initial_action_priority = {"Take Order": 5.0, "Deliver Food": 5.0, "Take Bill": 5.0}

        # distance importance multiplier
        self.distance_factor_multiplier = 0.5
    
    def odom_callback(self, msg):
        # get the current position of the robot
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

    def table_ready_to_order_callback(self, msg):
        x, y = coords.get(msg.data)
        
        #priority logic to be changed
        #prio = random.uniform(1.0, 10.0)
        # add increased priority for earlier requests
        prio = self.initial_action_priority["Take Order"]
        prio += 1.0/(datetime.now() - self.start_time).total_seconds()

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

        x1, y1 = coords.get(7) # kitchen coords
        x, y = coords.get(table_number)
        #priority logic to be changed
        #prio = random.uniform(1.0, 10.0)
        prio = self.initial_action_priority["Deliver Food"]
        prio += 1.0/(datetime.now() - self.start_time).total_seconds()

        self.get_logger().info(f'Queuing a deliver food request for table {msg.data} at ({x}, {y}) with priority {prio}')
        goal_msg = DeliverFood.Goal()
        goal_msg.x1 = x1
        goal_msg.y1 = y1
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.priority = prio
        goal_msg.table_number = table_number
        goal_msg.action_type = "Deliver Food"
        goal_msg.start_time = datetime.now().isoformat()


        self.goals_queue.put((-prio, goal_msg)) # negative prio to get highest priority first

        self.try_send_next_goal()
    
    def bill_request_callback(self, msg):
        table_number = int(msg.data)
        x, y = coords.get(table_number)
        #priority logic to be changed
        #prio = random.uniform(1.0, 10.0)
        prio = self.initial_action_priority["Take Bill"]
        prio += 1.0/(datetime.now() - self.start_time).total_seconds()

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
            self.distance_based_priority()
            priority, goal_msg = self.goals_queue.get()
            self.send_goal(goal_msg)
    
    def distance_based_priority(self):
        # get the distance of the robot from each action location
        # iterate over the queue
        # recalculate the priority based on the distance
        # sort the queue based on the updated priority
        static_x = self.current_x
        static_y = self.current_y
        self.action_store = {}
        self.get_logger().info(f'size: {self.goals_queue.qsize()}')
        for i in range(0, self.goals_queue.qsize()):
            priority, goal_msg = self.goals_queue.get()
            x = goal_msg.x
            y = goal_msg.y
            distance = ((static_x - x)**2 + (static_y - y)**2)**0.5
            goal_msg.priority = priority - self.distance_factor_multiplier*distance
            self.get_logger().info(f'Updated priority for table {goal_msg.table_number} is {goal_msg.priority} with distance {distance}')
            self.action_store[goal_msg.priority] = goal_msg

        for prio, goal_msg in self.action_store.items():
            self.goals_queue.put((-prio, goal_msg))
        return

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
        self.get_logger().info(f'future result: {future.result()}')
        table_number = future.result().result.table_number
        result = future.result().result.result
        get_start_time = future.result().result.start_time
        start_time = datetime.fromisoformat(get_start_time)
        
        completion_time = datetime.now()
        time_taken = (completion_time - start_time).total_seconds()
        # the longer it takes to complete the task, score decreases exponentially
        score_gained = self.calculate_score(result, time_taken)
        self.get_logger().info(f'Score gained: {score_gained}')
        self.score += score_gained
        if result == "Order Sent":
            self.get_logger().info(f'Order complete for table: {table_number}')
            self.order_completed_publisher.publish(String(data=str(table_number)))
            
        elif result == "Food Delivered":
            self.get_logger().info(f'Food delivered for table: {table_number}')
            self.food_delivery_completed_publisher.publish(String(data=str(table_number)))
            
        else: # Bill Taken
            self.get_logger().info(f'Bill taken for table: {table_number}')
            self.bill_complete_publisher.publish(String(data=str(table_number)))

        self.get_logger().info(f'Current Score: {self.score}')
        self.robot_busy = False
        self.try_send_next_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback: {0}'.format(feedback))

    # increasing value of k increases the rate of decrease of the score
    def calculate_score(self, action_type, time_taken, k=0.1):
        if action_type == "Take Order":
            return 10 * math.exp(-k*time_taken)
        elif action_type == "Deliver Food":
            return 20 * math.exp(-k*time_taken)
        else:
            return 10 * math.exp(-k*time_taken)
    

def main(args=None):
    rclpy.init(args=args)
    move_action_client = MoveActionClient()
    rclpy.spin(move_action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
