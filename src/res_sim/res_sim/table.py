import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import random
from rclpy.callback_groups import ReentrantCallbackGroup

class TableReadyPublisher(Node):
    def __init__(self):
        super().__init__('table_ready_publisher')
        self.number_publisher = self.create_publisher(Int32, 'table_ready_to_order_number', 10)
        self.bill_publisher = self.create_publisher(String, 'bill_request', 10)
        self.food_publisher = self.create_publisher(String, 'food_request', 10)
        # timer_period = random.uniform(5.0, 10.0)  # Random time between 5 and 10 seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.callback_group = ReentrantCallbackGroup()
        # customer arrives every 10 seconds
        self.customer_timer = self.create_timer(10.0, self.customer_arrived, callback_group=self.callback_group)

        self.empty_tables = [1, 2, 3, 4, 5, 6]

        self.order_completed = self.create_subscription(
            String,
            'order_completed',
            self.order_completed_callback,
            10
        )

        self.food_recieved = self.create_subscription(
            String,
            'food_delivery_complete',
            self.food_recieved_callback,
            10
        )
        
        self.bill_completed = self.create_subscription(
            String,
            'bill_completed',
            self.bill_completed_callback,
            10
        )

    def order_completed_callback(self, msg):
        table_number = int(msg.data)
        self.get_logger().info(f'Sending order to be made by kitcken for table {table_number}')

        timer_wrapper = [None]
        # food is ready in 20 seconds

        def timer_callback():
            self.cooking(timer_wrapper[0], table_number)
        timer_wrapper[0] = self.create_timer(20.0, timer_callback, callback_group=self.callback_group)
    
    def cooking(self, timer, table_number):
        self.food_ready(table_number)
        timer.cancel()

    def food_ready(self, table_number):
        self.get_logger().info(f'Food ready for delivery for table {table_number}')
        food_msg = String()
        food_msg.data = str(table_number)
        self.food_publisher.publish(food_msg)
        
    

    def food_recieved_callback(self, msg):
        table_number = int(msg.data)
        self.get_logger().info(f'Food recieved for table {table_number}')

        timer_wrapper = [None]

        # table eats for 25 seconds, then requests bill after
        def timer_callback():
            self.eating(timer_wrapper[0], table_number)
        timer_wrapper[0] = self.create_timer(25.0, timer_callback, callback_group=self.callback_group)


    def eating(self, timer, table_number):
        self.request_bill(table_number)
        timer.cancel()

    def request_bill(self, table_number):
        self.get_logger().info(f'Table {table_number} requested bill')
        bill_msg = String()
        bill_msg.data = str(table_number)
        self.bill_publisher.publish(bill_msg)

    def bill_completed_callback(self, msg):
        table_number = int(msg.data)
        self.get_logger().info(f'Bill completed for table {table_number}')
        self.empty_tables.append(table_number)

    def customer_arrived(self):
        # check if any tables are free
        if len(self.empty_tables) == 0:
            self.get_logger().info('All tables are occupied')
            return
        # otherwise assign a random table to the customer
        table_number = random.choice(self.empty_tables)
        self.empty_tables.remove(table_number)

        # send message to the topic
        num_msg = Int32()
        num_msg.data = table_number
        self.number_publisher.publish(num_msg)
        self.get_logger().info(f'Table {table_number} requested order')
    



def main(args=None):
    rclpy.init(args=args)
    table_ready_publisher = TableReadyPublisher()

    rclpy.spin(table_ready_publisher)
    table_ready_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()