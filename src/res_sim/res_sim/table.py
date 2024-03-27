class Table:
    def __init__(self, readyToOrder, location):
        self.readyToOrder = readyToOrder
        self.location = location


import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import random

class TableReadyPublisher(Node):
    def __init__(self):
        super().__init__('table_ready_publisher')
        self.number_publisher = self.create_publisher(Int32, 'table_ready_number', 10)
        timer_period = random.uniform(5.0, 10.0)  # Random time between 5 and 10 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        table_number = random.randint(1,4)
        num_msg = Int32()
        num_msg.data = table_number
        self.number_publisher.publish(num_msg)
        
        self.timer.cancel()  # Cancel the current timer if it's a one-time event

def main(args=None):
    rclpy.init(args=args)
    table_ready_publisher = TableReadyPublisher()

    rclpy.spin(table_ready_publisher)
    table_ready_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()