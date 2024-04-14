import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import threading
import queue
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from simulation_interfaces.action import TakeOrder, DeliverFood, TakeBill


direction = {
    1: "-x", #cafe_table1
    2: "-x", #cafe_table2
    3: "-x-y", #couch_table
    4: "+x", #sofa_table1
    5: "+x", #sofa_table2
    6: "+x", #sofa_table3
    7: "-y", #kicthen
}

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

        self.deliver_food_action_server = ActionServer(
            self,
            DeliverFood,
            'deliver_food',
            self.deliver_food_callback
        )

        self.take_bill_action_server = ActionServer(
            self,
            TakeBill,
            'take_bill',
            self.take_bill_callback
        )

    async def take_order_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'Executing Take Order: Table {goal_handle.request.table_number}, Priority {goal_handle.request.priority}')
        feedback_msg = TakeOrder.Feedback()
        dir = direction[goal_handle.request.table_number]

        self.move_to(goal_handle.request.x, goal_handle.request.y, dir)
        
        # only succeed if the robot is at the goal
        goal_handle.succeed()
        result = TakeOrder.Result()
        result.result = "Order Sent"
        result.table_number = goal_handle.request.table_number
        result.start_time = goal_handle.request.start_time
        self.get_logger().info('Goal succeeded!')
        return result
    
    async def deliver_food_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'Executing Food Delivery: Table {goal_handle.request.table_number}, Priority {goal_handle.request.priority}')
        feedback_msg = DeliverFood.Feedback()
        dir = direction[goal_handle.request.table_number]

        # go to kitchen then to table
        self.collect_and_deliver_food(goal_handle.request.x1, goal_handle.request.y1, goal_handle.request.x, goal_handle.request.y, dir)
        # only succeed if the robot is at the goal
        goal_handle.succeed()
        result = DeliverFood.Result()
        result.result = "Food Delivered"
        result.table_number = goal_handle.request.table_number
        result.start_time = goal_handle.request.start_time
        self.get_logger().info('Goal succeeded!')
        return result
    
    async def take_bill_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'Executing Take Bill: Table {goal_handle.request.table_number}, Priority {goal_handle.request.priority}')
        feedback_msg = TakeBill.Feedback()
        dir = direction[goal_handle.request.table_number]

        self.move_to(goal_handle.request.x, goal_handle.request.y, dir)
        # only succeed if the robot is at the goal
        goal_handle.succeed()
        result = TakeBill.Result()
        result.result = "Bill Taken"
        result.table_number = goal_handle.request.table_number
        result.start_time = goal_handle.request.start_time
        self.get_logger().info('Goal succeeded!')
        return result

    
    def move_to(self, x, y, direction):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        #change orientation based on direction
        if direction == "-x":
            # add orientation to face towards negative x-axis
            goal_pose.pose.orientation.z = 1.0
            goal_pose.pose.orientation.w = 0.0
        elif direction == "+x":
            # add orientation to face towards positive x-axis
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0
        elif direction == "-x-y":
            # add orientation to face towards negative x-axis and negative y-axis
            goal_pose.pose.orientation.z = 0.707
            goal_pose.pose.orientation.w = 0.707


        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            # self.get_logger().info('task still running')
            time.sleep(1.0)
        return
    
    def collect_and_deliver_food(self, x1, y1, x2, y2, direction):
        kitchen_pose = PoseStamped()
        kitchen_pose.header.stamp = self.get_clock().now().to_msg()
        kitchen_pose.header.frame_id = 'map'
        kitchen_pose.pose.position.x = x1
        kitchen_pose.pose.position.y = y1
        kitchen_pose.pose.position.z = 0.0
        # add orientation to face towards negative y-axis
        kitchen_pose.pose.orientation.z = -0.707
        kitchen_pose.pose.orientation.w = 0.707

        table_pose = PoseStamped()
        table_pose.header.stamp = self.get_clock().now().to_msg()
        table_pose.header.frame_id = 'map'
        table_pose.pose.position.x = x2
        table_pose.pose.position.y = y2
        table_pose.pose.position.z = 0.0

        #change orientation based on direction
        if direction == "-x":
            # add orientation to face towards negative x-axis
            table_pose.pose.orientation.z = 1.0
            table_pose.pose.orientation.w = 0.0
        elif direction == "+x":
            # add orientation to face towards positive x-axis
            table_pose.pose.orientation.z = 0.0
            table_pose.pose.orientation.w = 1.0
        elif direction == "-x-y":
            # add orientation to face towards negative x-axis and negative y-axis
            table_pose.pose.orientation.z = 0.707
            table_pose.pose.orientation.w = 0.707



        self.navigator.goToPose(kitchen_pose)
        while not self.navigator.isTaskComplete():
            # self.get_logger().info('task still running')
            time.sleep(1.0)
        
        self.navigator.goToPose(table_pose)
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
