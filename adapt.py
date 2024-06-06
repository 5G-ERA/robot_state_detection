#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus
from datetime import datetime
import time

class MapExplorationClient(Node):
    def __init__(self, map_width, map_height, exploration_duration):
        super().__init__('map_exploration_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_width = map_width
        self.map_height = map_height
        self.exploration_duration = exploration_duration
        self.goal_timeout = 150  # 5 minutes timeout for each goal
        self.goal_timeout_timer = None
        self.current_goal_handle = None

        self.map_finished_publisher = self.create_publisher(Bool, '/map_finished', 10)
        self.exploration_timer = None
        self.do_not_continue = False
        self.counter = 0

    def publish_map_finished(self, is_finished):
        msg = Bool()
        msg.data = is_finished
        self.map_finished_publisher.publish(msg)

    def exploration_timer_callback(self):        
        if self.exploration_timer:
            self.exploration_timer.cancel()
        self.do_not_continue = True

    def start_exploration_timer(self):
        self.start_time = datetime.now()
        print("Start time is: ", self.start_time)
        self.exploration_timer = self.create_timer(self.exploration_duration, self.exploration_timer_callback)
        self.publish_map_finished(False)  # Indicate exploration is starting

    def goal_timeout_callback(self):
        if self.current_goal_handle:
            self.get_logger().info('Goal timeout reached, cancelling current goal...')
            self.current_goal_handle.cancel_goal_async()
        self.cancel_goal_timeout_timer()

    def start_goal_timeout_timer(self):
        self.cancel_goal_timeout_timer()  # Cancel any existing goal timeout timer
        self.goal_timeout_timer = self.create_timer(self.goal_timeout, self.goal_timeout_callback)

    def cancel_goal_timeout_timer(self):
        if self.goal_timeout_timer:
            self.goal_timeout_timer.cancel()
            self.goal_timeout_timer = None

    def movebase_client(self):
        self.start_exploration_timer()

        goal_positions = [
            (-8.88, -5.12),
            (-12.2, -6.12),
            (-16.8, -4.19),
            (-21.1, -4.94),
            (-20.9, -1.18),
            (-14.2, -1.77),
            (-15.5, -9.9),
            (-11.5, 1.55),
            (-20.2, 2.92),
            (-22.5, 3.33),
            (-23.3, -4.44)
        ]

        for goal_position in goal_positions:
            if self.do_not_continue or not rclpy.ok():
                break

            if not self.client.wait_for_server(timeout_sec=10.0):
                self.get_logger().info('Action server not available, waiting...')
                continue

            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'robot/map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x, goal_msg.pose.position.y = goal_position
            goal_msg.pose.orientation.w = 1.0

            if self.current_goal_handle:
                # Ensure to cancel any ongoing goal
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None

            send_goal_future = self.client.send_goal_async(NavigateToPose.Goal(pose=goal_msg))
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle:
                self.get_logger().info("Goal was rejected by server")
                continue

            self.current_goal_handle = goal_handle
            self.start_goal_timeout_timer()  # Start the goal timeout timer


            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            current_time = time.strftime("%H:%M:%S", time.localtime())
            self.counter += 1


            result = get_result_future.result().result
            if result:
                pass
            else:
                self.get_logger().info(f'Goal execution failed or was cancelled! Goal position: {goal_position}' + str(current_time) + ' ' + str(self.counter))
            


            status = get_result_future.result().status
            status2 = ""
            if status == GoalStatus.STATUS_SUCCEEDED:
                #self.get_logger().info('Goal succeeded!')
                status2 = 'Goal succeeded!'
                self.reset_values()
            elif status == GoalStatus.STATUS_CANCELED:
                #self.get_logger().info('Goal not completed cancelled.')
                status2 = 'Goal not completed cancelled.'
                self.reset_values()
            elif status  == GoalStatus.STATUS_ABORTED:
                #self.get_logger().info('Goal not completed aborted.')
                status2 = 'Goal not completed aborted.'
                self.reset_values()
            else:
                #self.get_logger().info("Goal complete: 0%")
                status2 = 'Goal not completed.'

                self.reset_values()
            self.get_logger().info(f'Goal' + str(self.counter) + '; ' + str(status2) + str(goal_position) + str(current_time))



            

            self.current_goal_handle = None
            self.cancel_goal_timeout_timer()  # Clean up the goal timeout timer
            

        if self.do_not_continue or not rclpy.ok():
            self.publish_map_finished(True)  # Indicate exploration is finished

    def reset_values(self):        
        self.countt = False
        self.recoveries = -1
        self.total_sum_ = 0
        self.current_sum_ = 0
        self.buffer_current_sum = 0
        self.remain_path_sum_ = 0

def main(args=None):
    rclpy.init(args=args)
    map_exploration_client = MapExplorationClient(map_width=5, map_height=7, exploration_duration = 1200)
    try:
        map_exploration_client.movebase_client()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown ROS client libraries
        map_exploration_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
