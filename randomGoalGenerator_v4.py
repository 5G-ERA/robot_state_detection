#!/usr/bin/env python3
from math import sqrt
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
import random
from datetime import datetime
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus
from std_msgs.msg import String


class MapExplorationClient(Node):
    def __init__(self, map_width, map_height,exploration_duration):
        super().__init__('map_exploration_client')


        #self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_width = map_width
        self.map_height = map_height
        # Set a duration for the map exploration phase, e.g., 300 seconds (5 minutes)
        self.exploration_duration = exploration_duration
        self.get_logger().info(f"Map_width: {self.map_width}", once = True)
        self.get_logger().info(f"Map_height: {self.map_height}", once = True)
        self.get_logger().info(f"Random exploration duration: {self.exploration_duration}", once = True)
        self.map_finished_publisher = self.create_publisher(Bool, '/map_finished', 10)
        self.exploration_timer = None
        self.do_not_continue = False
        self.path_subscriber = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10)
        
        # Creating subscriber which will receive information of signal strength as colour   
        self.localization_precision_subscriber = self.create_subscription(String, "/localization_precision",self.localization_precision,1)
        
        self.path_subscriber  # Prevent unused variable warning
        self.countt= False
        self.total_poses = 0
        self.current_position = None
        self.position = None
        self.last_pose = None
        self.recoveries = -1
        self.current_pose = None
        self.total_sum_ = 0
        self.current_sum_ = 0
        self.remain_path_sum_ = 0
        self.buffer_current_sum = 0
        self.length_current_buffer_sum = 0.5
        self.keep_distance = 0
        self.localization = None

    def localization_precision(self, msg):

        if msg.data != self.localization:                

            self.get_logger().info("Localization precizion changed " + str(msg.data))

            if msg.data == "GOOD":
                self.get_logger().info("GOOD")
            elif msg.data == "POOR":
                self.get_logger().info("POOR")
            elif msg.data == "LOST":
                self.get_logger().info("LOST")
            else:
                self.get_logger().info("UNKNOWN")
                msg.data = "UNKNOWN"
            self.localization = msg.data


    def path_callback(self, msg):
        if self.countt == False:
            self.countt = True
            self.total_poses = len(msg.poses)
            if self.total_poses == 0:
                self.get_logger().warn("Threshold cannot be zero.")
                self.reset_values()
                return
            if self.total_poses > 0:
                poses_count=len(msg.poses)
                i = 0
                
                for i in range(0, poses_count-1):
                    self.total_sum_ += sqrt(pow((msg.poses[i+1].pose.position.x - msg.poses[i].pose.position.x),2) + pow((msg.poses[i+1].pose.position.y - msg.poses[i].pose.position.y), 2))
                    self.remain_path_sum_ = self.total_sum_
                    self.keep_distance = self.total_sum_
                    
                self.last_pose = msg.poses[-1].pose                
                self.position = self.last_pose.position
                self.current_pose = msg.poses[0].pose.position
                self.get_logger().info(f"Started goal from (x: {self.current_pose.x:.2f}, y: {self.current_pose.y:.2f}, z: {self.current_pose.z:.2f})")
                self.get_logger().info(f"Moving towards goal (x: {self.position.x:.2f}, y: {self.position.y:.2f}, z: {self.position.z:.2f}")
                self.get_logger().info(f"Started goal with path length: {self.total_sum_:.2f}.")
                self.movebase_client()
        else:

            if self.total_poses == 0:
                self.get_logger().warn("Threshold cannot be zero.")
                self.reset_values()
                return
                
            poses_count=len(msg.poses)
            
            for i in range(0, poses_count-1):
                self.current_sum_ += sqrt(pow((msg.poses[i+1].pose.position.x - msg.poses[i].pose.position.x),2) + pow((msg.poses[i+1].pose.position.y - msg.poses[i].pose.position.y), 2))

            remain = self.remain_path_sum_ - self.current_sum_

            self.current_sum_ = 0

            if remain > 0.5:
                #i = 0
                self.remain_path_sum_ = 0

                for i in range(0, poses_count-1):
                    self.remain_path_sum_ += sqrt(pow((msg.poses[i+1].pose.position.x - msg.poses[i].pose.position.x),2) + pow((msg.poses[i+1].pose.position.y - msg.poses[i].pose.position.y), 2))

                self.current_pose = msg.poses[0].pose.position
                self.get_logger().info(f"Localization precision: {self.localization}")
                self.get_logger().info(f"Moved towards goal (x: {self.current_pose.x:.2f}, y: {self.current_pose.y:.2f}, z: {self.current_pose.z:.2f})")
                self.get_logger().info(f"Moved towards goal by meters (x: {(remain):.2f})")
                percentage = (poses_count* 100) / self.total_poses
                self.get_logger().info(f" Percentage remain to reach the goal: %.2f%%" % percentage)
                self.get_logger().info(f" Lenght remain to reach the goal: {self.remain_path_sum_:.2f})")
                #self.remain_path_sum_ = 0
                remain = 0

    def publish_map_finished(self, is_finished):
        msg = Bool()
        msg.data = is_finished
        self.map_finished_publisher.publish(msg)
    def exploration_timer_callback(self):
        # Stop the timer if it's no longer needed
        if self.exploration_timer:
            self.exploration_timer.cancel()
            self.do_not_continue = True

    def start_exploration_timer(self):
        self.start_time = datetime.now()
        print("Start time is: ", self.start_time)
        self.exploration_timer = self.create_timer(self.exploration_duration, self.exploration_timer_callback)
        self.publish_map_finished(False)  # Indicate exploration is starting
    def movebase_client(self):
        self.start_exploration_timer()
        while rclpy.ok() and not self.do_not_continue:
            if not self.navigate_to_pose_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().info('Action server not available, waiting...')
                continue
            
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()            
            goal_msg.pose.position.x = self.position.x
            goal_msg.pose.position.y = self.position.y
            goal_msg.pose.orientation.w = 1.0

            self.get_logger().info(f"Sending goal: x={goal_msg.pose.position.x:.2f}, y={goal_msg.pose.position.y:.2f}")

            send_goal_future = self.navigate_to_pose_client.send_goal_async(
                NavigateToPose.Goal(pose=goal_msg), 
                feedback_callback=self.feedback_callback
            )

            # Here's the corrected line to add the done callback for goal response
            send_goal_future.add_done_callback(self.goal_response_callback)

            # The loop should only send one goal and then exit. The while loop isn't needed if you're processing one goal at a time.
            break  # Remove this if you intend to send multiple goals in a sequence without waiting for one to finish before sending the next.

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.reset_values()
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)



    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(" Percentage remain to reach the goal: 0%")
            self.get_logger().info('Goal succeeded!')
            self.reset_values()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(" Percentage remain to reach the goal: 0%")
            self.get_logger().info('Goal not completed cancelled.')
            self.reset_values()
        elif status  == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(" Percentage remain to reach the goal: 0%")
            self.get_logger().info('Goal not completed aborted.')
            self.reset_values()
        else:
            self.get_logger().info(" Percentage remain to reach the goal: 0%")
            #self.get_logger().info(f'Goal finished with status: {status}')
            self.reset_values()
    def feedback_callback(self, feedback_msg):
        
        
        #print(feedback_msg)
        feedback = feedback_msg.feedback
        nr_of_recoveries = feedback.number_of_recoveries

        #distance_remaining = feedback.distance_remaining

        if nr_of_recoveries != self.recoveries:
            self.recoveries = nr_of_recoveries
            # Log or process the feedback here
            self.get_logger().info(f'Number of recoveries goal: {self.recoveries}')
            #self.get_logger().info(f'Received feedback: ...')  # Customize this

    def reset_values(self):        
        self.countt = False
        self.recoveries = -1
        self.total_sum_ = 0
        self.current_sum_ = 0
        self.buffer_current_sum = 0
        self.remain_path_sum_ = 0


def main(args=None):
    rclpy.init(args=args)
    map_exploration_client = MapExplorationClient(map_width=2.24, map_height=2.24, exploration_duration = 120)
    rclpy.spin(map_exploration_client)
if __name__ == '__main__':
    main()