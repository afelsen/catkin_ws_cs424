#!/usr/bin/env python
import rospy
from actionlib_msgs.msg import GoalStatusArray

from geometry_msgs.msg import PoseStamped

import time

class NavBot:
    def __init__(self):
        rospy.init_node('nav_bot_node')
        self.target_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        self.navigation_status_subsubscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, self.update_status)
        self.status = None
        
    
    def update_status(self, status):
        self.status = status
    
    def four_room_navigation(self):
        # top_right = MoveBaseActionGoal()
        # top_right.header = Header()
        # top_right.goal_id = GoalID()
        # top_right.goal = MoveBaseGoal()
        # top_right.goal_id = 1
        # top_right.goal.target_pose.header.frame_id = "map"
        # top_right_pose = Pose()
        # top_right_pose.position.x = 4.39270353317
        # top_right_pose.position.y = -1.11814343929
        # top_right.goal.target_pose.pose = top_right_pose

        # top_left = MoveBaseActionGoal()
        # top_left_pose = Pose()
        # top_left_pose.position.x = 5.73903942108
        # top_left_pose.position.y = 2.37320065498
        # top_left.goal.target_pose.pose = top_left_pose

        # bottom_right = MoveBaseActionGoal()
        # bottom_right_pose = Pose()
        # bottom_right_pose.position.x = -7.03973436356
        # bottom_right_pose.position.y = 3.03496026993
        # bottom_right.goal.target_pose.pose = bottom_right_pose

        # bottom_left = MoveBaseActionGoal()
        # bottom_left_pose = Pose()
        # bottom_left_pose.position.x = -6.40079641342
        # bottom_left_pose.position.y = -0.775853872299
        # bottom_left.goal.target_pose.pose = bottom_left_pose


        top_right = PoseStamped()
        top_right.header.frame_id = 'map'
        top_right.pose.position.x = 4.39270353317
        top_right.pose.position.y = -1.11814343929
        top_right.pose.orientation.w = 1

        top_left = PoseStamped()
        top_left.header.frame_id = 'map'
        top_left.pose.position.x = 5.73903942108
        top_left.pose.position.y = 2.37320065498
        top_left.pose.orientation.w = 1

        bottom_left = PoseStamped()
        bottom_left.header.frame_id = 'map'
        bottom_left.pose.position.x = -7.03973436356
        bottom_left.pose.position.y = 3.03496026993
        bottom_left.pose.orientation.w = 1

        bottom_right = PoseStamped()
        bottom_right.header.frame_id = 'map'
        bottom_right.pose.position.x = -6.40079641342
        bottom_right.pose.position.y = -0.775853872299
        bottom_right.pose.orientation.w = 1


        rate = rospy.Rate(10)  # 10hz

        while self.target_publisher.get_num_connections() < 1:
            rate.sleep()
            print("sleeping")
        print("awake!")
        time.sleep(2)
        all_targets = [top_right, top_left, bottom_right, bottom_left]
        for target in all_targets:
            self.target_publisher.publish(target)

            while not self.status or len(self.status.status_list) == 0 or self.status.status_list[0].status != 1:
                rate.sleep()

            while self.status.status_list[0].status == 1:
                rate.sleep()

            
        rospy.spin()
        # rospy.spin()

        
        # for i in range(4):
        #     if not self.status or self.status == 3:
        #         self.target_publisher.publish(all_targets[i])
        #     else:
        #         raise Exception("Something went wrong with the navigation!")
        #     while (self.status != 1):
        #         time.sleep(1)
        #     while (self.status == 1):
        #         time.sleep(1)
        

if __name__ == "__main__":
    bot = NavBot()

    bot.four_room_navigation()

    