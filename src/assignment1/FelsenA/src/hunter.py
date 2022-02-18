#!/usr/bin/env python

import math
import rospy

from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Kill

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from turtlesim.msg import Pose

class Hunter:
    def __init__(self):
        rospy.wait_for_service('/spawn')
        spawn_hunter = rospy.ServiceProxy('/spawn', Spawn)
        spawn_hunter(1,1,0,"hunter")

        self.linear_velocity = 1
        
        self.velocity_publisher = rospy.Publisher('/hunter/cmd_vel', Twist, queue_size=10)
        self.kill_publisher = rospy.Publisher('/hunter/kill_runner', Bool, queue_size=1)
        
        self.vel_msg = Twist()

        self.vel_msg.linear.x = self.linear_velocity

        self.hunter_pose_subscriber = rospy.Subscriber('/hunter/pose', Pose, self.update_hunter_pose)
        self.runner_pose_subscriber = rospy.Subscriber('/runner/pose', Pose, self.update_runner_pose)

        self.hunter_pose = Pose(1, 1, 0, 0, 0)

        self.runner_pose = Pose(1, 1, 0, 0, 0)

        self.max_angular_velocity = 5

    def get_desired_angle(self):
        x_dist = self.hunter_pose.x - self.runner_pose.x
        y_dist = self.hunter_pose.y - self.runner_pose.y
        angle = math.atan2(y_dist, x_dist)  # Comes in radians
        return angle      

    def runner_hunter_distance(self):
        return math.sqrt((self.hunter_pose.x - self.runner_pose.x) ** 2 + (self.hunter_pose.y - self.runner_pose.y) ** 2)

    def chase(self):
        while True:

            #self.hunter_pose.position.x, y or z
            current_angle = self.hunter_pose.theta
            desired_angle = self.get_desired_angle()

            # Between -1 and 1
            direction = 0
            if desired_angle-current_angle != 0:
                direction = -1*(desired_angle-current_angle)/abs(desired_angle-current_angle)

            angular_velocity_magnitude = abs(desired_angle-current_angle)/math.pi
            if angular_velocity_magnitude > 1:
                angular_velocity_magnitude -= 2
        
            self.vel_msg.angular.z = direction * angular_velocity_magnitude * self.max_angular_velocity
            self.velocity_publisher.publish(self.vel_msg)
            
            if self.runner_hunter_distance() < 1:
                self.kill_publisher.publish(True)


    def update_hunter_pose(self, coord):
        self.hunter_pose = coord

    def update_runner_pose(self, coord):
        self.runner_pose = coord


if __name__ == "__main__":
    rospy.init_node('hunt')

    rospy.wait_for_service('/kill')
    kill_original = rospy.ServiceProxy('/kill', Kill)
    kill_original('turtle1')
    
    
    hunter = Hunter()
    hunter.chase()

    rospy.spin()