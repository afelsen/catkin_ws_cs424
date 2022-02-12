#!/usr/bin/env python

import rospy

from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Kill

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random

class Runner:
    def __init__(self):
        rospy.wait_for_service('/spawn')
        self.spawn_runner = rospy.ServiceProxy('/spawn', Spawn)
        self.spawn_runner(7,1,0,"runner")

        self.linear_velocity = 1
        
        self.velocity_publisher = rospy.Publisher('/runner/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
        
    def run(self):

        self.vel_msg.linear.x = self.linear_velocity
        
        while True:
            anglular_velocity = random.random()*2 - 1

            self.vel_msg.angular.z = anglular_velocity

            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()

            

            while t1-t0 < 2:
                t1=rospy.Time.now().to_sec()
                self.velocity_publisher.publish(self.vel_msg)
        

if __name__ == "__main__":
    rospy.init_node('run')

    runner = Runner()
    runner.run()

    rospy.spin()