#!/usr/bin/env python

import rospy
import time

from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Kill

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
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

        self.kill_subscriber = rospy.Subscriber('/hunter/kill_runner', Bool, self.kill)

        rospy.wait_for_service('/kill')
        self.kill_runner = rospy.ServiceProxy('/kill', Kill)
        
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
    
    def kill(self, die):
        if die.data:
            self.kill_runner('runner')
            self.spawn_runner(random.random()*9,random.random()*9,0,"runner")
            time.sleep(2)
        

if __name__ == "__main__":
    rospy.init_node('run')

    runner = Runner()
    runner.run()

    rospy.spin()