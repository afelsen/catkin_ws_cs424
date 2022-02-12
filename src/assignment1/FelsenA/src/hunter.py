#!/usr/bin/env python


import rospy

from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Kill

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# def handle_spawn(s):
#     print("TESTING")
#     s.x = 1
#     s.y = 2
#     s.theta = 0
#     return s

def huntedPose(data):
    pass

class Hunter:
    def __init__(self):
        rospy.wait_for_service('/spawn')
        spawn_hunter = rospy.ServiceProxy('/spawn', Spawn)
        spawn_hunter(1,1,0,"hunter")



if __name__ == "__main__":
    rospy.init_node('hunt')

    rospy.wait_for_service('/kill')
    kill_original = rospy.ServiceProxy('/kill', Kill)
    kill_original('turtle1')
    
    
    hunter = Hunter()

    rospy.spin()