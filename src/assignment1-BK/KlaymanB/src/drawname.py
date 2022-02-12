#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen
PI = 3.1415926535897932384626433832795028841971693

class Draw_Name():
    def __init__(self):
        rospy.init_node('draw_name_node')
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()

        self.speed = 4
        self.angular_speed = 60*2*PI/360
        self.reset_vel_msg()

    def draw(self):
        # Draws the B
        self.rotate(100)
        self.move_forward(8)

        # At Top of the B
        self.rotate(90, -1)
        self.move_forward(2)
        self.rotate(115, -1)
        self.move_forward(3)

        # At Middle of the B
        self.rotate(75)
        self.move_forward(4)
        self.rotate(90, -1)
        self.move_forward(3)

        # Moving From B to K
        self.rotate(120)
        self.move_forward(4)

        self.rotate(90)
        self.move_forward(8)
        self.rotate(180)
        self.move_forward(4)
        self.rotate(45)
        self.move_forward(3)
        self.rotate(180)
        self.move_forward(3)
        self.rotate(90, -1)
        self.move_forward(3)


    def move_forward(self, distance):
        self.vel_msg.linear.x = abs(self.speed)

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while current_distance < distance:
            self.velocity_publisher.publish(self.vel_msg)
            t1=rospy.Time.now().to_sec()

            current_distance = self.speed*(t1-t0)

        self.vel_msg.linear.x = 0

        self.velocity_publisher.publish(self.vel_msg)

    def rotate(self, angle_degrees, direction = 1):
        angle = angle_degrees*2*PI/float(360)

        self.vel_msg.angular.z = direction*self.angular_speed

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle < angle:
            self.velocity_publisher.publish(self.vel_msg)
            t1=rospy.Time.now().to_sec()

            current_angle = self.angular_speed*(t1-t0)

        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def reset_vel_msg(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

def set_to_bottomleft():
    rospy.wait_for_service('turtle1/set_pen')
    pen_service = rospy.ServiceProxy('turtle1/set_pen', SetPen)

    rospy.wait_for_service('turtle1/teleport_absolute')
    teleport_service = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

    pen_service(0,0,0,0,1)
    teleport_service(1,2,0)
    pen_service(0,0,255,0,0)


if __name__ == "__main__":
    set_to_bottomleft()

    drawer = Draw_Name()
    drawer.draw()
