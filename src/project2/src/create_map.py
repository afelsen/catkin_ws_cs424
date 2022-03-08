#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
import math
import time

class MapBot:
    def __init__(self):
        rospy.init_node('map_bot_node')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()

        self.speed = .2
        self.angular_speed = .8
        self.wall_range = 1.2
        self.far_wall_range = .8
        self.collision = .15
        self.buffer = .2


        self.laser_scan_subsubscriber = rospy.Subscriber('/scan', LaserScan, self.update_laser_pos)
        self.ranges = [1] * 360

    def move_forward(self, distance, direction=1):
        self.vel_msg.linear.x = direction*abs(self.speed)

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while current_distance < distance:
            self.velocity_publisher.publish(self.vel_msg)
            t1=rospy.Time.now().to_sec()

            current_distance = self.speed*(t1-t0)

        self.vel_msg.linear.x = 0

        self.velocity_publisher.publish(self.vel_msg)
    
    def rotate(self, angle_degrees, direction = 1):
        angle = angle_degrees*2*(math.pi)/float(360)

        self.vel_msg.angular.z = direction*self.angular_speed

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle < angle:
            self.velocity_publisher.publish(self.vel_msg)
            t1=rospy.Time.now().to_sec()

            current_angle = self.angular_speed*(t1-t0)

        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def random_roam(self):
        i = 1
        while(True):
            self.move_forward(random.random())
            self.rotate(random.random()*3)

            i += 1
            if i % 10 == 0:
                self.get_unstuck()

    def update_laser_pos(self, lasers):
        self.ranges = lasers.ranges


    def get_unstuck(self):
        self.move_forward(1, direction = -1)
        self.rotate(90)

    def smart_roam2(self):
        state = 0
        self.move_into_wall()
        while True:
            front = self.ranges[0]
            front_left = self.ranges[45]
            front_right = self.ranges[315]
            #go forward-left if there is no wall in front and the bot is not only against the right wall
            if front > self.wall_range and (not (front_left > self.wall_range and front_right < self.wall_range)):
                self.vel_msg.linear.x = self.speed/2.5
                self.vel_msg.angular.z = -1*self.angular_speed/2.5
            #If there is only an obstacle on the right, continue straight
            elif front > self.wall_range and front_left > self.wall_range and front_right < self.wall_range:
                self.vel_msg.linear.x = self.speed
                self.vel_msg.angular.z = 0
            
            #Otherwise, turn left
            else:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = self.angular_speed

            self.velocity_publisher.publish(self.vel_msg)

            #unstuck
            if front < self.collision:
                self.get_unstuck()

        

    def smart_roam(self):
        # self.ranges at .3 or less means it is at a wall
        self.move_into_wall()
        while True:
            if min(self.ranges[0:45] + self.ranges[315:]) < self.collision:
                print("Collision!")
                self.stop_bot()
                self.fix_collision()
                self.stop_bot()
            
            elif self.ranges[0] < self.wall_range:
                print("Wall ahead!")
                self.stop_bot()
                self.rotate_out_of_wall()
                self.stop_bot()
            
            elif self.ranges[315] > self.far_wall_range:
                print("No Right Wall")
                self.stop_bot()
                self.move_forward(.3)
                self.rotate(90, direction = -1)
                self.move_forward(.3)
                self.stop_bot()
            
            elif max(self.ranges[225:315]) > self.wall_range + self.buffer:
                print("Rotating away from Right Wall")
                self.stop_bot()
                self.rotate_towards_wall()
                self.stop_bot()
            
            
            
            
            self.vel_msg.linear.x = self.speed
            self.velocity_publisher.publish(self.vel_msg)

    def move_into_wall(self):
        self.vel_msg.linear.x = abs(self.speed)

        t0 = rospy.Time.now().to_sec()

        while self.ranges[0] > self.wall_range - self.wall_range*.2:
            print("Moving towards wall")
            print(self.ranges[0], self.wall_range)
            self.velocity_publisher.publish(self.vel_msg)
            t1=rospy.Time.now().to_sec()

        self.vel_msg.linear.x = 0

        self.velocity_publisher.publish(self.vel_msg)

    def rotate_towards_wall(self):
        self.vel_msg.angular.z = self.angular_speed*-1
        self.vel_msg.linear.x = 0

        minimum = min(self.ranges)

        #Rotate until roughly parallel with wall
        while abs(self.ranges[299] - self.ranges[239]) > .02:
            self.velocity_publisher.publish(self.vel_msg)

        self.vel_msg.linear.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def stop_bot(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
        time.sleep(1)

    def rotate_out_of_wall(self):
        # front_dist = self.ranges[0]
        self.vel_msg.angular.z = self.angular_speed
        self.vel_msg.linear.x = 0

        #Rotate until roughly parallel with wall
        while abs(self.ranges[299] - self.ranges[239]) > .02:
            self.velocity_publisher.publish(self.vel_msg)

        #rotate an extra few degrees
        self.rotate(5)

        self.vel_msg.linear.z = 0
        self.velocity_publisher.publish(self.vel_msg)
        # self.rotate(90, direction=1)
    
    def fix_collision(self):
        self.move_forward(.2, -1)
        self.stop_bot()
        self.rotate(90, -1)
        self.stop_bot()

    

 
if __name__ == "__main__":
    bot = MapBot()

    bot.smart_roam2()