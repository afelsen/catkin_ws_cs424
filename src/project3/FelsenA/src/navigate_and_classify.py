#!/usr/bin/env python
import rospy
from actionlib_msgs.msg import GoalStatusArray

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

import time
import torch
import rospkg
from cv_bridge import CvBridge
import cv2
import math
import numpy as np

from NN import CNN


class NavBot:
    def __init__(self):
        rospy.init_node('nav_bot_node')
        self.target_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        self.string_publisher = rospy.Publisher("classification", String)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()


        self.navigation_status_subsubscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, self.update_status)
        self.status = None

        self.image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.update_image)
        self.image = np.zeros((64,64,3))

        rospack = rospkg.RosPack()
        path = rospack.get_path('FelsenA')
        

        model_path = path + "/models/model-1.py"
        self.net = CNN(2)

        # device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        # print("Device: ", device)
        # self.net.to(device)

        checkpoint = torch.load(model_path, map_location=torch.device('cpu'))
        self.net.load_state_dict(checkpoint['model_state_dict'])
        
    
    def update_status(self, status):
        self.status = status

    def update_image(self, image):
        bridge = CvBridge()
        cv_image_full = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        cv_image_cropped = cv_image_full[:540, 350:1300]
        cv_image = cv2.resize(cv_image_cropped, (64,64))
             
        self.image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # rospack = rospkg.RosPack()
        # path = rospack.get_path('project3')
        # cv2.imwrite(path + "/test.png", cv_image)

    def classify(self, image):
        image = image[..., np.newaxis]
        image = image.astype(np.float64)
        image = torch.from_numpy(image)
        image = image.permute((3, 2, 0, 1))

        output = self.net(image.float())
        prediction = torch.argmax(output)

        # self.string_publisher.publish(str(output))

        if output[0][0] - output[0][1] > 15:
            return True
        else:
            return False

    def continuously_classify(self):
        while True:
            self.classify(self.image)

    def rotate_and_classify(self, location):
        # rate = rospy.Rate(10)
        # angles = [(0.8660254, 0.5), (0.8660254, -0.5), (0, 1)]

        # for angle in angles:
        #     location.pose.orientation.z = angle[0]
        #     location.pose.orientation.w = angle[1]

        #     self.target_publisher.publish(location)

        #     self.string_publisher.publish(str(self.status.status_list[0].status))
        #     # wait for the robot to start moving toward the destination
        #     while not self.status or len(self.status.status_list) == 0 or self.status.status_list[0].status != 1:
        #         # rate.sleep()
        #         self.string_publisher.publish(str(self.status.status_list[0].status))
            
        #     self.string_publisher.publish(str("HERE1"))

        #     # wait for the robot to reach the destination
        #     while self.status.status_list[0].status == 1:
        #         rate.sleep()

        #         self.string_publisher.publish(str("HERE2"))


        #         if self.classify(self.image):
        #             print (True)
        #             return

        #     self.string_publisher.publish(str("HERE3"))


        angle = 2*math.pi
        speed = .3

        self.vel_msg.angular.z = speed

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle < angle:
            self.velocity_publisher.publish(self.vel_msg)
            t1=rospy.Time.now().to_sec()

            current_angle = speed*(t1-t0)
            if self.classify(self.image):
                self.string_publisher.publish(str("True"))

                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                return
            else:
                pass

        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

        self.string_publisher.publish(str("False"))


            
    def navigation(self):
        target1 = PoseStamped()
        target1.header.frame_id = 'map'
        target1.pose.position.x = 5.42
        target1.pose.position.y = 2.722
        target1.pose.orientation.w = 1

        target2 = PoseStamped()
        target2.header.frame_id = 'map'
        target2.pose.position.x = 4.05
        target2.pose.position.y = -4.433
        target2.pose.orientation.w = 1

        target3 = PoseStamped()
        target3.header.frame_id = 'map'
        target3.pose.position.x = -7.03973436356
        target3.pose.position.y = -1.5
        target3.pose.orientation.w = 1

        rate = rospy.Rate(10)  # 10hz

        while self.target_publisher.get_num_connections() < 1:
            rate.sleep()
            print("sleeping")
        print("awake!")
        time.sleep(2)

        all_targets = [target1, target2, target3]
        for target in all_targets:
            self.target_publisher.publish(target)

            # wait for the robot to start moving toward the destination
            while not self.status or len(self.status.status_list) == 0 or self.status.status_list[0].status != 1:
                rate.sleep()

            # wait for the robot to reach the destination
            while self.status.status_list[0].status == 1:
                rate.sleep()

            #The robot is at its destination
            self.rotate_and_classify(target)
            
        rospy.spin()

if __name__ == "__main__":
    bot = NavBot()
    # bot.continuously_classify()
    bot.navigation()

    