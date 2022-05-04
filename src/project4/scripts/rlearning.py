#!/usr/bin/env python
import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import String
import time
from std_msgs.msg import Bool
from std_msgs.msg import String, Float32, Empty
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped, Quaternion, PoseStamped
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry
import random

import numpy as np
import json

global current_state

new_state_received = Bool()

rooms = ["r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"]
actions = ["N", "E", "S", "W"]

coordinates = { "r1": [-10.2, 8.63],
                "r2": [-1.49, 8.56],
                "r3": [8.97, 9.13],
                "r4": [-9.22, 1.69],
                "r5": [-0.629, 1.94],
                "r6": [8.97, 1.83],
                "r7": [-9.97, -6.94],
                "r8": [-0.162, -7.58],
                "r9": [9.15, -7.33]
                }

accessible_rooms = {
                "r1": {"N": "r2", "S": "None", "E": "r4", "W": "None"},
                "r2": {"N": "r3", "S": "r1", "E": "r5", "W": "None"},
                "r3": {"N": "None", "S": "r2", "E": "r6", "W": "None"},
                "r4": {"N": "r5", "S": "None", "E": "r7", "W": "r1"},
                "r5": {"N": "r6", "S": "r4", "E": "r8", "W": "r2"},
                "r6": {"N": "None", "S": "r5", "E": "r9", "W": "r3"},
                "r7": {"N": "r8", "S": "None", "E": "None", "W": "r4"},
                "r8": {"N": "r9", "S": "r7", "E": "None", "W": "r5"},
                "r9": {"N": "None", "S": "r8", "E": "None", "W": "r6"}
    
}

current_state = "r1"
next_state = "None"

stateOut = "r5"
statePrimeOut = ""
actionOut = ""
rewardOut = ""
actionStartTime = 0

endEpisode = False


def goto_location(location):
    goalReached = False

    while(not goalReached):

        goalReached = moveToGoal(location)
        # if (goalReached):
        #     rospy.loginfo("Reached destination!")

def moveToGoal(location):

    move = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    while(not move.wait_for_server(rospy.Duration.from_sec(5.0))):
            alpha = 1
            # rospy.loginfo("Waiting for the move_base action server to come up")

    simplegoal = MoveBaseGoal()
    simplegoal.target_pose.header.frame_id = "map"
    simplegoal.target_pose.header.stamp = rospy.Time.now()
    # goal_location_coordinates = location.split(",")
    simplegoal.target_pose.pose.position =  Point(float(location[0]),float(location[1]),0)
    # simplegoal.target_pose.pose.orientation.x = 0.0
    # simplegoal.target_pose.pose.orientation.y = 0.0
    simplegoal.target_pose.pose.orientation.z = float(-0.384135527805)
    simplegoal.target_pose.pose.orientation.w = float(0.923276717067)
    # rospy.loginfo("Sending Next goal location ")
    move.send_goal(simplegoal)
    # pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    # msg = Twist()
    # speed = 2
    # msg.linear.x = speed
    # pub.publish(msg)

    

    move.wait_for_result(rospy.Duration(60))
    if(move.get_state() ==  GoalStatus.SUCCEEDED):
        # rospy.loginfo("This is the end of navigation")
        return True

    # else:
    #     rospy.loginfo("The robot failed to reach the destination")


def updateRobotState(nextStateIn):
    global current_state
    current_state = nextStateIn
    # state_pub.publish(nextStateIn)

def updateRobotNextState():
    next_state_pub.publish(next_state)




def takeNewAction(Q_table, is_random):
    global next_state, current_state, actionOut, rewardOut, stateOut, statePrimeOut
    global endEpisode, actionStartTime
    actionSpace = []

    if not is_random:
        epsilon = 0
        if random.random() < epsilon: #random variation
            actionOut = random.choice(actions)
        else:
            state_index = rooms.index(current_state)
            if current_state == "r9":
                actionOut = "report"
            else:
                actionOut = actions[np.argmax(Q_table[state_index,:])]
    else:
        if current_state != "r9":
            #actionSpace = ["N", "S", "E", "W"]
            currentRandomFloat = random.uniform(0, 1)
            if currentRandomFloat <= 0.4:
                actionOut = "N"
            elif (currentRandomFloat > 0.4) and (currentRandomFloat <= 0.8):
                actionOut = "E"
            else:
                extraRandomActions = random.uniform(0, 1)
                if extraRandomActions <=0.5:
                    actionOut = "S"
                else:
                    actionOut = "W"   
        else:
            actionOut = "report"    

    actionStartTime = time.time()

    # if current_state == "term":
    current_accessible_rooms = accessible_rooms[current_state]
    if actionOut == "report":
        if current_state == "r9":
            stateOut = current_state
            rewardOut = 1000
            statePrimeOut = "term"
            endEpisode = True
        else:
            stateOut = current_state
            rewardOut = -10
            statePrimeOut = current_state
        # state_pub.publish(current_state)
    else:
        if (current_accessible_rooms[actionOut] == "None"):
            # state_pub.publish(current_state)
            stateOut = current_state
            statePrimeOut = current_state
            rewardOut = -1

            
        else:
            next_state = current_accessible_rooms[actionOut]
        
            goto_location(coordinates[next_state])
            # actionOut = randAction
            stateOut = current_state
            statePrimeOut = next_state
            rewardOut = calculateReward()
            updateRobotState(next_state)
    print (stateOut, actionOut, statePrimeOut, rewardOut)
    return stateOut, actionOut, statePrimeOut, rewardOut


def calculateReward():
    # now = time.time()
    currentTime = time.time()
    reward = int(currentTime - actionStartTime) + 60
    return -(reward) 




def episode(Q_table, is_random, id):
    global endEpisode, current_state, next_state, actionStartTime

    current_state = "r1"
    next_state = "None"
    endEpisode = False
    actionStartTime = 0
    
    alpha = .1
    gamma = .9

    calculations = ""

    try:

        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('rlearning')

        counter = 0
        goto_location(coordinates["r1"])
        while not endEpisode:

            stateOut, actionOut, statePrimeOut, rewardOut = takeNewAction(Q_table, is_random)
            if actionOut == "report":
                continue

            state = rooms.index(stateOut)
            action = actions.index(actionOut)

            statePrime = rooms.index(statePrimeOut)
            maxQPrime = np.max(Q_table[statePrime])

            update = Q_table[state, action] + alpha * (rewardOut + gamma * maxQPrime - Q_table[state, action])

            calculations += "Q(" + stateOut + ", " + actionOut + ") = " + str(Q_table[state, action]) + " + " + str(alpha) + " * (" + str(rewardOut) + " + " + str(gamma) + " * " + str(maxQPrime) + " - " + str(Q_table[state, action]) + ") = " + str(update) + "\n" 

            Q_table[state, action] = update

            counter += 1

        with open("Q_calculations_" + id + ".txt", 'w') as f:
            f.write(calculations)

        with open("Q_table_" + id + ".npy", 'wb') as f:
            np.save(f, Q_table)


    except rospy.ROSInterruptException:
        print "finished!"


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    Q_table = np.zeros((9,4))
    Q_table[8, 0] = 1000
    Q_table[8, 1] = 1000
    Q_table[8, 2] = 1000
    Q_table[8, 3] = 1000

    print "Starting Random Episode:"
    episode(Q_table, True, "1")

    
    print "Starting Episode with Q_Values:"
    with open("Q_table_1.npy", 'rb') as f:
        Q_table = np.load(f)
    episode(Q_table, False, "2")


    with open("Q_table_2.npy", 'rb') as f:
        Q_table = np.load(f)
    policy = ""
    for i in range(9):
        actionOut = actions[np.argmax(Q_table[i,:])]
        policy += "pi[" + rooms[i] + "] = " + str(actionOut) + "\n"
    
    with open("policy.txt", 'w') as f:
        f.write(policy)


