#!/usr/bin/env python
import rospy
import math
import time
from mpi_control import MegaPiController
from geometry_msgs.msg import Pose2D

# At speed S = 65 => 1.5 seconds = 3.14 radian rotation
# 1 radian rotaion = 0.5 second sleep => R = 0.5
# 1 second moving straight is 20 cm
# M = 3 means 3 seconds moving straight

R = 0.5
M = 3
S = 65

class HW1Node:
    def __init__(self):
        self.mpi = MegaPiController(port='/dev/ttyUSB0', verbose=True)
        time.sleep(1)
        self.x_curr = 0
        self.y_curr = 0
        self.theta_curr = 0


    def run(self, msg):
        x, y, theta = msg.x, msg.y, msg.theta
        gamma_init = 0
        gamma = 0
        delx = x-self.x_curr
        dely = y-self.y_curr
        if delx!=0:
            gamma_init = math.atan(dely/delx)
        else:
            if y <= self.y_curr:
                gamma_init = -math.pi/2
            else:
                gamma_init = math.pi/2
        if x >= self.x_curr and y >= self.y_curr:
            gamma = gamma_init
        elif x <= self.x_curr and y >= self.y_curr:
            gamma = math.pi - gamma_init
        elif x <= self.x_curr and y <= self.y_curr:
            gamma = gamma_init - math.pi
        elif x >= self.x_curr and y <= self.y_curr:
            gamma = gamma_init
        turning_angle = gamma - self.theta_curr
        # print (self.x_curr, self.y_curr, self.theta_curr), "---->", (x,y,theta)
        # print ("turning angle", turning_angle)
        # Rotate here
        if turning_angle < 0:
            self.mpi.carRotate(-S*1.07)
        else:
            self.mpi.carRotate(S*1.07)
        time.sleep(abs(R*turning_angle))
        self.mpi.carStop()
        dist = math.sqrt(math.pow(x-self.x_curr,2)+math.pow(y-self.y_curr,2))
        # print ("move dist", dist)
        # Move here
        self.mpi.carStraight(S)
        time.sleep(M*dist)
        self.mpi.carStop()
        final_turn_angle = theta - gamma
        # print ("final turn angle", final_turn_angle)
        # Rotate here
        if final_turn_angle < 0:
            self.mpi.carRotate(-S*1.105)
        else:
            self.mpi.carRotate(S*1.02)
        time.sleep(abs(R*final_turn_angle))
        self.mpi.carStop()
        self.x_curr = x
        self.y_curr = y
        self.theta_curr = theta
        # return

if __name__ == "__main__":
    hw1_node = HW1Node()
    rospy.init_node("hw1_sub")
    rospy.Subscriber('/hw1', Pose2D, hw1_node.run, queue_size=5) 
    rospy.spin()
