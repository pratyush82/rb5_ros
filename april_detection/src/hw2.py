#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist
from april_detection.msg import AprilTagDetectionArray
from tf.transformations import quaternion_matrix, euler_from_matrix
import matplotlib.pyplot as plt
import numpy as np

"""
The class of the pid controller.
"""
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.1

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result 

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)

def april_callback(msg):
    global detArray
    detArray = msg.detections  

def updateCurrentState(current):
    for d in detArray:
        q = [d.pose.orientation.x,d.pose.orientation.y,d.pose.orientation.z,d.pose.orientation.w]
        tagId = d.id
        pos = [d.pose.position.x,d.pose.position.y,d.pose.position.z]
        det = quaternion_matrix(q)
        det[0,3],det[1,3],det[2,3]=pos
        # print(det)
        rot = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        cam = np.vstack([np.hstack([rot,[[0.05],[0.015],[0.145]]]),[0,0,0,1]])
        print(tagId)
        if tagId == 4:
            rot1 = np.array([[-1,0,0],[0,0,-1],[0,-1,0]])
            world2tag = np.vstack([np.hstack([rot1,[[0],[-1],[0]]]),[0,0,0,1]])
        elif tagId == 5:
            rot2 = np.array([[0,0,-1],[1,0,0],[0,-1,0]])
            world2tag = np.vstack([np.hstack([rot2,[[0],[2],[0]]]),[0,0,0,1]])
        elif tagId == 8:
            rot3 = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
            world2tag = np.vstack([np.hstack([rot3,[[2],[0],[0]]]),[0,0,0,1]])
        elif tagId == 3:
            rot4 = np.array([[1,0,0],[0,0,1],[0,-1,0]])
            world2tag = np.vstack([np.hstack([rot4,[[1],[2.5],[0]]]),[0,0,0,1]])
        # print(world2tag)
        final = np.dot(world2tag,np.linalg.inv(np.dot(cam,det)))
        # print(final)
        [x,y,z] = np.dot(final,[[0],[0],[0],[1]]).flatten()[:3]
        theta = euler_from_matrix(final[:3,:3])[-1]
        current = np.array([x,y,theta])
        # print(current)
    return current
        


if __name__ == "__main__":
    import time
    global detArray
    detArray = []
    rospy.init_node("hw2")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)
    sub_aprils = rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, april_callback, queue_size=1)

    plt.figure()
    plt.xlabel("X")
    plt.ylabel("Y")
    color = 'skyblue'
    current_state_array = []
    
    waypoint = np.array([
                        #  [0.0,0.0,0.0], 
                         [1.0,0.0,-0.07],
                         [1.0,2.0,-np.pi+0.07],
                         [0.0,0.0,0.0]]) 

    # init pid controller
    pid = PIDcontroller(0.0175,0.0015,0.11)

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        current_state += update_value
        current_state = updateCurrentState(current_state)
        while(np.linalg.norm(pid.getError(current_state, wp)[:2]) > 0.12): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += update_value
            current_state = updateCurrentState(current_state)
            print(current_state)
            current_state_array.append(current_state[:2])
        
        plt.plot(wp[0], wp[1], 'rx')
        plt.text(wp[0] + 0.05, current_state[1] + 0.05, str(wp[:2]))
        
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
    prev = None
    for i, current_state in enumerate(current_state_array):
        if i % 4 == 0:
            plt.plot(current_state[0],current_state[1],marker='o',color=color)
            if prev is not None:
                plt.plot([current_state[0], prev[0]],[current_state[1], prev[1]],color=color)
            prev = current_state
    plt.savefig('/root/rosws/src/rb5_ros/april_detection/graph_' + color +'.png')

