#!/usr/bin/env python
import sys
import roslib
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import numpy as np
import math
import tf
import tf2_ros
from tf.transformations import quaternion_matrix, euler_from_quaternion
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.patches import Ellipse

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
        self.maximumValue = 0.02

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

def getCurrentPos(l):
    """
    Given the tf listener, we consider the camera's z-axis is the header of the car
    """
    br = tf.TransformBroadcaster()
    result = None
    foundSolution = False

    for i in range(0, 9):
        camera_name = "camera_" + str(i)
        if l.frameExists(camera_name):
            try:
                now = rospy.Time()
                # wait for the transform ready from the map to the camera for 1 second.
                l.waitForTransform("map", camera_name, now, rospy.Duration(1.0))
                # extract the transform camera pose in the map coordinate.
                (trans, rot) = l.lookupTransform("map", camera_name, now)
                # convert the rotate matrix to theta angle in 2d
                matrix = quaternion_matrix(rot)
                angle = math.atan2(matrix[1][2], matrix[0][2])
                # this is not required, I just used this for debug in RVIZ
                br.sendTransform((trans[0], trans[1], 0), tf.transformations.quaternion_from_euler(0,0,angle), rospy.Time.now(), "base_link", "map")
                result = np.array([trans[0], trans[1], angle])
                foundSolution = True
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                print("meet error")
    listener.clear()
    return foundSolution, result

def kPredict(l, state, sigma, twist, fDict, dets, timeDict):
    br = tf.TransformBroadcaster()
    br.sendTransform((state[0,0], state[1,0], 0), tf.transformations.quaternion_from_euler(0,0,state[2,0]), rospy.Time.now(), "robot", "world")
    tags_det = l.getFrameStrings()
    tag_ids = [t[-1] for t in tags_det if "camera_" in t]
    new_tags = set(tag_ids)-set(fDict)
    n = len(new_tags)
    theta = state[2][0]
    T_mat = np.array([[np.cos(theta), np.sin(theta), state[0]],
                  [-np.sin(theta), np.cos(theta), state[1]],
                  [0.0, 0.0, 1.0]])
    for t in tag_ids:
        try:
            now = rospy.Time()
            latest_time = l.getLatestCommonTime("robot", "camera_"+t)
            if t in timeDict :
                if timeDict[t] == latest_time:
                    continue
            timeDict[t] = latest_time
            (trans, rot) = l.lookupTransform("robot", "camera_"+t, now)
            matrix = quaternion_matrix(rot)
            angle = math.atan2(matrix[1][2], matrix[0][2])
            det = np.array([trans[0],trans[1],angle])
            dets[t] = det
            if t in new_tags:
                z_trans_new = np.dot(T_mat, np.array([[trans[0]], [trans[1]], [1.0]]))
                z_new = np.array([z_trans_new[0][0],z_trans_new[1][0],angle+state[2]]).reshape(-1,1)
                state = np.vstack((state,z_new))
                fDict[t] = len(fDict.keys())
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
            print("error in try section", e)
    sigma_new = np.hstack([
                        np.vstack([sigma,
                                   np.zeros((3*n, len(sigma)))]),
                        np.zeros((len(sigma) + 3*n, 3*n))
                    ])
    state_pred = state + np.vstack((twist.reshape(-1,1),np.zeros((3*len(fDict.keys()),1))))
    state_pred[2][0] = (state_pred[2][0] + np.pi) % (2 * np.pi) - np.pi
    for i in range(2, state_pred.shape[0], 3):
        state_pred[i] = (state_pred[i] + np.pi) % (2 * np.pi) - np.pi
    q = 1e-3
    sigma_pred = sigma_new + q*np.identity(sigma_new.shape[0])
    return state_pred, sigma_pred, fDict, dets, timeDict

def kUpdate(state, sigma, fDict, dets):
    d = len(dets)
    if d == 0:
        return state, sigma
    theta = state[2][0]
    h = np.array([[-np.cos(theta), -np.sin(theta), 0.0],
                  [np.sin(theta), -np.cos(theta), 0.0],
                  [0.0, 0.0, -1.0]])
    H = np.array([])
    tot = len(fDict)
    for i in dets:
        pos = fDict[i]
        row = np.hstack((h, np.zeros((3,pos*3)), -h, np.zeros((3,3*(tot-pos-1)))))
        if len(H) == 0:
            H = row
        else:
            H = np.vstack((H, row))
    r = 1e-5
    s = np.dot(H,np.dot(sigma,H.T)) + r*np.identity(3*d)
    K = np.dot(sigma,np.dot(H.T,np.linalg.inv(s)))
    z = np.hstack(dets.values()).reshape(-1,1)
    new_state = state + np.dot(K,z-np.dot(H,state))
    for i in range(2, new_state.shape[0], 3):
            new_state[i] = (new_state[i] + np.pi) % (2 * np.pi) - np.pi
    new_sigma = np.dot(np.identity(sigma.shape[0])-np.dot(K,H),sigma)
    return new_state, new_sigma

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
    """
    Convert the twist into the car coordinate
    """
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)

def plot_states(states, mapping):
    keys_list = mapping.keys()
    values_list = mapping.values()
    s = []
    for s in states:
        plt.plot(s[0,0],s[1,0],marker='o',color = 'skyblue')
        colors = cm.rainbow(np.linspace(0, 1, len(s)//3))
    for i in range(3, len(s), 3):
        plt.plot(s[i,0], s[i+1,0], marker='x', color = colors[i//3])
        plt.text(s[i,0], s[i+1,0], keys_list[values_list.index((i//3)-1)])
    plt.savefig('/root/rosws/src/rb5_ros/rb5_control/src/graph.png', dpi=120)

def plot_final_states(state, cov, fDict):
    fig = plt.figure(dpi=120)
    tags = fDict.keys()
    row_indices = fDict.values()
    ax = plt.subplot(111)
    for i in range(1, len(state) // 3):
        cov_mat = cov[3 * i:3 * (i + 1) - 1, 3 * i:3 * (i + 1) - 1]
        x, y = state[3 * i, 0], state[3 * i + 1, 0]
        lambda_, v = np.linalg.eig(cov_mat)
        lambda_ = np.sqrt(lambda_)
        for j in range(1, 2):
            ell = Ellipse(xy=(x, y),
                          width=lambda_[0] * j * 2,
                          height=lambda_[1] * j * 2,
                          edgecolor='black',
                          angle=np.rad2deg((np.arccos(v[0, 0]) + 0j).real))
            ell.set_facecolor('none')
            ax.add_artist(ell)
        plt.plot(x, y, marker='x', color='maroon')
        plt.text(x, y, tags[row_indices.index(3 * i)], color='red')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    fig.savefig('/root/rosws/src/rb5_ros/rb5_control/src/landmarks.png', dpi=120)

if __name__ == "__main__":
    import time
    rospy.init_node("hw3")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    listener = tf.TransformListener()

    waypoint = np.array([[0.0,0.0,0.0], 
                         [1.0,0.0,0.0],
                         [1.0,0.0,np.pi/2],
                         [1.0,1.0,np.pi/2],
                         [1.0,1.0,np.pi], 
                         [0.0,1.0,np.pi],
                         [0.0,1.0,-np.pi/2],
                         [0.0,0.0,-np.pi/2],
                         [0.0,0.0,0.0],
                ])

    # waypoint = np.array([[1/3.0,0,0], 
    #                      [2/3.0,0.0,0],
    #                      [2/3.0,0,np.pi/4],
    #                      [1.0,1/3.0,np.pi/4],
    #                      [1.0,1/3.0,np.pi/2],
    #                      [1.0,2/3.0,np.pi/2],
    #                      [1.0,2/3.0,3*np.pi/4],
    #                      [2/3.0,1.0,3*np.pi/4],
    #                      [2/3.0,1.0,-np.pi],
    #                      [1/3.0,1.0,-np.pi],
    #                      [1/3.0,1.0,-3*np.pi/4],
    #                      [0.0,2/3.0,-3*np.pi/4],
    #                      [0.0,2/3.0,-np.pi/2],
    #                      [0.0,1/3.0,-np.pi/2],
    #                      [0.0,1/3.0,-np.pi/4],
    #                      [1/3.0,0.0,-np.pi/4],
    #                      [1/3.0,0.0,0]])

    # init pid controller
    pid = PIDcontroller(0.1,0.005,0.005)
    # pid = PIDcontroller(0.0175,0.0015,0.11)
    # init current state
    current_state = np.array([0.0,0.0,0.0])
    state = np.array([0.0,0.0,0.0]).reshape(-1,1)
    sigma = np.zeros((3,3))
    fDict = {}
    timeDict = {}
    Ucounter = 1
    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for wp in waypoint:
        states = []
        time.sleep(1)
        print("move to way point", wp)
        print("state = ",state)
        # set wp as the target point
        pid.setTarget(wp)
        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        twist = update_value
        dets = {}
        #print(coord(update_value, current_state))
        time.sleep(0.1)
        pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
        # update the current state
        # current_state += update_value
        state, sigma, fDict, dets, timeDict = kPredict(listener, state, sigma, twist, fDict, dets, timeDict)
        state, sigma = kUpdate(state, sigma, fDict, dets)
        current_state = state.flatten()[:3]
        states.append(state)
        # print(current_state.shape,"cshape",current_state, state.shape)
        count = 1
        # found_state, estimated_state = getCurrentPos(listener)
        # if found_state: # if the tag is detected, we can use it to update current state.
        #     current_state = estimated_state
        
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.15): # check the error between current state and current way point
            dets = {}
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            twist = update_value
            #print(coord(update_value, current_state))
            time.sleep(0.1)
            # pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
            # update the current state
            # current_state += update_value
            state, sigma, fDict, dets, timeDict = kPredict(listener, state, sigma, twist, fDict, dets, timeDict)
            # state, sigma = kUpdate(state, sigma, fDict, dets)
            if count % Ucounter == 0:
                state, sigma = kUpdate(state, sigma, fDict, dets)
                states.append(state)
                dets = {}
                count = 0
            current_state = state.flatten()[:3]
            count += 1
            print("state = ",state)
            # found_state, estimated_state = getCurrentPos(listener)
            # if found_state:
            #     current_state = estimated_state
        pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
        plot_states(states,fDict)
    plot_final_states(states,sigma,fDict)
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

