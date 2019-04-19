#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from socket import *

class pose2d:
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.yaw = 0.

    @staticmethod
    def set_pose(data,self):
        self.x, self.y, self.yaw = data.x, data.y, data.theta

    @staticmethod
    def get_pose(self):
        return (self.x, self.y, self.yaw)

def pose_subscriber():
    global pose
    pose = pose2d()
    rospy.init_node('pose_data_node', anonymous=True)
    rospy.Subscriber('/pose2D', Pose2D, pose.set_pose, (pose))
    rospy.spin()

def get_pose_object():
    return pose

if __name__ == '__main__':
    pose_subscriber()
