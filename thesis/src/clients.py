import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, SpawnModelRequest
from thesis_msgs.srv import MvGrpSrv, MvGrpSrvRequest, MvGrpSrvResponse
from geometry_msgs.msg import Quaternion, Pose, TransformStamped
from gazebo_msgs.msg import ModelStates
import os
import ipdb
from numpy import ndarray
import sys
import random
from math import pi


class GotoPoseCli():

    def __init__(self):
        rospy.wait_for_service("mvGrpSrv")
        self.srvProxy = rospy.ServiceProxy("mvGrpSrv", MvGrpSrv)
        self.req = MvGrpSrvRequest()
        self.req.poseNumber = 0

    def goToNthPose(self, goToNext=True):
        if goToNext and self.req.poseNumber <= 1:
            self.req.poseNumber += 1
        else:
            rospy.logwarn("Has tried all defined positions, if no object recognized, please define more position for the robot")
        self.resp = self.srvProxy(self.req)
        rospy.loginfo("Go n-th pose success: " + str(self.resp.success))

    def goToFirstPose(self):
        self.req.poseNumber = 1
        self.resp = self.srvProxy(self.req)
        rospy.loginfo("Go 1st pose success: " + str(self.resp.success))