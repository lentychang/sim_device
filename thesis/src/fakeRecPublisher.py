#!/usr/bin/env python

import sys, os
import rospy
from tf import TransformListener
import tf
from copy import deepcopy
import random
from geometry_msgs.msg import PoseStamped, Transform, Pose
import ipdb
from thesis_visualization_msgs.msg import objectLocalization
import std_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix,translation_matrix
from math import pi, degrees, radians


def rotate_orientation(ori, q):
    eu_ori = list(euler_from_quaternion([ori.x, ori.y, ori.z, ori.w]))
    eu_q = euler_from_quaternion(q)

    eu_ori[0] += eu_q[0]
    eu_ori[1] += eu_q[1]
    eu_ori[2] += eu_q[2]

    
    q_with_noise = quaternion_from_euler(eu_ori[0],eu_ori[1], eu_ori[2])
    # rot_mat = quaternion_matrix(quaternion_from_euler(eu_ori[0],eu_ori[1], eu_ori[2]))
    # pose_rot = rot_mat.dot([ori.x, ori.y, ori.z, ori.w])
    ori.x = q_with_noise[0]
    ori.y = q_with_noise[1]
    ori.z = q_with_noise[2]
    ori.w = q_with_noise[3]
    return ori


def translate_position(pos, t):
    pos.x += t.x
    pos.y += t.y
    pos.z += t.z
    return pos


def list2Transform(xyzrpy):
    transform = Transform()
    # transform.header.frame_id = "iiwa_link_0"

    transform.translation.x = xyzrpy[0]
    transform.translation.y = xyzrpy[1]
    transform.translation.z = xyzrpy[2]
    transform.rotation = quaternion_from_euler(xyzrpy[3], xyzrpy[4], xyzrpy[5])
    return transform

class FakeRecPublisher:
    def __init__(self, *args):
        # rospy.init_node("fake_recognition_publisher2")

        # self.pubTopic = sys.argv[1]
        self.pubTopic = "/detectedObjs_beforeAligned"
        self.pub = rospy.Publisher(self.pubTopic, objectLocalization,queue_size=1)
        self.rate = rospy.Rate(1)
        self.msg = objectLocalization()
        self.fake_msg = objectLocalization()

    def resetPubTopic(self, topicName):
        self.pubTopic = topicName
        self.pub = rospy.Publisher(self.pubTopic, objectLocalization,queue_size=1)

    #[TODO] fix the error keep changing
    def setMsg(self, addedModels, modelsPoses):
        assert len(addedModels) == len(modelsPoses), 'The length of addedModels and modelsPoses must be the same'
        ipdb.set_trace()
        self.msg = objectLocalization()
        self.msg.modelList = addedModels
        self.msg.pose = modelsPoses

    def __noise_trsf(self):
        randomList = []
        for i  in range(0,3):
            randomList.append(random.gauss(mu=0.01, sigma=0.02))
        for i  in range(0,3):
            randomList.append(random.gauss(mu=radians(10)/2.0, sigma=radians(20)/2.0))
        print("randomList gen:{0}".format(randomList))
        trsf = list2Transform(randomList)
        print("trsf gen:{0}".format(trsf))
        return trsf
    def __addNoise(self, add_noise=True):
        # the input msg is already noisy poses, except the last pose haven't been imposed noise
        self.fake_msg = self.msg
        print("before add noise\n{0}".format(self.fake_msg))
        
        # Add noise to last one
        pose = self.fake_msg.pose[-1]
        trsf = self.__noise_trsf()
        pose.position = translate_position(pose.position, trsf.translation)
        pose.orientation = rotate_orientation(pose.orientation, trsf.rotation)

        print("after add noise\n{0}".format(self.fake_msg))


    def __setMsgHeader(self):
        n = len(self.msg.modelList)
        t_stamp = rospy.Time.now()
        self.msg.headers = [std_msgs.msg.Header()] * n
        for i in range(0,len(self.msg.modelList)):
            self.msg.headers[i].stamp=t_stamp
            self.msg.headers[i].frame_id = "world"
    def pubOnce(self, add_noise=True):
        self.__setMsgHeader()
        if add_noise:
            self.__addNoise()
            self.pub.publish(self.fake_msg)
        else:
            self.pub.publish(self.msg)

    def publish(self):
        while (not rospy.is_shutdown()):
            self.__setMsgHeader()
            self.pub.publish(self.msg)
            self.rate.sleep()

def __testPublisher():
    rospy.init_node("testFakePublisher")
    fake_node = FakeRecPublisher()
    pose = Pose()
    pose.orientation.w = 1.0
    poseList = [deepcopy(pose), deepcopy(pose), deepcopy(pose)]
    modelList = ["lf064-01"]*3
    fake_node.setMsg(modelList,poseList)
    fake_node.addNoise()


if __name__ == "__main__":
    __testPublisher()

