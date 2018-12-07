#!/usr/bin/env python

import sys, os
import rospy
from tf import TransformListener
import tf
import geometry_msgs
import ipdb
from thesis_visualization_msgs.msg import objectLocalization
import std_msgs


class FakeRecPublisher:
    def __init__(self, *args):
        # rospy.init_node("fake_recognition_publisher2")

        # self.pubTopic = sys.argv[1]
        self.pubTopic = "/detectedObjs_beforeAligned"
        self.pub = rospy.Publisher(self.pubTopic, objectLocalization,queue_size=1)
        self.rate = rospy.Rate(1)
        self.msg = objectLocalization()

    def resetPubTopic(self, topicName):
        self.pubTopic = topicName
        self.pub = rospy.Publisher(self.pubTopic, objectLocalization,queue_size=1)

    def setMsg(self, addedModels, modelsPoses):
        assert len(addedModels) == len(modelsPoses), 'The length of addedModels and modelsPoses must be the same'
        self.msg = objectLocalization()
        self.msg.modelList = addedModels
        self.msg.pose = modelsPoses
    def __setMsgHeader(self):
        n = len(self.msg.modelList)
        t_stamp = rospy.Time.now()
        self.msg.headers = [std_msgs.msg.Header()] * n
        for i in range(0,len(self.fakeHeaders)):
            self.msg.headers[i].stamp=t_stamp
            self.msg.headers[i].frame_id = "world"
    def pubOnce(self):
        self.__setMsgHeader()
        self.pub.publish(self.msg)

    def pubish(self):
        while (not rospy.is_shutdown()):
            self.__setMsgHeader()
            self.pub.publish(self.msg)
            self.rate.sleep()

if __name__ == "__main__":
    fake_node = FakeRecPublisher()
    fake_node.pubish()

