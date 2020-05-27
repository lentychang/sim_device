#!/usr/bin/python3

import sys, os
import rospy
from tf import TransformListener
import tf
import geometry_msgs
import ipdb
from thesis_visualization_msgs.msg import objectLocalization
import std_msgs


class FakeNode:
    def __init__(self, *args):
        rospy.init_node("fake_recognition_publisher")

        self.cameraFrameName = sys.argv[1]
        self.pubTopic = sys.argv[2]
       # print self.cameraFrameName
        self.__stopPub = False
        self.pub = rospy.Publisher(self.pubTopic, objectLocalization,queue_size=1)
        self.rate = rospy.Rate(1)
        t_stamp = rospy.Time.now()

        self.fakeHeaders = [std_msgs.msg.Header()]*5
        for i in range(0,len(self.fakeHeaders)):
            self.fakeHeaders[i].stamp=t_stamp
            self.fakeHeaders[i].frame_id = self.cameraFrameName
        self.fakeModels = ["lf064-04", "lf064-05", "lf064-04", "lf064-05", "lf064-02"]
        
        self.fakePoses = [geometry_msgs.msg.Pose()]*5        
        poseList = [ [[1,1,1],[0,0,0,1]],
                     [[0.5,0.5,0.5],[0,0,1,0]],
                     [[0.5,0.2,0.5],[1,0,0,0]],
                     [[0.5,0.5,0.3],[0,0,1,0]],
                     [[0.5,0.8,0.7],[0,1,1,0]]]
        for i in range(0, len(poseList)):
            self.fakePoses[i].position.x= poseList[0][0][0]
            self.fakePoses[i].position.y= poseList[0][0][1]
            self.fakePoses[i].position.z= poseList[0][0][2]
            self.fakePoses[i].orientation.x= poseList[0][1][0]
            self.fakePoses[i].orientation.y= poseList[0][1][1]
            self.fakePoses[i].orientation.z= poseList[0][1][2]
            self.fakePoses[i].orientation.w= poseList[0][1][3]
    def pubish(self):
        msg = objectLocalization()
        msg.headers = self.fakeHeaders
        msg.modelList = self.fakeModels
        msg.pose = self.fakePoses
        
        while (not rospy.is_shutdown()):
            t_stamp = rospy.Time.now()
            for i in range(0,len(self.fakeHeaders)):
                self.fakeHeaders[i].stamp=t_stamp
                self.fakeHeaders[i].frame_id= self.cameraFrameName
            self.pub.publish(msg)
            self.rate.sleep()
            

if __name__ == "__main__":
    fake_node = FakeNode()
    fake_node.pubish()
