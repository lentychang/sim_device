#!/usr/bin/env python

import sys
import rospy
from tf import TransformListener
import tf
import geometry_msgs
import ipdb
import thesis_visualization.msg
import std_msgs
from threading import Thread

class TfNode:
    def __init__(self, *args):
        self.tf = TransformListener()
        self.sub = rospy.Subscriber("/detectedObjs_primitive",thesis_visualization.msg.objectLocalization, self.__subCb, queue_size=1)
        self.msg = thesis_visualization.msg.objectLocalization()
        self.thread_sub = Thread(target=self.start_sub)

    def __subCb(self,msg):
        self.msg = msg
        print self.msg.modelList
    def start_sub(self):        
        rospy.spin()
    def subscribe(self):
        self.thread_sub.start()
    def unregister_sub(self):
        self.sub.unregister()
        rospy.signal_shutdown("close subcriber")


    def example_function(self):
        if self.tf.frameExists("/world") and self.tf.frameExists("/kinect2_depth"):
            t = self.tf.getLatestCommonTime("/world", "/kinect2_depth")
            p1 = geometry_msgs.msg.PoseStamped()
            p1.header.frame_id = "kinect2_depth"
            p1.pose.orientation.w = 1.0    # Neutral orientation
            p_in_base = self.tf.transformPose("/world", p1_)
            print "Position of the fingertip in the robot base:"
            print p_in_base

    def pcd2worldFrame(self, pcd):
        self.tf.transformPointCloud("world",pcd)


class FakeNode:
    def __init__(self, *args):
        rospy.init_node("fake_recognition_publisher")
        self.__stopPub = False
        self.pub = rospy.Publisher("/detectedObjs_primitive",thesis_visualization.msg.objectLocalization,queue_size=1)
        self.rate = rospy.Rate(1)
        self.thread_pub = Thread(target = self.pubisher)
        t_stamp = rospy.Time.now()

        self.fakeHeaders = [std_msgs.msg.Header()]*5
        for i in range(0,len(self.fakeHeaders)):
            self.fakeHeaders[i].stamp=t_stamp
            self.fakeHeaders[i].frame_id="camera"
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
    def pubisher(self):
        msg = thesis_visualization.msg.objectLocalization()
        msg.headers = self.fakeHeaders
        msg.modelList = self.fakeModels
        msg.pose = self.fakePoses
        while (not self.__stopPub):
            self.pub.publish(msg)
            self.rate.sleep()
            print "published!!"
    def publish(self):
        self.thread_pub.start()



    def stopPublish(self):
        self.__stopPub = True

        


if __name__ == "__main__":
    fakenode = FakeNode()
    fakenode.publish()
    tNode = TfNode()
    tNode.subscribe()

    flag = True
    while flag:
        key = raw_input("sp:stop publisher, ss: stop subsriber, e: close program")
        if key == "sp":
            fakenode.stopPublish()
            print "wait for join"
            fakenode.thread_pub.join()
            print "joined"
        elif key == "ss":
            tNode.unregister_sub()
            print tNode.thread_sub.is_alive()
        if key == "e":
            if fakenode.thread_pub.is_alive():
                fakenode.stopPublish()
                fakenode.thread_pub.join()
            if tNode.thread_sub.is_alive():
                tNode.unregister_sub()
            print tNode.thread_sub.is_alive()
            flag = False
    print "stopped!!"


        





    # msg = thesis_visualization.msg.objectLocalization()
    # msg.modelList = ["aaa","nnn","ccc"]
    # node = TfNode()