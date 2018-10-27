#!/usr/bin/env python

import roslib
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from thesis_visualization_msgs.srv import srv_visualModel, srv_visualModelRequest, srv_visualModelResponse
import rospy
import math
import ipdb
import sys
import random
from std_msgs.msg import ColorRGBA
from thesis_visualization.msg import objectLocalization

# this node subscribe recognized models, and publish marker arrays
# takes 3 args
# subscribe_topic, publich_topic, ns



class MarkersManipulation:
    def __init__(self):
        rospy.init_node('addPremitiveObj',anonymous=True)
        
        self.subTopic = sys.argv[1]
        self.pubTopic = sys.argv[2]
        self.ns = sys.argv[3]
        self.scale = 0.001
        self.color = ColorRGBA()
        self.color.r = round(random.uniform(0,1),1)
        self.color.g = round(random.uniform(0,1),1)
        self.color.b = round(random.uniform(0,1),1)
        self.color.a = round(random.uniform(0.4,0.8),1)

        self.markerArray = MarkerArray()
        self.objList = objectLocalization()
        self.sub = rospy.Subscriber(self.subTopic, objectLocalization, self.__subCb, queue_size=1)
        self.pub = rospy.Publisher(self.pubTopic, MarkerArray,queue_size=1)
        self.rate = rospy.Rate(1)
        

    def __subCb(self, msg):
        self.objList = msg

    def run(self):
        while (not rospy.is_shutdown()):
            self.gen_markerArray()
            self.pub.publish(self.markerArray)
            # print "Number of models in the marker Array: %d" %len(self.markerArray.markers)
            self.markerArray = MarkerArray()
            self.rate.sleep()

    def gen_markerArray(self):
        for i in range(0, len(self.objList.modelList)):
            self.__add_model(self.objList.headers[i].frame_id, self.objList.modelList[i], self.objList.pose[i])

    def __add_model(self, frame_id, modelname, pose):
        marker = Marker()
        marker.id = len(self.markerArray.markers)
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.ns
        marker.type = marker.MESH_RESOURCE
        filePath = "package://thesis/meshes/" + modelname + ".stl"
        # print filePath
        marker.mesh_resource = filePath
        marker.action = marker.ADD
        marker.scale.x = self.scale
        marker.scale.y = self.scale
        marker.scale.z = self.scale
        marker.color = self.color
        marker.pose = pose
        self.markerArray.markers.append(marker)
        # print "added model: " + modelname

    

if __name__ == "__main__":
    marker_mani = MarkersManipulation()
    marker_mani.run()
