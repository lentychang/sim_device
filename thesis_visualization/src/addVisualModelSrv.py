#!/usr/bin/env python

import roslib
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from thesis_visualization.srv import srv_visualModel, srv_visualModelRequest, srv_visualModelResponse
import rospy
import math
import ipdb

class MarkersManipulation:
    def __init__(self):
        rospy.init_node('premitiveObjectLocalization_Server')
        self.markerArray = MarkerArray()
        self.topic = 'visualization_marker_array'
        self.pub = rospy.Publisher(self.topic, MarkerArray,queue_size=1)
        self.rate = rospy.Rate(1)
        self.visualModelServer()
        self.addedModelList = []

    def visualModelServer(self):
        s = rospy.Service('/addModel2RvizSrv', srv_visualModel, self.visualModelCb)
        print "addModel2Rviz Server started!"
        rospy.spin()

    def visualModelCb(self, req):
        self.add_model(req.modelName, req.pose, req.colorRGBA, req.scale, req.ns)
        self.publish()
        return srv_visualModelResponse(success=True)

    def publish(self):
        self.pub.publish(self.markerArray)
        self.rate.sleep()

    def add_model(self, modelname, pose, color, scale, ns):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.type = marker.MESH_RESOURCE
        filePath = "package://thesis/meshes/" + modelname + ".stl"
        print filePath
        marker.mesh_resource = filePath
        marker.action = marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        marker.pose = pose
        self.markerArray.markers.append(marker)
        print "added model: " + modelname
        self.addedModelList.append(modelname)
    

if __name__ == "__main__":
    marker_mani = MarkersManipulation()