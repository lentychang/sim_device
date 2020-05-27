#!/usr/bin/python3

import rospy
from tf import TransformListener, Transformer
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix,translation_matrix
import tf
import tf2_ros
import tf2_py
from tf2_ros import BufferInterface
import sys
import ipdb
import PyKDL
from tf2_kdl import transform_to_kdl
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PointStamped
from tf2_py import ConnectivityException


tfTopic = "/tf"
depthCameraTopic = sys.argv[1]
worldFrame =  sys.argv[2]


def translation2list(translation):
    return [translation.x, translation.y, translation.z]

def qaternion2list(qaternion):
    return [qaternion.x, qaternion.y, qaternion.z, qaternion.w]


class ModelTransform:

    def __init__(self, *args):
        rospy.init_node('tf2_listener')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(3.0)
        self.depthCameraTopic = depthCameraTopic #sys.argv[1]
        self.worldFrame = worldFrame #sys.argv[2]

    def printTransform(self):
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(self.depthCameraTopic, worldFrame, rospy.Time())
            except (tf2_ros.LookupException , tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            
            print trans.transform.translation, trans.transform.rotation

            translation_matrix(translation2list(trans.transform.translation))
            quaternion_matrix(qaternion2list(trans.transform.rotation))

            pt = PointStamped()
            pt.header.stamp = trans.header.stamp
            print pt.header.stamp
            pt.header.frame_id = self.depthCameraTopic
            pt.point.x =0.0
            pt.point.y = 1.0
            pt.point.z = 0.0
            self.targetPnt = self.tfBuffer.transform(pt,"world")
            print self.targetPnt

            self.rate.sleep()
        # transformation computation
        # https://answers.ros.org/question/133331/multiply-two-tf-transforms-converted-to-4x4-matrices-in-python/


if __name__ == "__main__":
    modelTf = ModelTransform()
    modelTf.printTransform()
