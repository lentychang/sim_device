#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Quaternion, Pose, TransformStamped
import os
import ipdb
from numpy import ndarray
import sys
from pathlib import Path
catkin_parent = str(Path.home())+ "/"

urdfDir = catkin_parent + "catkin_ws/src/sim_device/thesis/urdf"


if __name__ == '__main__':
    rospy.init_node("deleteModel")
    print("Waiting for gazebo delete_model services...")
    rospy.wait_for_service("/gazebo/delete_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    while True:
        name_model = raw_input("enter the name of the model you want to delete\n")
        delete_model(name_model)
