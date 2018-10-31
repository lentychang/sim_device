#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Quaternion, Pose, TransformStamped
from gazebo_msgs.msg import ModelStates
import os
import ipdb
from numpy import ndarray
import sys
import random
from math import pi

urdfDir = "/root/catkin_ws/src/sim_device/thesis/urdf"

model_relPose_list = [[[]],
                      [[0, 0, 0, pi / 2, 0, 0]],
                      [[0.04, 0.01, 0, 0, 0, 0], [-0.04, 0.01, 0, 0, 0, 0], [0.041012, 0.09041, 0.01, 0, 0, 0], [-0.041012, 0.09041, 0.01, 0, 0, 0]],
                      [[0, 0, 0.018, 0, 0, 0]],
                      [[0, 0.06, 0, 0, 0, 0]],
                      [[0, 0.06, 0.018, 0, 0, 0]]]
startXYZ = [0.05, 0.08, 1.015, 0, 0, 0]
randlist = [r for r in random.sample(range(0, 51), 20)]

modelXmlList = []
for i in xrange(1, 6):
    modelName = "lf0640" + str(i)
    with open(os.path.join(urdfDir, modelName + ".urdf"), "r") as f:
        model_xml = f.read()
    modelXmlList.append(model_xml)


def addModelMsg(add_model, modelNumber):

    modelName = "lf0640" + str(modelNumber)
    add_model.model_name = modelName + "_" + str(randlist.pop())

    if (modelNumber > 5 or modelNumber < 0 or type(modelNumber) != int):
        rospy.logfatal("Model number must be integer and between [1, 5]")
        raise TypeError

    add_model.model_xml = modelXmlList[modelNumber - 1]

    # when there are to many possible pose for a model, randomly choose a pose
    model_relPose = model_relPose_list[modelNumber].pop(random.sample(range(0, len(model_relPose_list[modelNumber])), 1)[0])
    worldPose = [startXYZ[i] + model_relPose[i] for i in range(0, 6)]
    print(worldPose)

    # assign pose
    add_model.initial_pose.position.x = worldPose[0]
    add_model.initial_pose.position.y = worldPose[1]
    add_model.initial_pose.position.z = worldPose[2]
    tf_qFromEuler = tf.transformations.quaternion_from_euler(worldPose[3], worldPose[4], worldPose[5])
    orient = Quaternion(tf_qFromEuler[0], tf_qFromEuler[1], tf_qFromEuler[2], tf_qFromEuler[3])
    add_model.initial_pose.orientation = orient

    return add_model


if __name__ == '__main__':
    rospy.init_node("spawn_products_in_bins")

    rospy.loginfo("Waiting for gazebo delete_model services...")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.loginfo("Waiting for gazebo spawn_urdf_model services...")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")

    rospy.loginfo("Service Server Connected")

    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

    add_model = SpawnModelRequest()
    add_model.reference_frame = "world"
    add_model.robot_namespace = ""
    added_models = []
    # prepare parameters
    for i in [1, 2, 2, 2, 2, 3, 4, 5]:
        raw_input("Press Enter to add next model\n")
        spawn_model(addModelMsg(add_model, i))
        added_models.append(add_model.model_name)

    raw_input("Press Enter to delete\n")
    # delete_model(add_model.model_name)

    for i in added_models:
        delete_model(i)
        print("Deleted model: ", i)

