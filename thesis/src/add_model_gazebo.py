#!/usr/bin/env python

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


class GazeboModelCli():

    def __init__(self):
        self.urdfDir = "/root/catkin_ws/src/sim_device/thesis/urdf"
        self.model_relPose_list = [[[]],
                                   [[0, 0, 0, pi / 2, 0, 0]],
                                   [[0.04, 0.01, 0, 0, 0, 0], [-0.04, 0.01, 0, 0, 0, 0], [0.041012, 0.09041, 0.01, 0, 0, 0], [-0.041012, 0.09041, 0.01, 0, 0, 0]],
                                   [[0, 0, 0.018, 0, 0, 0]],
                                   [[0, 0.06, 0, 0, 0, 0]],
                                   [[0, 0.06, 0.018, 0, 0, 0]]]
        self.startXYZ = [0.05, 0.08, 1.015, 0, 0, 0]
        self.randlist = [r for r in random.sample(range(0, 51), 20)]
        self.modelsInBin = [1, 2, 2, 2, 2, 3, 4, 5]
        self.modelXmlList = []
        self.added_models = []
        for i in xrange(1, 6):
            modelName = "lf0640" + str(i)
            with open(os.path.join(self.urdfDir, modelName + ".urdf"), "r") as f:
                model_xml = f.read()
            self.modelXmlList.append(model_xml)

        rospy.loginfo("Waiting for gazebo delete_model services...")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.loginfo("Waiting for gazebo spawn_urdf_model services...")
        rospy.wait_for_service("/gazebo/spawn_urdf_model")

        # instances regards to server client
        self.srvProxy = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        self.req = SpawnModelRequest()
        self.req.reference_frame = "world"
        self.req.robot_namespace = ""

    def add_one_model(self):

        self.resp = self.srvProxy(self.__getReq())
        rospy.loginfo("add model to gazebo success: " + str(self.resp.success))
        if self.resp.success:
            ipdb.set_trace()
            info = "Add model: '" + str(self.req.model_name), "' at : " + str(self.req.initial_pose) + "(xyz,quaternion)"
            rospy.loginfo(info)
        self.added_models.append(self.req.model_name)

    def __getReq(self):
        modelNumber = self.modelsInBin.pop(0)
        modelName = "lf0640" + str(modelNumber)
        self.req.model_name = modelName + "_" + str(self.randlist.pop())
        ipdb.set_trace()

        if (modelNumber > 5 or modelNumber < 0 or type(modelNumber) != int):
            rospy.logfatal("Model number must be integer and between [1, 5], check GazeboModelCli.modelsInBin")
            raise TypeError

        # assign urdf file
        self.req.model_xml = self.modelXmlList[modelNumber - 1]

        # when there are to many possible pose for a model, randomly choose a pose
        model_relPose = self.model_relPose_list[modelNumber].pop(random.sample(range(0, len(self.model_relPose_list[modelNumber])), 1)[0])
        worldPose = [self.startXYZ[i] + model_relPose[i] for i in range(0, 6)]

        # assign pose
        self.req.initial_pose.position.x = worldPose[0]
        self.req.initial_pose.position.y = worldPose[1]
        self.req.initial_pose.position.z = worldPose[2]
        tf_qFromEuler = tf.transformations.quaternion_from_euler(worldPose[3], worldPose[4], worldPose[5])
        orient = Quaternion(tf_qFromEuler[0], tf_qFromEuler[1], tf_qFromEuler[2], tf_qFromEuler[3])
        self.req.initial_pose.orientation = orient
        return self.req

    def srvIsDone(self):
        return True if len(self.modelsInBin) else False


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


if __name__ == '__main__':
    # wait for all servers
    rospy.init_node("spawn_products_in_bins")

    gazeboModelCli = GazeboModelCli()
    gotoPoseCli = GotoPoseCli()
    ipdb.set_trace()

    rospy.loginfo("All Service Server Connected")

    # initialize all clients
    # delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    gazeboModelCli.add_one_model()
    gotoPoseCli.goToFirstPose()


    raw_input("Press Enter to add next model\n")


    ipdb.set_trace()

    # raw_input("Press Enter to delete\n")
    # # delete_model(req.model_name)

    # for i in added_models:
    #     delete_model(i)
    #     print("Deleted model: ", i)

