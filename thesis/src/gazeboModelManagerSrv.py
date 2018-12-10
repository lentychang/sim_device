#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest, SpawnModel, SpawnModelRequest, DeleteModelResponse, SpawnModelResponse
from thesis_msgs.srv import MvGrpSrv, MvGrpSrvRequest, MvGrpSrvResponse
from geometry_msgs.msg import Quaternion, Pose, TransformStamped
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse, Trigger, TriggerResponse
import os
import ipdb
from numpy import ndarray
import sys
import random
from math import pi
from fakeRecPublisher import FakeRecPublisher


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
        self.modelsPoses = []
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
        self.spawn_model_proxy = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        self.delete_model_proxy = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.req = SpawnModelRequest()
        self.req.reference_frame = "world"
        self.req.robot_namespace = ""

    def add_one_model(self):
        req = self.__getReq()
        if req is not None:
            tmpResp = self.spawn_model_proxy(req)
        else:
            tmpResp = SpawnModelResponse()
            tmpResp.success = False
            tmpResp.status_message = "No model in Bin"
        if tmpResp.success:
            rospy.loginfo("add model to gazebo success: " + str(tmpResp.success))
            info = "Add model: '" + str(self.req.model_name), "' at : " + str(self.req.initial_pose) + "(xyz,quaternion) succeed!"
            rospy.loginfo(info)
            self.added_models.append(self.req.model_name)
            self.modelsPoses.append(self.req.initial_pose)
        else:
            status_message = "Add model failed"
            rospy.logwarn(status_message)
            tmpResp.status_message = status_message + ", " + tmpResp.status_message

        resp = TriggerResponse()
        resp.success = tmpResp.success
        resp.message = tmpResp.status_message

        return resp

    def del_one_model(self):
        if len(self.added_models) > 0:
            delReq = DeleteModelRequest()
            delReq.model_name = self.added_models.pop()
            tmp=self.modelsPoses.pop()
            tmpResp = self.delete_model_proxy(delReq)
            rospy.loginfo("delete model to gazebo success: " + str(tmpResp.success))
            if tmpResp.success:
                info = "Deleted model: " + str(delReq.model_name) + " succeed!"
                rospy.loginfo(info)
            else:
                status_message = "Delete failed due to unknown reason"
                rospy.logwarn(status_message)
                tmpResp.status_message = status_message

        else:
            status_message = "No model to be deleted"
            rospy.logwarn(status_message)
            tmpResp = DeleteModelResponse()
            tmpResp.status_message = status_message

        resp = TriggerResponse()
        resp.success = tmpResp.success
        resp.message = tmpResp.status_message

        return resp

    def __getReq(self):
        if len(self.modelsInBin) > 0:
            modelNumber = self.modelsInBin.pop(0)
            modelName = "lf0640" + str(modelNumber)
            self.req.model_name = modelName + "_" + str(self.randlist.pop())

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
        else:
            rospy.logwarn("no model in Bin")
            return None

    def srvIsDone(self):
        return True if len(self.modelsInBin) else False

rospy.init_node("addModelSrv")
gazeboModelCli = GazeboModelCli()
fakeRecPub = FakeRecPublisher()


def addModelSrvCb(req):
    resp = gazeboModelCli.add_one_model()
    fakeRecPub.setMsg(gazeboModelCli.added_models, gazeboModelCli.modelsPoses)
    fakeRecPub.pubOnce()
    return resp


def delModelSrvCb(req):
    return gazeboModelCli.del_one_model()


def delAllModelSrvCb(req):
    for i in range(0, len(gazeboModelCli.added_models)):
        gazeboModelCli.del_one_model()
    resp = TriggerResponse()
    resp.success = True
    msg = "All model deleted!"
    rospy.loginfo(msg)
    resp.message = msg
    return resp


if __name__ == '__main__':
    # wait for all servers
    s = rospy.Service('/addModelSrv', Trigger, addModelSrvCb)
    d = rospy.Service('/delModelSrv', Trigger, delModelSrvCb)
    d = rospy.Service('/delAllModelSrv', Trigger, delAllModelSrvCb)

    rospy.loginfo("Following Server are Connected: /addModelSrv, /delModelSrv")
    rospy.spin()



    # raw_input("Press Enter to delete\n")
    # # delete_model(req.model_name)

    # for i in added_models:
    #     delete_model(i)
    #     print("Deleted model: ", i)

