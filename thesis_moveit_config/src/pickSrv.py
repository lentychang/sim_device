#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import ipdb
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Transform
from math import pi
from std_msgs.msg import String, Float64
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Grasp
from thesis_msgs.srv import GripSrv, GripSrvRequest, GripSrvResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
from wsg_50_common.srv import Move, MoveRequest, MoveResponse
from moveit_msgs.msg import OrientationConstraint, Constraints
from math import pi
from copy import deepcopy
import wsg_gcl_socket


sim_ = False

if sim_:
    gl_pub = rospy.Publisher('/iiwa/wsg_50_gl/command', Float64)
    gr_pub = rospy.Publisher('/iiwa/wsg_50_gr/command', Float64)
else:
    gripper_socker_ = wsg_gcl_socket.Wsg50Gcl_Socket()
    gripper_socker_.connect()
    gripper_socker_.homing()
    


def rotate_orientation(ori, q):
    rot_mat = quaternion_matrix(q)
    pose_rot = rot_mat.dot([ori.x, ori.y, ori.z, ori.w])
    ori.x = pose_rot[0]
    ori.y = pose_rot[1]
    ori.z = pose_rot[2]
    ori.w = pose_rot[3]
    return ori


def translate_position(pos, t):
    pos.x += t.x
    pos.y += t.y
    pos.z += t.z
    return pos


def list2Transform(xyzrpy):
    transform = Transform()
    # transform.header.frame_id = "iiwa_link_0"

    transform.translation.x = xyzrpy[0]
    transform.translation.y = xyzrpy[1]
    transform.translation.z = xyzrpy[2]
    transform.rotation = quaternion_from_euler(xyzrpy[3], xyzrpy[4], xyzrpy[5])
    return transform


def grip(force=None, width=None, speed=None):
    if sim_:
        if width is None:
            width = 0
            open = width / 2.0
        rCommand = open/1000.0
        lCommand = -1.0 * rCommand
        gl_pub.publish(lCommand)
        gr_pub.publish(rCommand)
        rospy.sleep(3.0)
    else:
        gripper_socker_.grip(force=force, width=width, speed=speed)


def release(pullback_dist=None, speed=None):
    if sim_:
        if pullback_dist is None:
            pullback_dist = 55
        rCommand = pullback_dist
        lCommand = -1.0 * rCommand
        gl_pub.publish(lCommand)
        gr_pub.publish(rCommand)
        rospy.sleep(3.0)
    else:
        gripper_socker_.release(pullback_dist=pullback_dist, speed=speed)


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class GripperInterface(object):
    """GripperInterface"""
    def __init__(self):
        super(GripperInterface, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface',
        #                anonymous=True)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Panda
        ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
        ## you should change this value to the name of your robot arm planning group.
        ## This interface can be used to plan and execute motions on the Panda:
        group_name = "iiwa_arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

        ## Getting Basic Information
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.__setupRelDict()
        self.__graspPathOriConstraint = OrientationConstraint()
        self.__graspPathConstraints = Constraints()
        self.__first_grasp = True
        self.__srvReq_is_set = False
        self.__preGraspPose = PoseStamped()
        self.__graspPose = PoseStamped()


        # q = quaternion_from_euler(0,0,0)
        # r,p,y = euler_from_quaternion(q)
    def __setupRelDict(self):
        self.__preLocGraspRel_1 = {'lf064-01': list2Transform([0,0,0.05,0,0,0]), 
                                   'lf064-02': list2Transform([0.1,0,0.05,0,0,pi/2]), 
                                   'lf064-03': list2Transform([0,0,0,0,0,0]), 
                                   'lf064-04': list2Transform([0,0,0,0,0,0]), 
                                   'lf064-05': list2Transform([0,0,0,0,0,0])}

        self.__locGraspRel_1 = {'lf064-01': list2Transform([0,0,0,0,0,0]), 
                                'lf064-02': list2Transform([0,0,0,0,0,0]), 
                                'lf064-03': list2Transform([0,0,0,0,0,0]), 
                                'lf064-04': list2Transform([0,0,0,0,0,0]), 
                                'lf064-05': list2Transform([0,0,0,0,0,0])}
        # Check why the angle should be half??
        self.__preLocGraspRel_2 = {'lf064-01': list2Transform([0.1,0,0.05,0,0,pi/4]), 
                                   'lf064-02': list2Transform([0,0,0,0,0,0]), 
                                   'lf064-03': list2Transform([0,0,0,0,0,0]), 
                                   'lf064-04': list2Transform([0,0,0,0,0,0]), 
                                   'lf064-05': list2Transform([0,0,0,0,0,0])}


        self.__locGraspRel_2 = {'lf064-01': list2Transform([0.1,0,0.0,0,0,pi/4]), 
                                'lf064-02': list2Transform([0,0,0,0,0,0]), 
                                'lf064-03': list2Transform([0,0,0,0,0,0]), 
                                'lf064-04': list2Transform([0,0,0,0,0,0]), 
                                'lf064-05': list2Transform([0,0,0,0,0,0])}

        self.__preGraspRel = {'lf064-01': list2Transform([0,0,0.05,0,0,0]), 
                              'lf064-02': list2Transform([0,0,0,0,0,0]), 
                              'lf064-03': list2Transform([0,0,0,0,0,0]), 
                              'lf064-04': list2Transform([0,0,0,0,0,0]), 
                              'lf064-05': list2Transform([0,0,0,0,0,0])}

        self.__GraspRel = {'lf064-01': list2Transform([0,0,0.05,0,0,0]), 
                           'lf064-02': list2Transform([0,0,0,0,0,0]), 
                           'lf064-03': list2Transform([0,0,0,0,0,0]), 
                           'lf064-04': list2Transform([0,0,0,0,0,0]), 
                           'lf064-05': list2Transform([0,0,0,0,0,0])}

    def setSrvReq(self, req):
        self.__srvReq = req
        self.__srvReq_is_set = True

    def setGripObj(self):
        assert self.__srvReq_is_set, "srvReq is not set"
        try:
            self.__srvReq.modelName in self.__preLocGraspRel_1.keys()
            self.__modelName = self.__srvReq.modelName
        except:
            rospy.logfatal("Service request failed, the RelPose of given modelName is not defined")
        self.__objPose = self.__srvReq.stampedPose
        self.__setPregraspPose()
        self.__setGraspPose()


    def __setPregraspPose(self):
        position = deepcopy(self.__objPose.pose.position)
        orientation = deepcopy(self.__objPose.pose.orientation)

        if self.__srvReq.is_loc_grasp:
            if self.__first_grasp:
                rospy.loginfo("setting for the first pregrasp")
                translation = self.__preLocGraspRel_1[self.__modelName].translation
                rotation = self.__preLocGraspRel_1[self.__modelName].rotation
            else:
                rospy.loginfo("setting for the second pregrasp")
                translation = self.__preLocGraspRel_2[self.__modelName].translation
                rotation = self.__preLocGraspRel_2[self.__modelName].rotation

        else:
            rospy.loginfo("setting for the real pregrasp")
            translation = self.__preGraspRel[self.__modelName].translation
            rotation = self.__GraspRel[self.__modelName].rotation
            self.__first_grasp = True

        self.__preGraspPose.header.frame_id = "world"
        self.__preGraspPose.pose.position = translate_position(position, translation)
        self.__preGraspPose.pose.orientation = rotate_orientation(orientation, rotation)
        print("pregraspPose: {0}\n".format(self.__preGraspPose))


        
    def __setGraspPose(self):
        position = deepcopy(self.__objPose.pose.position)
        orientation = deepcopy(self.__objPose.pose.orientation)
        if self.__srvReq.is_loc_grasp:
            if self.__first_grasp:
                rospy.loginfo("setting for the first grasp")
                translation = self.__locGraspRel_1[self.__modelName].translation
                rotation = self.__locGraspRel_1[self.__modelName].rotation
                self.__first_grasp = False
            else:
                rospy.loginfo("setting for the second grasp")
                translation = self.__locGraspRel_2[self.__modelName].translation
                rotation = self.__locGraspRel_2[self.__modelName].rotation
        else: 
            rospy.loginfo("setting for the real grasp")
            translation = self.__preGraspRel[self.__modelName].translation
            rotation = self.__GraspRel[self.__modelName].rotation
            self.__first_grasp = True

        self.__graspPose.header.frame_id = "world"
        self.__graspPose.pose.position = translate_position(position, translation)
        self.__graspPose.pose.orientation = rotate_orientation(orientation, rotation)
        print("graspPose: {0}\n".format(self.__graspPose))

    def set_path_ori_constraints(self, stampedPoseList):
        # self.__graspPathOriConstraint.orientation = self.pose.__preGraspPose.orientation
        self.__graspPathConstraints.orientation_constraints = []
        ori_constraints = [] 
        for stampedPose in stampedPoseList:
            constraint = OrientationConstraint()
            constraint.header = stampedPose.header
            constraint.orientation = stampedPose.pose.orientation
            constraint.link_name = "world"
            constraint.absolute_x_axis_tolerance = 0.05
            constraint.absolute_y_axis_tolerance = 0.05
            constraint.absolute_z_axis_tolerance = 0.05
            ori_constraints.append(constraint)


    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def set_pose_goal(self, pose):
        self.__pose_goal = pose

    def go_to_goal(self):
        ## Planning to a Pose Goal
        ## We can plan a motion for this group to a desired pose for the end-effector
        self.group.set_pose_target(self.__pose_goal)
        self.group.set_goal_tolerance(0.005)
        #self.group.set_path_constraints(self.__graspPathConstraints)

        ## Plan and execute
        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()
        self.group.clear_path_constraints()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.group.get_current_pose()
        return all_close(self.__pose_goal, current_pose, 0.01)

    def localization_grasp(self):
        '''1. Go to preGrasp pose, 
           2. Down to graspPose with path orientation constraint
           3. Gripper close
           4. Gripper open
           5. Up to preGrasppose with path orientation constraint
        
        Arguments:
            req {GripSrvRequest} -- string modelName
                                    geometry_msgs/StampedPose pose
                                    bool is_loc_grasp
        '''

        self.setGripObj()
        self.set_pose_goal(self.__preGraspPose)
        print(self.__preGraspPose)
        self.go_to_goal()
        release()
        self.set_pose_goal(self.__graspPose)
        print(self.__graspPose)
        self.set_path_ori_constraints([self.__graspPose])
        self.go_to_goal()
        # close gripper
        grip()
        release()

        self.set_pose_goal(self.__preGraspPose)
        self.set_path_ori_constraints([self.__graspPose])
        self.go_to_goal()

        self.__first_grasp = False
    def resetAll(self):
        self.__graspPathOriConstraint = OrientationConstraint()
        self.__graspPathConstraints = Constraints()
        self.__first_grasp = True
        self.__srvReq_is_set = False
        self.__preGraspPose = PoseStamped()
        self.__graspPose = PoseStamped()

        self.group.clear_pose_targets()
        self.group.clear_path_constraints()

        

    def real_grasp(self):
        '''1. Go to preGrasp pose, 
           2. Down to graspPose with path orientation constraint
           3. Gripper close
           4. Up to preGrasppose with path orientation constraint
        '''

        self.setGripObj()
        self.set_pose_goal(self.__preGraspPose)
        self.go_to_goal()
        release()
        self.set_pose_goal(self.__graspPose)
        self.set_path_ori_constraints([self.__graspPose])
        self.go_to_goal()
        # close gripper
        grip()
        rospy.sleep(1.0)
        self.set_pose_goal(self.__preGraspPose)
        self.set_path_ori_constraints([self.__graspPose])
        self.go_to_goal()



    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:
        ##
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                            waypoints,   # waypoints to follow
                                            0.01,        # eef_step
                                            0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL


    def display_trajectory(self, plan):
        ## Displaying a Trajectory
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

gripGrp = GripperInterface()
modelList = ["lf064-01", "lf064-02", "lf064-03", "lf064-04", "lf064-05"]



def gripSrvCb(req):
    resp = GripSrvResponse()
    resp.success = False
    rospy.loginfo("Srv moveGrpSrv recieved the request")

    # global variable
    gripGrp.setSrvReq(req)
    
    if req.modelName not in modelList:
        rospy.logwarn("Given model is not in the list")

    if req.is_loc_grasp:
        gripGrp.localization_grasp()
        gripGrp.localization_grasp()

    else:
        gripGrp.real_grasp()

    gripGrp.resetAll()

    resp.success = True
    rospy.loginfo("pick service finished")
    return resp


def homingSrvCb(req):
    gripper_socker_.homing()
    return {'sucecss': True}

def gripSrvCb(req):
    gripper_socker_.grip()
    return {'sucecss': True}

def releaseSrvCb(req):
    gripper_socker_.release()
    return {'sucecss': True}

def ackSrvCb(req):
    gripper_socker_.ack_fast_stop()
    return {'sucecss': True}

def gripperServer():
    rospy.init_node('gripperSrv')
    s = rospy.Service('pickSrv', GripSrv, gripSrvCb)

    if not sim_:
        homingSrv = rospy.Service('homingSrv', Empty, homingSrvCb)
        gripSrv = rospy.Service('gripSrv', Empty, gripSrvCb)
        releaseSrv = rospy.Service('releaseSrv', Empty, releaseSrvCb)
        ackSrv = rospy.Service('ackSrv', Empty, ackSrvCb)

    print("Service Servers are launched!")
    rospy.spin()


## Test Command
# rosservice call /pickSrv "modelName: 'lf064-01'
# stampedPose:
#   header:
#     seq: 0
#     stamp:
#       secs: 0
#       nsecs: 0
#     frame_id: 'world'
#   pose:
#     position:
#       x: -0.6
#       y: 0.2
#       z: 1.4
#     orientation:
#       x: 1
#       y: 0
#       z: 0
#       w: 0
# is_loc_grasp: true"


if __name__ == '__main__':
  # main()
    if len(sys.argv) > 1:
        sim_ = sys.argv[1]
    gripperServer()
