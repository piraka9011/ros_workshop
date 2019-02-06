#!/usr/bin/env python

"""RSDK Inverse Kinematics Example
A simple example of using the Rethink Inverse Kinematics
Service which returns the joint angles and validity for
a requested Cartesian Pose.
Run this example, the example will use the default limb
and call the Service with a sample Cartesian
Pose, pre-defined in the example code, printing the
response of whether a valid joint solution was found,
and if so, the corresponding joint angles.
"""
import argparse

# ROS Specific imports
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# Robot specific imports
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from intera_core_msgs.msg import JointCommand

# Dynamic reconfigure imports
from dynamic_reconfigure.server import Server
from ros_workshop.cfg import WorkshopConfig


class IkServiceClass:
    """A simple class that updates an end effector goal based on
    dynamic configuration.
    dyn_cfg_callback: Dynamic reconfigure callback to update goal
    ik_service_client: Service caller to execute IK
    """
    def __init__(self):
        # Messages to set the goal position
        self.seq = 1
        self.dyn_point = Point()
        self.dyn_quat = Quaternion()
        self.goal_pose = PoseStamped()
        self.joint_cmd = JointCommand()
        self.joint_cmd.header.seq = self.seq
        self.joint_cmd.mode = JointCommand.POSITION_MODE
        self._joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                             'right_j4', 'right_j5', 'right_j6']
        self.joint_cmd.names = self._joint_names
        # Create a publisher to send joint position commands
        self.joint_publisher = rospy.Publisher('/robot/limb/right/joint_command',
                                               JointCommand, queue_size=5)
        # Wait for subscribers to listen
        while self.joint_publisher.get_num_connections() == 0:
            pass

    def dyn_cfg_callback(self, config, level):
        rospy.loginfo("Reconfigure Request: \nx: {x_goal}, y: {y_goal}, z: {z_goal}\n"
                      "ax: {ax_goal}, ay: {ay_goal}, az: {az_goal}".format(**config))
        # Your code here

        return config

    def ik_service_client(self, goal=None, limb="right", use_advanced_options=False):
        # Setup for the service request
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        if goal is None:
            hdr = Header(stamp=rospy.Time.now(), frame_id='base')
            poses = {
                'right': PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position=Point(
                            x=0.450628752997,
                            y=0.161615832271,
                            z=0.217447307078,
                        ),
                        orientation=Quaternion(
                            x=0.704020578925,
                            y=0.710172716916,
                            z=0.00244101361829,
                            w=0.00194372088834,
                        ),
                    ),
                ),
            }
            # Add desired pose for inverse kinematics
            ikreq.pose_stamp.append(poses[limb])
        else:
            ikreq.pose_stamp.append(goal)
        # Request inverse kinematics from base to "right_hand" link
        ikreq.tip_names.append('right_hand')

        if use_advanced_options:
            # Optional Advanced IK parameters
            rospy.loginfo("Running Advanced IK Service Client example.")
            # The joint seed is where the IK position solver starts its optimization
            ikreq.seed_mode = ikreq.SEED_USER
            seed = JointState()
            seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                         'right_j4', 'right_j5', 'right_j6']
            seed.position = [0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
            ikreq.seed_angles.append(seed)

            # Once the primary IK task is solved, the solver will then try to bias the
            # the joint angles toward the goal joint configuration. The null space is 
            # the extra degrees of freedom the joints can move without affecting the
            # primary IK task.
            ikreq.use_nullspace_goal.append(True)
            # The nullspace goal can either be the full set or subset of joint angles
            goal = JointState()
            goal.name = ['right_j1', 'right_j2', 'right_j3']
            goal.position = [0.1, -0.3, 0.5]
            ikreq.nullspace_goal.append(goal)
            # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
            # If empty, the default gain of 0.4 will be used
            ikreq.nullspace_gain.append(0.4)
        else:
            rospy.loginfo("Running Simple IK Service Client example.")

        # Call the IK Service
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        # Check if result valid, and type of seed ultimately used to get solution
        if resp.result_type[0] > 0:
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp.result_type[0], 'None')
            rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  (seed_str,))
            # Format solution into Limb API-compatible dictionary
            self.joint_cmd.header.stamp = rospy.Time()
            self.joint_cmd.names = resp.joints[0].name
            self.joint_cmd.position = resp.joints[0].position
            self.joint_publisher.publish(self.joint_cmd)
            rospy.loginfo("Publishing Joint Message: {}".format(self.joint_cmd))
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
            rospy.loginfo("------------------")
            rospy.loginfo("Response Message:\n%s", resp)
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            rospy.logerr("Result Error %d", resp.result_type[0])
            return False

        return True

    def test_ik_service(self, advanced=False):
        rospy.loginfo("Testing IK Service...")
        if self.ik_service_client(use_advanced_options=advanced):
            if advanced:
                rospy.loginfo(rospy.loginfo("Advanced IK call passed!"))
            else:
                rospy.loginfo("Simple IK call passed!")
        else:
            if advanced:
                rospy.loginfo("Advanced IK call passed!")
            else:
                rospy.logerr("Simple IK call FAILED")

    def test_publisher(self):
        rospy.loginfo("Testing publisher...")
        self.joint_cmd.header = Header(seq=self.seq+1, stamp=rospy.Time.now())
        self.joint_cmd.position = [0.5, 1.1, 0.0, 0.42, -0.8, -0.05, 0.02]
        rospy.loginfo("Publishing joint command: {}".format(self.joint_cmd))
        self.joint_publisher.publish(self.joint_cmd)

    def start_spinning(self):
        # Create the dyanmic reconfigure server
        dyn_cfg_server = Server(WorkshopConfig, self.dyn_cfg_callback)
        rospy.spin()


if __name__ == '__main__':
    # ===================================================================================
    # Parse input arguments
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description="ROS Workshop IK Solver service")
    parser.add_argument("-tp", "--testpub", action="store_true",
                        help="Test the joint position publisher")
    parser.add_argument("-ts", "--testsrv", action="store_true",
                        help="Test the IK service call")
    args = parser.parse_args(rospy.myargv()[1:])
    # ===================================================================================

    # Create a ROS node
    rospy.init_node('workshop_ik_service')
    # Instantiate the class
    ik_service = IkServiceClass()
    # Test
    if args.testpub:
        ik_service.test_publisher()
    if args.testsrv:
        ik_service.test_ik_service()
    # Run
    ik_service.start_spinning()
