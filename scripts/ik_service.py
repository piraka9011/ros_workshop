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
    JointCommand
)

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
        self.dyn_point = Point()
        self.dyn_quat = Quaternion()
        self.goal_pose = PoseStamped()
        self.joint_cmd = JointCommand()
        self.joint_cmd.mode = JointCommand.POSITION_MODE
        # Create the dyanmic reconfigure server
        self.dyn_cfg_server = Server(WorkshopConfig, self.dyn_cfg_callback)
        # Create a publisher to send joint position commands
        self.joint_publisher = rospy.Publisher('/robot/limb/right/joint_command',
                                               JointCommand, queue_size=5)

    def dyn_cfg_callback(self, config, level):
        rospy.loginfo("Reconfigure Request: \nx: {x_goal}, y: {y_goal}, z: {z_goal}\n"
                      "ax: {ax_goal}, ay: {ay_goal}, az: {az_goal}".format(**config))
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        # Your code here
        self.dyn_point.x = config.x_goal
        self.dyn_point.y = config.y_goal
        self.dyn_point.z = config.z_goal
        self.dyn_quat.x = config.ax_goal
        self.dyn_quat.y = config.ay_goal
        self.dyn_quat.z = config.az_goal
        self.dyn_quat.w = 0.00
        pose = Pose(position=self.dyn_point, orientation=self.dyn_quat)
        self.goal_pose = PoseStamped(header=header, pose=pose)
        self.ik_service_client(goal=self.goal_pose)
        return config

    def ik_service_client(self, goal=None, limb="right", use_advanced_options=False):
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

    def start_spinning(self):
        rospy.spin()


if __name__ == '__main__':
    # Create a ROS node
    rospy.init_node('workshop_ik_service')
    # Instantiate the class
    ik_service = IkServiceClass()
    # Test
    ik_service.test_ik_service()
    # Run
    # ik_service.start_spinning()
