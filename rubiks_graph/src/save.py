#! /usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#         * Redistributions of source code must retain the above copyright
#             notice, this list of conditions and the following disclaimer.
#         * Redistributions in binary form must reproduce the above copyright
#             notice, this list of conditions and the following disclaimer in the
#             documentation and/or other materials provided with the distribution.
#         * Neither the name of the Willow Garage, Inc. nor the names of its
#             contributors may be used to endorse or promote products derived from
#             this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author:  Author: Lorenzo Riano <lorenzo.riano@gmail.com> (based on code from Jon Scholz)

PKG="pr2_pose_saver"
import roslib; roslib.load_manifest(PKG)
import rospy
import time
import sys

from geometry_msgs.msg import Twist, Pose, PoseStamped, TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import *

class PoseSaver(object):
    def __init__(self):
        rospy.loginfo("Waiting for joint_states")
        self.joint_states = rospy.wait_for_message("joint_states", JointState)
        
        self.r_joints = ['r_shoulder_pan_joint', 
                         'r_shoulder_lift_joint', 
                         'r_upper_arm_roll_joint', 
                         'r_elbow_flex_joint', 
                         'r_forearm_roll_joint', 
                         'r_wrist_flex_joint', 
                         'r_wrist_roll_joint']        
        self.l_joints = ['l_shoulder_pan_joint', 
                         'l_shoulder_lift_joint', 
                         'l_upper_arm_roll_joint', 
                         'l_elbow_flex_joint', 
                         'l_forearm_roll_joint', 
                         'l_wrist_flex_joint', 
                         'l_wrist_roll_joint']
        self.head_joints = ['head_pan_joint', 'head_tilt_joint']        
        self.torso_joints = ['torso_lift_joint']
        self.laser_tilt_joints = ['laser_tilt_mount_joint']
        
    def find_positions(self, joints):
        positions = []
        for joint in joints:
            if not joint in self.joint_states.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint_name)
                return None
            index = self.joint_states.name.index(joint)
            positions.append(self.joint_states.position[index])
        return positions
    
    def save_left_arm_pose_to_file(self, file_handle):
        jpos = self.find_positions(self.l_joints)
        if jpos is None:
            return
        f.write("l_arm:")
        f.write(", ".join(str(i) for i in jpos))
        f.write("\n")
    
    def save_right_arm_pose_to_file(self, file_handle):
        jpos = self.find_positions(self.r_joints)
        if jpos is None:
            return
        f.write("r_arm:")
        f.write(", ".join(str(i) for i in jpos))
        f.write("\n")
    
    def save_head_pose_to_file(self,file_handle):
        jpos = self.find_positions(self.head_joints)
        if jpos is None:
            return
        f.write("head:")
        f.write(", ".join(str(i) for i in jpos))
        f.write("\n")
    
    def save_torso_position_to_file(self,file_handle):
        jpos = self.find_positions(self.torso_joints)
        if jpos is None:
            return
        f.write("torso:")
        f.write(", ".join(str(i) for i in jpos))
        f.write("\n")
    
    def save_gripper_angle_to_file(self,file_handle, gripper):
        gripper_state = rospy.wait_for_message("/%s_gripper_controller/state" % gripper, JointControllerState) # r or l
        jpos = gripper_state.process_value
        f.write("%s_gripper: %f\n" % (gripper, jpos))
    
    def save_all(self, file):
        self.joint_states = rospy.wait_for_message("joint_states", JointState)
        self.save_right_arm_pose_to_file(file)
        self.save_left_arm_pose_to_file(file)
        self.save_gripper_angle_to_file(file, 'r')
        self.save_gripper_angle_to_file(file, 'l')
        self.save_head_pose_to_file(file)
        self.save_torso_position_to_file(file)
        self.save_tilting_laser_angle_to_file(file)
        self.save_base_position_to_file(file)
    
    def save_tilting_laser_angle_to_file(self,file_handle):
        pass
    
    def save_base_position_to_file(self,file_handle):
        pass
    

if __name__ == "__main__":
    from optparse import OptionParser
    usage = "usage: %prog [options] [filename]"
    parser = OptionParser(usage=usage)
    parser.add_option("-a", "--all", action="store_true", dest="save_all", 
                      help="save all", default=False)
    parser.add_option("-r", "--right_arm", action = "store_true", dest="save_right_arm",
                      help="Save pose of right arm", default=False)
    parser.add_option("-l", "--left_arm", action = "store_true", dest="save_left_arm",
                      help="Save pose of left arm", default=False)
    parser.add_option("-R", "--right-gripper", action = "store_true", dest="save_right_gripper",
                      help="Save pose of right gripper", default=False)
    parser.add_option("-L", "--left-gripper", action = "store_true", dest="save_left_gripper",
                      help="Save pose of right gripper", default=False)
    parser.add_option("-H", "--head", action = "store_true", dest="save_head",
                      help="Save pose of head", default=False)
    parser.add_option("-t", "--torso", action = "store_true", dest="save_torso",
                      help="Save pose of torso", default=False)
    parser.add_option("-T", "--tilting-laser", action = "store_true", dest="save_laser",
                      help="Save pose of tilting laser", default=False)
    (options, args) = parser.parse_args()

    rospy.init_node('bookmarker_listener', anonymous=True)

    filename = ""
    if len(args) == 1:
        filename = args[0]
    else:
        datestr="%s_%s_%s-%s:%s:%s" % (time.localtime().tm_mon,time.localtime().tm_mday,time.localtime().tm_year,
                                       time.localtime().tm_hour, time.localtime().tm_min,time.localtime().tm_sec)
        filename = "%s.pps" %  datestr

    f=open(filename, "w")
    
    saver = PoseSaver()
    # Step through requested topics and save#
    
    if options.save_right_arm or options.save_all:
        rospy.loginfo("Saving right arm")
        saver.save_right_arm_pose_to_file(f,)
    if options.save_left_arm or options.save_all:
        rospy.loginfo("Saving left arm")
        saver.save_left_arm_pose_to_file(f)
    if options.save_right_gripper or options.save_all:
        rospy.loginfo("Saving right gripper")
        saver.save_gripper_angle_to_file(f, 'r')
    if options.save_left_gripper or options.save_all:
        rospy.loginfo("Saving left gripper")
        saver.save_gripper_angle_to_file(f, 'l')
    if options.save_head or options.save_all:
        rospy.loginfo("Saving head")
        saver.save_head_pose_to_file(f)
#    if options.save_torso or options.save_all:
#        rospy.loginfo("Saving torso")
#        saver.save_torso_position_to_file(f)
    
    f.close()
