#! /usr/bin/env python
# Copyright (c) 2010, Lorenzo Riano.
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
#         * Neither the name of the Lorenzo Riano. nor the names of its
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

# Author: Lorenzo Riano <lorenzo.riano@gmail.com>

PKG="rubiks_graph"
import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
import rubiks_graph.msg
import sys
from polled_camera.srv import GetPolledImage, GetPolledImageRequest

if __name__ == "__main__":
    rospy.init_node('rubiks_graph_tester', anonymous=False)
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) == 1:
        rospy.loginfo("Usage: %s state"%argv[0])
        rospy.signal_shutdown("")
        sys.exit()
    
    client = actionlib.SimpleActionClient('rubiks_graph', rubiks_graph.msg.MoveToStateAction)
    
    rospy.loginfo("Waiting for server")
    client.wait_for_server()
    rospy.loginfo("Server is ready")
    
    goal = rubiks_graph.msg.MoveToStateGoal()
    goal.actions = [argv[1]]
    
    client.send_goal_and_wait(goal)
    
#    rospy.loginfo("Asking for camera image")    
#    rospy.wait_for_service("/prosilica/request_image")
#    client = rospy.ServiceProxy("/prosilica/request_image", GetPolledImage)
#    req = GetPolledImageRequest()
#    req.request.response_namespace = "/prosilica_req"
#    client.call(req)
    
    rospy.loginfo("DONE")