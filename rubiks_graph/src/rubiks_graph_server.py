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

PKG = "rubiks_graph"
import roslib
roslib.load_manifest(PKG)
import rospy
import actionlib
import rubiks_graph.msg
from pr2_control_utilities import RobotState, PR2JointMover

import graph_create
import networkx as nx
import os


class GraphServer(object):
    '''
    The class responsible for executing the actions to solve the rubik's cube. It could be
    easily adapted to execute a generic sequence of actions.
    '''
    
    actions_mapping = {"INIT" :         ["observation_prep"],
                       "INIT_CLOSED" :  ["observation_prep_close"],
                       "INSPECT_F" :    ["observation_F"],
                       "INSPECT_U" :    ["observation_U"],
                       "INSPECT_B" :    ["observation_B"],
                       "INSPECT_L" :    ["observation_L"],
                       "INSPECT_D" :    ["observation_D"],
                       "INSPECT_R" :    ["observation_R"],
                       "U":             ["RF_U_C", "RB_U_C"],
                       "U2":            ["RF_U_D", "RB_U_D"],
                       "Ui":            ["RF_Ui_C", "RB_Ui_C"],
                       "U2i":           ["RF_Ui_D", "RB_Ui_D"],
                       "F":             ["RF_F_D"],
                       "F2":            ["RF_F_E"],
                       "Fi":            ["RF_Fi_D"],
                       "F2i":           ["RF_Fi_E"],
                       "B":             ["RB_B_D"],
                       "B2":            ["RB_B_E"],
                       "Bi":            ["RB_Bi_D"],
                       "B2i":           ["RB_Bi_E"],
                       "R":             ["LU_R_D"],
                       "R2":            ["LU_R_E"],
                       "Ri":            ["LU_Ri_D"],
                       "R2i":           ["LU_Ri_E"],
                       "D":             ["LU_D_C", "LD_D_C"],
                       "D2":            ["LU_D_D", "LD_D_D"],
                       "Di":            ["LU_Di_C", "LD_Di_C"],
                       "D2i":           ["LU_Di_D", "LD_Di_C"],
                       "L":             ["LD_L_D"],
                       "L2":            ["LD_L_E"],
                       "Li":            ["LD_Li_D"],
                       "L2i":           ["LD_Li_E"],                       
                       }
    head_point_mapping = {"observation_F" :    "r_gripper",
                          "observation_U" :    "r_gripper",
                          "observation_B" :    "r_gripper",
                          "observation_L" :    "l_gripper",
                          "observation_D" :    "l_gripper",
                          "observation_R" :    "l_gripper",
                          }

    
    def __init__(self, name, nodes_dir):
        
        self.name = name        
        rospy.loginfo("Loading Graph")
        self.G = graph_create.create_graph(nodes_dir)
#        self.G = nx.read_gpickle(os.path.join(nodes_dir, "graph.dat"))
        rospy.loginfo("Graph loaded")
        
        #Dancing stuff
        robot_state = RobotState()        
        self.dance1_mover = PR2JointMover(robot_state)
        filename = os.path.join(nodes_dir, "dance1"+".pos")
        self.dance1_mover.parse_bookmark_file(filename)
        
        self.dance2_mover = PR2JointMover(robot_state)
        filename = os.path.join(nodes_dir, "dance2"+".pos")
        self.dance2_mover.parse_bookmark_file(filename)
        
        self.current_state = None
        rospy.loginfo("Moving to init pose")
        self.init_pose("observation_prep")        
        self.server = actionlib.SimpleActionServer(self.name, 
                                                   rubiks_graph.msg.MoveToStateAction,
                                                   execute_cb = self.execute_cb)
        self.server.start()        
        rospy.loginfo("Waiting for requests")        
    
    def init_pose(self, init_state):
        '''
        Move to the state defined by init_state
        @param init_state: the target state
        '''
        mover = self.G.node[init_state]["mover"]
        mover.execute_and_wait()
        self.current_state = init_state
        rospy.loginfo("Current state: %s"%self.current_state)
    
    def __point_head(self, node, mover):
        try:
            gripper = self.head_point_mapping[node]
            mover.point_head_gripper(gripper)
        except KeyError:
            return
    
    def execute_path(self, path, index=0):    
        '''
        Execute a path
        @param path: a list of state names to follow
        @param index: an optional index to use for logging purpose
        '''
        for node in path:
            if node == self.current_state:
                continue
            mover = self.G.node[node]["mover"]
            rospy.loginfo("Executing %s"%mover.name)
            mover.time_to_reach = 1.5
            mover.execute_and_wait()
#            self.__point_head(node, mover)
            self.current_state = node
            rospy.loginfo("Current state: %s"%self.current_state)
            
            feedback = rubiks_graph.msg.MoveToStateFeedback()
            feedback.action_number = index
            feedback.status = self.current_state            
            self.server.publish_feedback(feedback)            

    def find_best_bath(self, goal_states):
        '''
        Find the shortest path between the current state and a goal state.
        Returns a list of all the intermediate states to reach the goal_state.
        @param goal_states: a list of possible states to reach. The shortest among all the
        paths is taken.
        '''
        bestpath = []
        bestlength = 100000
        for state in goal_states:
            path = nx.algorithms.shortest_path(self.G, self.current_state, state)
            if type(path) is bool:
                continue
            if len(path) < bestlength:
                bestpath = path
                bestlength = len(bestpath)
        return bestpath

    def execute_chain(self, chain):        
        '''
        Follows a chain of states. 
        @param chain: The list of states to go through.
        '''
        if chain[0] == "INIT":
            self.init_pose("observation_prep")
            return True
        
        for index, node in enumerate(chain):            
            if node not in self.actions_mapping:
                goal_states = [node]
            else:
                goal_states = self.actions_mapping[node]
            path = self.find_best_bath(goal_states)
            if len(path) == 0:
                rospy.logerr("Error: there is no path from %s to %s" 
                             %(self.current_state, goal_states))
                return False
            
            self.execute_path(path, index)
            rospy.loginfo("Current state after chain is %s"
                          %self.current_state)      
        
        return True
    
    def dance(self):        
        '''
        Special method to execute the solving dance.
        '''
        self.dance1_mover.time_to_reach = 3.0
        self.dance2_mover.time_to_reach = 3.0
        
        if self.current_state[0] == "R":
            self.dance2_mover.open_left_gripper(True)
        else:
            self.dance2_mover.open_right_gripper(True)
        rospy.sleep(1)
        
        self.dance1_mover.execute_and_wait()
        self.dance2_mover.execute_and_wait()
        self.dance2_mover.spin_wrists(10)
        
    def execute_cb(self, goal):
        rospy.loginfo("Received request for state: %s"%goal)
        chain = goal.actions
        if chain[0] == "DANCE":
            self.dance()
            self.server.set_succeeded()
            return
        if not self.execute_chain(chain):
            self.server.set_aborted()
        else:
            self.server.set_succeeded()
    
if __name__ == "__main__":
    rospy.init_node('rubiks_graph', anonymous=False)
    DIR = roslib.packages.get_pkg_dir(PKG, required=True) + "/RCube/"    
    graph_server = GraphServer(rospy.get_name(), DIR)
    
    rospy.spin()



