#! /usr/bin/env python

PKG="rubiks_graph"
import roslib; roslib.load_manifest(PKG)
import rospy

import networkx as nx
import os
import time
from pr2_control_utilities import RobotState, PR2JointMover

def add_mover(G,node,robot_state, nodes_dir):    
    if node in G:
        return    
    pose_name = node
    mover = PR2JointMover(robot_state, pose_name)
    filename = os.path.join(nodes_dir, pose_name+".pos")
    mover.parse_bookmark_file(filename)
    G.add_node(pose_name, mover=mover)

def add_chain(chain, node_1, G, robot_state, nodes_dir, bidirectional):
    for node_2 in chain:        
        add_mover(G, node_2, robot_state,nodes_dir)
        G.add_edge(node_1, node_2)
        if bidirectional:
            G.add_edge(node_2, node_1)
        node_1 = node_2

def create_graph(nodes_dir):
    
    robot_state = RobotState()    
    
    G = nx.DiGraph()    
    
    node = "observation_prep"
    add_mover(G, node, robot_state, nodes_dir)
    node = "observation_prep_close"
    add_mover(G, node, robot_state, nodes_dir)
    node = "observation_B"
    add_mover(G, node, robot_state, nodes_dir)
    node = "observation_D"
    add_mover(G, node, robot_state, nodes_dir)
    node = "observation_F"
    add_mover(G, node, robot_state, nodes_dir)
    node = "observation_L"
    add_mover(G, node, robot_state, nodes_dir)
    node = "observation_R"
    add_mover(G, node, robot_state, nodes_dir)
    node = "observation_U"
    add_mover(G, node, robot_state, nodes_dir)
    node = "swap_right_left_step_a"
    add_mover(G, node, robot_state, nodes_dir)
    node = "swap_right_left_step_b"
    add_mover(G, node, robot_state, nodes_dir)
    node = "swap_right_left_step_c"
    add_mover(G, node, robot_state, nodes_dir)
    node = "swap_right_left_step_d"
    add_mover(G, node, robot_state, nodes_dir)
    node = "swap_right_left_step_e"
    add_mover(G, node, robot_state, nodes_dir)
    node = "initA"
    add_mover(G, node, robot_state, nodes_dir)
    node = "initB"
    add_mover(G, node, robot_state, nodes_dir)
    node = "initC"
    add_mover(G, node, robot_state, nodes_dir)
    node = "initD"
    add_mover(G, node, robot_state, nodes_dir)
    
    
    node_1 = "observation_prep"
    node_2 = "observation_prep_close"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_prep_close"
    node_2 = "observation_F"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_prep_close"
    node_2 = "observation_B"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_prep_close"
    node_2 = "observation_U"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_F"
    node_2 = "observation_U"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_F"
    node_2 = "observation_B"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_U"
    node_2 = "observation_B"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_F"
    node_2 = "swap_right_left_step_a"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_U"
    node_2 = "swap_right_left_step_a"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_B"
    node_2 = "swap_right_left_step_a"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "swap_right_left_step_a"
    node_2 = "swap_right_left_step_b"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "swap_right_left_step_b"
    node_2 = "swap_right_left_step_c"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "swap_right_left_step_c"
    node_2 = "swap_right_left_step_d"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "swap_right_left_step_d"
    node_2 = "swap_right_left_step_e"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "swap_right_left_step_e"
    node_2 = "observation_L"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "swap_right_left_step_e"
    node_2 = "observation_D"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_L"
    node_2 = "observation_D"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_L"
    node_2 = "observation_R"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_D"
    node_2 = "observation_R"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)

    node_1 = "observation_L"
    node_2 = "initA"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)    
    
    node_1 = "observation_D"
    node_2 = "initA"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "observation_R"
    node_2 = "initA"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "initA"
    node_2 = "initB"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "initB"
    node_2 = "initC"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "initC"
    node_2 = "initD"
    G.add_edge(node_1, node_2)
    G.add_edge(node_2, node_1)
    
    node_1 = "R_F_RL"
    add_mover(G, node_1, robot_state,nodes_dir)
    G.add_edge("initD", "R_F_RL")
    
    chain = ["RF_LD_A", "RF_LD_B", "RF_LD_C", "L_D_FB", "LD_RB_A", "LD_RB_B", "LD_RB_C", "R_B_RL", "RB_LU_A", "RB_LU_B", "RB_LU_C",
             "L_U_FB", "LU_RF_A","LU_RF_B","LU_RF_C","R_F_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=True)    
    
    #Cycles for R_F_RL    
    #Action U
    node_1 = "R_F_RL"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RF_U_A", "RF_U_B", "RF_U_C","RF_U_D","RF_U_E","R_F_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "RF_U_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RF_U_F", "R_F_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "RF_U_B"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RF_Ui_C","RF_Ui_D","RF_Ui_E","R_F_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    node_1 = "RF_Ui_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RF_Ui_F", "R_F_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    #Action F
    node_1 = "R_F_RL"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RF_F_A", "RF_F_B", "RF_F_C","RF_F_D","RF_F_E", "RF_F_F", "R_F_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "RF_F_D"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RF_F_G", "R_F_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "RF_F_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RF_Fi_D","RF_Fi_E","RF_Fi_F","R_F_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    node_1 = "RF_Fi_D"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RF_Fi_G", "R_F_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    #Cycles for R_B_RL    
    #Action U
    node_1 = "R_B_RL"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RB_U_A", "RB_U_B", "RB_U_C","RB_U_D","RB_U_E","R_B_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "RB_U_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RB_U_F", "R_B_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "RB_U_B"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RB_Ui_C","RB_Ui_D","RB_Ui_E","R_B_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    node_1 = "RB_Ui_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RB_Ui_F", "R_B_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    #Action B
    node_1 = "R_B_RL"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RB_B_A", "RB_B_B", "RB_B_C","RB_B_D","RB_B_E", "RB_B_F", "RB_B_H","R_B_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "RB_B_D"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RB_B_G", "RB_B_H", "R_B_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "RB_B_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RB_Bi_D","RB_Bi_E","RB_Bi_F","RB_Bi_H","R_B_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    node_1 = "RB_Bi_D"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["RB_Bi_G", "RB_Bi_H", "R_B_RL"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    #Cycles for L_D_FB    
    #Action D
    node_1 = "L_D_FB"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LD_D_A", "LD_D_B", "LD_D_C","LD_D_D","LD_D_E","L_D_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "LD_D_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LD_D_F", "L_D_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "LD_D_B"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LD_Di_C","LD_Di_D","LD_Di_E","L_D_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    node_1 = "LD_Di_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LD_Di_F", "L_D_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    #Action L
    node_1 = "L_D_FB"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LD_L_A", "LD_L_B", "LD_L_C","LD_L_D","LD_L_E", "LD_L_F", "LD_L_H","L_D_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "LD_L_D"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LD_L_G", "LD_L_H", "L_D_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "LD_L_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LD_Li_D","LD_Li_E","LD_Li_F","LD_Li_H","L_D_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    node_1 = "LD_Li_D"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LD_Li_G", "LD_Li_H", "L_D_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    #Cycles for L_U_FB    
    #Action D
    node_1 = "L_U_FB"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LU_D_A", "LU_D_B", "LU_D_C","LU_D_D","LU_D_E","L_U_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "LU_D_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LU_D_F", "L_U_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "LU_D_B"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LU_Di_C","LU_Di_D","LU_Di_E","L_U_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    node_1 = "LU_Di_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LU_Di_F", "L_U_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    #Action L
    node_1 = "L_U_FB"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LU_R_A", "LU_R_B", "LU_R_C","LU_R_D","LU_R_E", "LU_R_F", "LU_R_H","L_U_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "LU_R_D"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LU_R_G", "LU_R_H", "L_U_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    node_1 = "LU_R_C"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LU_Ri_D","LU_Ri_E","LU_Ri_F","LU_Ri_H","L_U_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)

    node_1 = "LU_Ri_D"
    add_mover(G, node_1, robot_state,nodes_dir)
    chain = ["LU_Ri_G", "LU_Ri_H", "L_U_FB"]
    add_chain(chain,node_1,G,robot_state,nodes_dir, bidirectional=False)
    
    return G

def execute_path(G, path):
    
    for node in path:
        mover = G.node[node]["mover"]
        print "Executing ", mover.name
        mover.execute_and_wait()

def execute_path_chain(G, start_node, chain):
    for node in chain:
        path = nx.algorithms.shortest_path(G, start_node, node)
        execute_path(G, path)
        start_node = node
        
    
if __name__ == "__main__":
    rospy.init_node('kubiks_graph', anonymous=True)
    
    DIR = roslib.packages.get_pkg_dir(PKG, required=True) + "/RCube/"    
    now = time.time() 
    G = create_graph(DIR)
    nx.write_gpickle(G, os.path.join(DIR, "graph.dat"))    
#    G = nx.read_gpickle(os.path.join(DIR, "graph.dat"))
    print "Graph loading time: ", time.time() - now

#    path = nx.algorithms.shortest_path(G, "LU_D_D", "R_F_RL")
#    execute_path(G, path)
    
    import matplotlib.pyplot as plt
    nx.draw(G)
    plt.show()