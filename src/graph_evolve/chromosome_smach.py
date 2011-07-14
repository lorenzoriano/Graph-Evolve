import smach
import random
import time

def convert_chromosome_smach(chromosome, 
                             names_mapping, 
                             transitions_mapping,
                             classes_mapping,
                             data_mapping):
    
    sm = smach.StateMachine(outcomes=['success', "failure", "timeout"],
                            )
    
    with sm:
        for node in chromosome.nodes:
            transitions = {"timeout":"timeout"}
            state_name = names_mapping[node.type_id] + "_" + str(node.number)
            edges = node.out_edges
            
            for edge_number, edge in edges.iteritems():
                dest_state = (names_mapping[chromosome[edge].type_id] +
                              "_" + str(edge))        
                state_output = (transitions_mapping[names_mapping[node.type_id]]
                                [edge_number] )
                transitions[state_output] = dest_state
            
            state_class = classes_mapping[names_mapping[node.type_id]]
            if names_mapping[node.type_id] == "CheckSuccess":
                transitions={'success': 'success',
                             'failure': 'failure',
                             'timeout': 'timeout'
                             }                               
            remapping = data_mapping[names_mapping[node.type_id]]
            if len(node.params):
                state = state_class(node.params[:])
            else:
                state = state_class()
            smach.StateMachine.add(state_name, state, 
                                   transitions = transitions,
                                   remapping = remapping)
        #setting the initial state
        node = chromosome[chromosome.starting_node]
        state_name = names_mapping[node.type_id] + "_" + str(node.number)
        sm.set_initial_state([state_name])

    return sm

try:
    import pygraphviz as pgv
except ImportError:
    pass

import itertools
def convert_chromosome_pygraphviz(chromosome, 
                                  names_mapping, 
                                  transitions_mapping):
    G = pgv.AGraph(strict = False, directed=True)
    for node in chromosome.nodes:        
        transitions = []
        labels = []
        state_name = names_mapping[node.type_id] + "_" + str(node.number)
        G.add_node(state_name)
        edges = node.out_edges

        for edge_number, edge in edges.iteritems():
            dest_state = (names_mapping[chromosome[edge].type_id] +
                          "_" + str(edge))                            
            state_output = (transitions_mapping[names_mapping[node.type_id]]
                            [edge_number] )
            transitions.append(dest_state)
            labels.append(state_output)
        for t,l in itertools.izip(transitions, labels):
            G.add_edge(state_name, t, label=l)
    
    node = chromosome[chromosome.starting_node]
    state_name = names_mapping[node.type_id] + "_" + str(node.number)
    G.get_node(state_name).attr["color"] = "red"
    return G


def test():
    
    import smach_grasp
    from smach_grasp import (names_mapping, classes_mapping, transitions_mapping,
                         data_mapping, node_degrees, nodes_params)

    
    import graph_genome
    random.seed()
    
    smach.set_loggers(smach.loginfo,
                      smach_grasp.null_print,
                      smach_grasp.null_print,
                      smach_grasp.null_print)
    
    num_nodes = 20
    
    start = time.time()
    chromosome = graph_genome.GraphGenome(num_nodes, 
                                          node_degrees,
                                          nodes_params)
    chromosome.initialize()
    G = convert_chromosome_pygraphviz(chromosome, names_mapping, 
                                      transitions_mapping)
    G.write("/home/pezzotto/tmp/tmp.dot")
    
    sm = convert_chromosome_smach(chromosome, 
                                  names_mapping, 
                                  transitions_mapping, 
                                  classes_mapping,
                                  data_mapping)
    robot_state =  smach_grasp.RobotWorldState()
    sm.userdata.robot_state = robot_state
    
    try:
        outcome = sm.execute()
    except smach.exceptions.InvalidUserCodeError as err:
        print "Error: ", err.message
    else:
        print "Outcome: ", outcome
    print "Distance is: ",  smach_grasp.state_dist_gripper_object(sm.userdata.robot_state)
    print "Time: ", time.time()-start


def test_solution():
    import networkx as nx
    import pyevolve
    import graph_genome
    import smach_grasp
    from smach_grasp import (names_mapping, classes_mapping, transitions_mapping,
                         data_mapping, node_degrees, nodes_params)

    
    G = nx.MultiDiGraph()
    G.add_node(0, type_id = 0, parameters=pyevolve.G1DList.G1DList(0))
    G.add_node(1, type_id = 1, parameters=pyevolve.G1DList.G1DList(0))
    G.add_node(2, type_id = 2, parameters=pyevolve.G1DList.G1DList(0))
    G.add_node(3, type_id = 4, parameters=pyevolve.G1DList.G1DList(3))
    G.add_node(4, type_id = 5, parameters=pyevolve.G1DList.G1DList(4))
    G.add_node(5, type_id = 3, parameters=pyevolve.G1DList.G1DList(0))

    G.node[3]["parameters"][:] = (0.5,0.5,0.5)
    G.node[4]["parameters"][:] = (0.75,0.5,0.5)
    
    G.add_edge(0,1, action_number = 0)
    G.add_edge(1,2, action_number = 0)
    G.add_edge(1,1, action_number = 1)
    G.add_edge(2,3, action_number = 0)
    G.add_edge(2,2, action_number = 1)
    G.add_edge(3,4, action_number = 0)
    G.add_edge(3,3, action_number = 1)
    G.add_edge(4,5, action_number = 0)
    
    genome = graph_genome.GraphGenome(len(G), 
                                      node_degrees, 
                                      nodes_params
                                      )
    genome.graph = G
    genome.starting_node = 0
    genome.prune_non_connected()
    G = convert_chromosome_pygraphviz(genome, names_mapping, 
                                      transitions_mapping)
    G.write("/home/pezzotto/tmp.dot")
    
    sm = convert_chromosome_smach(genome, 
                                  names_mapping, 
                                  transitions_mapping, 
                                  classes_mapping,
                                  data_mapping)
    
    sm.userdata.robot_state = smach_grasp.RobotWorldState()
    
    try:
        outcome = sm.execute()
    except smach.exceptions.InvalidUserCodeError as err:
        print "Error: ", err.message
    else:
        print "Outcome: ", outcome
    
    print "STATE:"
    print sm.userdata.robot_state
    
    print "DIST1: ", smach_grasp.state_dist_gripper_object(sm.userdata.robot_state,
                                                           smach_grasp.target_pos)
    print "DIST2: ", smach_grasp.state_dist_gripper_rotation(sm.userdata.robot_state,
                                                           smach_grasp.target_rot)

if __name__=="__main__":
    test_solution()    
        