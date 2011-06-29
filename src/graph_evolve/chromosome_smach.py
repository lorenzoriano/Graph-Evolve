import smach
import random
import time

#transition_mapping = {"RecogniseObject":["success", "timeout"]}
def convert_chromosome_smach(chromosome, 
                             names_mapping, 
                             transitions_mapping,
                             classes_mapping,
                             data_mapping):
    
    sm = smach.StateMachine(outcomes=['success', "failure"])
    
    with sm:
        for node in chromosome.nodes:
            transitions = {"timeout":"failure"}
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
                             'timeout': 'failure'
                }                               
            remapping = data_mapping[names_mapping[node.type_id]]
            if len(node.params):
                state = state_class(node.params[:])
            else:
                state = state_class()
            smach.StateMachine.add(state_name, state, 
                                   transitions = transitions,
                                   remapping = remapping)
        node = chromosome[chromosome.starting_node]
        state_name = names_mapping[node.type_id] + "_" + str(node.number)
        sm.set_initial_state([state_name])

    return sm

import pygraphviz as pgv
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
    import graph_genome
    random.seed()
    
    smach.set_loggers(smach.loginfo,
                      smach_grasp.null_print,
                      smach_grasp.null_print,
                      smach_grasp.null_print)
    
    names_mapping = ["RecogniseObject",
                     "MoveToObject",
                     "GraspObject",
                     "CheckSuccess",
                     "MoveGripperToParam"]
    classes_mapping = {"RecogniseObject" : smach_grasp.RecogniseObject,
                     "MoveToObject": smach_grasp.MoveGripperTo,
                     "GraspObject": smach_grasp.GraspObject,
                     "CheckSuccess": smach_grasp.CheckSuccess,
                     "MoveGripperToParam" : smach_grasp.MoveGripperToParam}
    transitions_mapping = {'RecogniseObject':["success",],
                           'MoveToObject':["success", "failure",],
                           'GraspObject':["success", "failure",],
                           "CheckSuccess":[],
                           "MoveGripperToParam":["success", "failure"]}
    
    default_mapping = {'state_in':'robot_state',
                       'state_out':'robot_state',
                       }
    data_mapping = {'RecogniseObject': default_mapping,
                    'MoveToObject': default_mapping,
                    'GraspObject' :default_mapping,
                    "CheckSuccess": default_mapping,
                    "MoveGripperToParam": default_mapping}
    
    num_nodes = 20
    node_degrees = [1,2,2,0,2]
    nodes_params = [0,0,0,0,3]
    
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
    
if __name__=="__main__":
    test()    
        