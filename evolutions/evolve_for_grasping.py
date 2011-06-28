from pyevolve import GSimpleGA
from pyevolve import G1DList
from pyevolve import Selectors
from pyevolve import Initializators, Mutators,  Consts
from pyevolve import DBAdapters
from pyevolve import Crossovers

import graph_evolve
from graph_evolve import graph_genome
import smach
from graph_evolve import smach_grasp
from graph_evolve.chromosome_smach import convert_chromosome_smach
from graph_evolve.chromosome_smach import convert_chromosome_pygraphviz

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
 
node_degrees = [1,2,2,0,2]
nodes_params = [0,0,0,0,3]

def single_try(chromosome, **args):
    sm = convert_chromosome_smach(chromosome, 
                                  names_mapping, 
                                  transitions_mapping, 
                                  classes_mapping,
                                  data_mapping)
    robot_state =  smach_grasp.RobotWorldState()
    sm.userdata.robot_state = robot_state
    outcome = sm.execute()
    
    if outcome == "success":
#        print "Great!"
        return 0
    else:
#        dist = smach_grasp.state_dist_gripper_object(sm.userdata.robot_state)
#        return dist
        if sm.userdata.robot_state.object_in_gripper:
            return smach_grasp.state_dist_gripper_object(sm.userdata.robot_state,
                                                         (0.0,0.0,0.0))
        else:
            return 25

def eval_func(chromosome, **args):
    res = []
    for _ in xrange(10): 
        res.append(single_try(chromosome, **args))
    return sum(res)


def stepCallback(ga_engine):
    generation = ga_engine.getCurrentGeneration()
    if generation % 10 == 0:
        best = ga.bestIndividual()
        G = convert_chromosome_pygraphviz(best, names_mapping, 
                                          transitions_mapping)
        G.write("/home/pezzotto/tmp/tmp.dot")
        smach.set_loggers(smach.loginfo,
                  smach.logwarn,
                  smach_grasp.null_print,
                  smach.logerr)
        score = eval_func(best)
        print "Score: ", score
        
        
    return False

if __name__ == "__main__":
    
    smach.set_loggers(smach_grasp.null_print,
                      smach.logwarn,
                      smach_grasp.null_print,
                      smach.logerr)
    num_nodes= 8    
    genome = graph_genome.GraphGenome(num_nodes, node_degrees, nodes_params)
    genome.evaluator.set(eval_func)
    
    ga = GSimpleGA.GSimpleGA(genome)
#    print "STOPPING ELITISM"
    ga.setElitism(True)
    ga.setElitismReplacement(1)
    
    ga.selector.set(Selectors.GRouletteWheel)
    ga.stepCallback.set(stepCallback)
    
    ga.setGenerations(10)
    ga.setPopulationSize(10)
    ga.setCrossoverRate(0.2)
    ga.setMutationRate(0.2)
#    ga.setMinimax(Consts.minimaxType["maximize"])
    ga.setMinimax(Consts.minimaxType["minimize"])

    # Do the evolution
    ga.evolve(freq_stats = 10)

    # Best individual
    best = ga.bestIndividual()
    G = convert_chromosome_pygraphviz(best, names_mapping, 
                                      transitions_mapping)
    G.write("/home/pezzotto/tmp/tmp.dot")
    
    sm = convert_chromosome_smach(best, 
                                  names_mapping, 
                                  transitions_mapping, 
                                  classes_mapping,
                                  data_mapping)
    robot_state =  smach_grasp.RobotWorldState()
    sm.userdata.robot_state = robot_state
    
    smach.set_loggers(smach.loginfo,
                      smach.logwarn,
                      smach_grasp.null_print,
                      smach.logerr)
    try:
        outcome = sm.execute()
    except smach.exceptions.InvalidUserCodeError as err:
        print "Error: ", err.message
    else:
        print "Outcome: ", outcome
    print "Distance is: ", smach_grasp.state_dist_gripper_object(sm.userdata.robot_state,
                                                                  (0.0,5.0,0.0))
    
    print "eval is: ", eval_func(best)
    
    
    