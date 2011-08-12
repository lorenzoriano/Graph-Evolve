#! /usr/bin/python

import cPickle
from optparse import OptionParser

from pyevolve import GSimpleGA
from pyevolve import G1DList
from pyevolve import Selectors
from pyevolve import Initializators, Mutators,  Consts
from pyevolve import DBAdapters
from pyevolve import Crossovers
from pyevolve import Consts

import graph_evolve
from graph_evolve import graph_genome, smach_stack
import smach
smach.set_loggers(smach_stack.null_print,
          smach_stack.null_print,
          smach_stack.null_print,
          smach_stack.null_print)
from graph_evolve.chromosome_smach import convert_chromosome_smach

import math
import numpy as np

#import graph_evolve

#from graph_evolve import graph_genome
import sys


from graph_evolve.smach_stack import (names_mapping, classes_mapping, transitions_mapping,
                         data_mapping, node_degrees, nodes_params)

def single_try(chromosome, **args):
    sm = convert_chromosome_smach(chromosome, 
                                  names_mapping, 
                                  transitions_mapping, 
                                  classes_mapping,
                                  data_mapping)
    robot_state =  smach_stack.RobotWorldState()
    sm.userdata.robot_state = robot_state
    outcome = sm.execute()

    if outcome == "success":
#        return float( -len(chromosome))
        return 0.0   
    else:        
        if (not robot_state.obj1.initialized) or (not robot_state.obj2.initialized):
            return 50.0
        
        robot_state = sm.userdata.robot_state
        dist_now = smach_stack.dist_poses(robot_state.obj1.pos,
                                          robot_state.obj2.pos)
        
        dist_init = smach_stack.dist_poses(robot_state.obj1.starting_pos,
                                          robot_state.obj2.starting_pos)
        
        if dist_now == dist_init: #the object didn't move
            return 25.0
        
        tot_dist = dist_now
        if outcome == "timeout":
            return 2. * tot_dist
        else:
            return tot_dist

def test(genome,
         names_mapping, 
         transitions_mapping, 
         classes_mapping,
         data_mapping):
    
    sm = convert_chromosome_smach(genome, 
                                  names_mapping, 
                                  transitions_mapping, 
                                  classes_mapping,
                                  data_mapping)
    robot_state =  smach_stack.RobotWorldState()
    sm.userdata.robot_state = robot_state
    outcome = sm.execute()
    

def eval_func(chromosome, **args):
    return single_try(chromosome, **args)

def main():
    parser = OptionParser()
    parser.add_option("-i", "--input_file", dest="input_file", 
                      help="input FILE", 
                      metavar="FILE", action="store")
    
    (options, _) = parser.parse_args()
    
    if options.input_file is None:
        print "The input file has to be specified"
        sys.exit()
        
    with open(options.input_file, "r") as f:
        (genome, all_mappings) = cPickle.load(f)
#        (names_mapping,
#         classes_mapping,
#         transitions_mapping,         
#         data_mapping,
#         node_degrees,
#         nodes_params) = all_mappings
        
        print "Generation: ", genome.generation
        print "Score: ", genome.score
        print "Genome: ", genome
        print "Len: ", len(genome)
        
#        test(genome, names_mapping, 
#                                  transitions_mapping, 
#                                  classes_mapping,
#                                  data_mapping)
#        sys.exit()

        sm = convert_chromosome_smach(genome, 
                                  names_mapping, 
                                  transitions_mapping, 
                                  classes_mapping,
                                  data_mapping)
        robot_state =  smach_stack.RobotWorldState()
        sm.userdata.robot_state = robot_state
        
        print "\nInitPoses: ", robot_state.obj1.starting_pos,"\t",robot_state.obj2.starting_pos
        print "Poses: ", robot_state.obj1.pos,"\t",robot_state.obj2.pos
        
        outcome = sm.execute()
        
        robot_state = sm.userdata.robot_state
        
        print "\nInitPoses: ", robot_state.obj1.starting_pos,"\t",robot_state.obj2.starting_pos
        print "Poses: ", robot_state.obj1.pos,"\t",robot_state.obj2.pos
        
        num_trials = 3000         

        all_means = []
        all_stds = []
        n_points = 30
        for p in np.linspace(0.0, 1.0, n_points):
            smach_stack.success_prob = p
            
            res = [single_try(genome) for _ in xrange(num_trials)]
            all_means.append(np.mean(res))
            all_stds.append(np.std(res))
        
        np.savez("stacking_samnples",
                 prob = np.linspace(0.0, 1.0, n_points),
                 means = all_means,
                 stds = all_stds
                 )
        
        
if __name__ == "__main__":
    main()
        
        
        
