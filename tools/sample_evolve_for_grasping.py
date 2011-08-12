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
from graph_evolve import graph_genome, smach_grasp
import smach
smach.set_loggers(smach_grasp.null_print,
          smach_grasp.null_print,
          smach_grasp.null_print,
          smach_grasp.null_print)
from graph_evolve.chromosome_smach import convert_chromosome_smach

import math
import numpy as np

#import graph_evolve

#from graph_evolve import graph_genome
import sys


from graph_evolve.smach_grasp import (names_mapping, classes_mapping, transitions_mapping,
                         data_mapping, node_degrees, nodes_params)

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
        return 0
    elif not sm.userdata.robot_state.object_in_gripper:
        return 20
    else:
        tot_dist = (smach_grasp.state_dist_gripper_object(sm.userdata.robot_state,
                                                          smach_grasp.target_pos) + 
                    smach_grasp.state_dist_gripper_rotation(sm.userdata.robot_state,
                                                            smach_grasp.target_rot)
                   )
        if outcome == "timeout":
            return 2. * tot_dist
        else:
            return tot_dist

def eval_func(chromosome, **args):
    return single_try

if __name__ == "__main__":
    
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
        (names_mapping,
         classes_mapping,
         transitions_mapping,         
         data_mapping,
         node_degrees,
         nodes_params) = all_mappings
        
        print "Generation: ", genome.generation
        print "Score: ", genome.score
        print "Genome: ", genome
        print "Len: ", len(genome)
        
        num_trials = 5000
        
        all_means = []
        all_stds = []
        n_points = 30
        for p in np.linspace(0.5, 1.0, n_points):
            smach_grasp.success_prob = p
            
            res = [single_try(genome) for _ in xrange(num_trials)]
            all_means.append(np.mean(res))
            all_stds.append(np.std(res))
        
        np.savez("grasping_samnples", 
                 prob = np.linspace(0.0, 1.0, n_points),
                 means = all_means,
                 stds = all_stds
                 )
        
        
        
        
        
        
        
