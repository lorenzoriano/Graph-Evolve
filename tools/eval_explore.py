#! /usr/bin/python
import sys
sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src')
sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src/pyevolve')

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
from graph_evolve import  smach_explore
import smach

def null_print(msg): pass
smach.set_loggers(null_print,
          null_print,
          null_print,
          null_print)

from graph_evolve.chromosome_smach import convert_chromosome_smach

import math

from matplotlib import pyplot as plt
import numpy as np

def eval_func(chromosome, **args):
    pass

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
        genome, factory, experiment = cPickle.load(f)
    
    ntrials = 5000
    succs = 0
    all_poses = []
    all_objects = []
#    smach_explore.max_transitions = 100
    for _ in xrange(ntrials):
        
#        world = factory.world
#        world.create_fixed_table( (3.5,-1.0) )
#        world.objects = []
#        world.create_object_on_table()
        
        factory = smach_explore.ExplorerFactory(experiment)
        sm = convert_chromosome_smach(genome, 
                                  factory.names_mapping, 
                                  factory.transitions_mapping, 
                                  factory.classes_mapping,
                                  factory.data_mapping)
        outcome = sm.execute()
        if outcome == "success":
            succs +=1
        all_poses.extend(factory.world.robot_positions)
        all_objects.append(factory.world.objects[0].pos)
    
    print "Successess: ", float(succs) / float(ntrials)
    
    all_poses = np.array(all_poses)
    all_objects = np.array(all_objects)
    plt.plot(all_objects[:,0], all_objects[:,1], 'r.',  alpha=0.1)
    plt.plot(all_poses[:,0], all_poses[:,1], 'b.', alpha=0.2)    
    plt.show()
    
    
        
        