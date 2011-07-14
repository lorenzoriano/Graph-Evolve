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
from graph_evolve import graph_genome
import smach
from graph_evolve import smach_grasp
smach.set_loggers(smach.loginfo,
          smach_grasp.null_print,
          smach_grasp.null_print,
          smach_grasp.null_print)
from graph_evolve.chromosome_smach import convert_chromosome_smach

#import graph_evolve

#from graph_evolve import graph_genome
import graph_genome

from graph_evolve import smach_grasp
import sys


from graph_evolve.chromosome_smach import convert_chromosome_pygraphviz


def eval_func(chromosome, **args):
    pass

if __name__ == "__main__":
    
    parser = OptionParser()
    parser.add_option("-i", "--input_file", dest="input_file", 
                      help="input FILE", 
                      metavar="FILE", action="store")
    parser.add_option("-o", "--output_file", dest="output_file", 
                      help="output FILE", metavar="FILE")
    
    (options, _) = parser.parse_args()
    
    if options.input_file is None:
        print "The input file has to be specified"
        sys.exit()
    if options.output_file is None:
        print "The output file has to be specified"
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
        
        sm = convert_chromosome_smach(genome, 
                                  names_mapping, 
                                  transitions_mapping, 
                                  classes_mapping,
                                  data_mapping)
        robot_state =  smach_grasp.RobotWorldState()
        sm.userdata.robot_state = robot_state
        outcome = sm.execute()
        
        print "STATE:"
        print sm.userdata.robot_state
        
        G = convert_chromosome_pygraphviz(genome, names_mapping, 
                                  transitions_mapping)
        G.write(options.output_file)
        
        
        
