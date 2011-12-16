#! /usr/bin/python
import sys
sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src')
sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src/pyevolve')

import cPickle
from optparse import OptionParser

from graph_evolve import  smach_svm_grasp
import smach

def null_print(msg): pass
smach.set_loggers(null_print,
          null_print,
          null_print,
          null_print)

from graph_evolve.chromosome_smach import convert_chromosome_smach


def single_try(genome, sm, inpt, world):
    obj = inpt[:3]
    table = inpt[3:]
    world.reset()
    world.create_object(obj)
    world.create_table(table)
    sm.userdata.clear() 

    return sm.execute()

def calculate_score(outp, world):
    score = world.grasp_probability()
    if outcome == "timeout":
        score = score / 2

    if outp and (outcome != "success"):
        score = 0

    return score


def eval_func(chromosome, **args):
    pass

import numpy as np

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
    
    world = factory.world
    sm = convert_chromosome_smach(genome, 
                                  factory.names_mapping, 
                                  factory.transitions_mapping, 
                                  factory.classes_mapping, 
                                  factory.data_mapping)

    for inpt, outp in zip(experiment.test_input, experiment.test_output):
        print "Outp: ", outp
        outcome = single_try(genome, sm, inpt, world)
        print "outcome: ", outcome
        print "score: ", calculate_score(outp, world)
        print "Obj, table: ", np.hstack((world.obj_vec, world.table_vec))
        print "Netiput: ", world.net_input()

