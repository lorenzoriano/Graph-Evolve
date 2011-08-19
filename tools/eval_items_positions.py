#! /usr/bin/python
import sys
import grasping_world
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
    
    
    ntrials = 15000
    all_poses = []
#    smach_explore.max_transitions = 100
    for _ in xrange(ntrials):
        world = grasping_world.GraspingWorld()
        world.create_with_fixed_robot()
        all_poses.append(world.objects[0].pos)
    
    
    all_poses = np.array(all_poses)
    plt.plot(all_poses[:,0], all_poses[:,1], '.', alpha=0.1)
    plt.show()
    
    
        
        