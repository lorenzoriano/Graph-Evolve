import sys
sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src')
sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src')

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
from graph_evolve import smach_stack
from graph_evolve.chromosome_smach import convert_chromosome_smach

import cPickle
from mpi4py import MPI
from pyevolve import mpi_migration
import math
import random
import time
import os
import datetime

smach.set_loggers(smach.loginfo,
                  smach_stack.null_print,
                  smach_stack.null_print,
                  smach_stack.null_print)

from graph_evolve.smach_stack import (names_mapping, classes_mapping, transitions_mapping,
                         data_mapping, node_degrees, nodes_params)

all_mappings = (names_mapping,
                classes_mapping,
                transitions_mapping,                
                data_mapping,
                node_degrees,
                nodes_params)

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

def eval_func(chromosome, **args):
    num_trials = 1
    return sum(single_try(chromosome, **args) for _ in xrange(num_trials) ) / num_trials

freq_stats = 1

def stepCallback(ga_engine):
    generation = ga_engine.getCurrentGeneration()
    if generation % freq_stats == 0:
        
        comm = MPI.COMM_WORLD
        if comm.rank == 0:
            migrator = ga_engine.migrationAdapter
            if migrator.all_stars is not None:                
                print "All stars: ", [i.score for i in migrator.all_stars]                
                if ga_engine.getMinimax() == Consts.minimaxType["maximize"]:                
                    best = max(migrator.all_stars, key = lambda x:x.score)
                else:
                    best = min(migrator.all_stars, key = lambda x:x.score)
                    
                dirname = ga_engine.getParam("dirname", "")
                wholepath = os.path.normpath(os.path.join(os.path.abspath(dirname),                                             
                                             "bestgenome.dat")
                                            )
                file = open(wholepath,"wb")
                cPickle.dump((best, all_mappings), 
                             file, 
                             protocol=cPickle.HIGHEST_PROTOCOL)
                file.close()
                smach.set_loggers(smach.loginfo,
                        smach.logwarn,
                        smach_stack.null_print,
                        smach.logerr)
#                score = eval_func(best)
                newbest = best.clone()
                newbest.evaluate()
                score = newbest.score
                print "Fitness: ", best.score
                print "Real Score: ", score
                print
                ga_engine.printTimeElapsed()
        
        
    return False

if __name__ == "__main__":

    comm = MPI.COMM_WORLD
    random.seed(time.time() * comm.rank)
    
    smach.set_loggers(smach_stack.null_print,
                      smach.logwarn,
                      smach_stack.null_print,
                      smach.logerr)
    num_nodes= 50
    pop_size = 300
    poolsize = int(pop_size / 10.)
    migration_size = 10
    migration_rate = 10
    elitism_size = 1
    generations = 5000
    stop_elitism = True    
    
    genome = graph_genome.GraphGenome(num_nodes, node_degrees, nodes_params)
    genome.evaluator.set(eval_func)
    
    ga = GSimpleGA.GSimpleGA(genome)
    
    if stop_elitism:
        ga.setElitism(False)
    else:
        ga.setElitism(True)
        ga.setElitismReplacement(elitism_size)
    
    ga.selector.set(Selectors.GRouletteWheel)
#    ga.setSortType(Consts.sortType["raw"])
    
    ga.setGenerations(generations)
    ga.setPopulationSize(pop_size)
    ga.setCrossoverRate(0.1)
    ga.setMutationRate(0.1)
    ga.getPopulation().setParams(tournamentPool = poolsize)
    genome.setParams(p_del=0.01, p_add=0.2)

    ga.setMinimax(Consts.minimaxType["minimize"])

    if comm.size > 1:
        migrator = mpi_migration.MPIMigrator()
        migrator.setGAEngine(ga)
        migrator.setNumReplacement(migration_size)
        migrator.setMigrationRate(migration_rate)
        migrator.selector.set(Selectors.GRankSelector)
#        migrator.selector.set(Selectors.GRouletteWheel)

        ga.setMigrationAdapter(migrator)
    if comm.rank == 0:
        ga.stepCallback.set(stepCallback)
    
    if comm.rank == 0:
        dirname = datetime.datetime.now().strftime("activity-%Y-%m-%d-%H-%M-%S/")
        os.mkdir(dirname)
        ga.setParams(dirname=dirname)
        
        print "Initial num nodes: ", num_nodes
        print "Pop Size: ", pop_size
        print "Tournament Size: ", poolsize
        print "Migration Size: ", migration_size
        print "Migration frequency: ", migration_rate
        if not stop_elitism:
            print "Elitism Size: ", elitism_size
        else:
            print "NO ELITISM"
        print "Number of Generations: ", generations

    # Do the evolution
    if comm.rank == 0:
        ga.evolve(freq_stats = freq_stats)
    else:
        ga.evolve(freq_stats = 0)

    if comm.rank == 0:
        stepCallback(ga)
    
    
    