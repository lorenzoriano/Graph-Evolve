import sys
sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src')
sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src\\pyevolve')

sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src')
sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src/pyevolve')

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
from graph_evolve import smach_explore
from graph_evolve.chromosome_smach import convert_chromosome_smach

import cPickle
from mpi4py import MPI
from pyevolve import MpiMigration
import math
import random
import time
import os
import datetime

def null_print(msg): pass
smach.set_loggers(null_print,
                  null_print,
                  null_print,
                  null_print)


def single_try(chromosome, **args):
    
    factory = smach_explore.ExplorerFactory()
    world = factory.world
    
    dist_init = world.robot.dist(world.objects[0]) 
    
    
    sm = convert_chromosome_smach(chromosome, 
                                  factory.names_mapping, 
                                  factory.transitions_mapping, 
                                  factory.classes_mapping,
                                  factory.data_mapping)
    try:
        outcome = sm.execute()
    except smach.InvalidUserCodeError, e:
        print "\nERROR: ", e.message
        raise

    if outcome == "success":
        return 1.0
    else:
        return 0.0
        


def eval_func(chromosome, **args):
    return sum(single_try(chromosome, **args) for _ in xrange(num_trials) ) / num_trials

freq_stats = 10

def stepCallback(ga_engine):
    generation = ga_engine.getCurrentGeneration()
    if generation % freq_stats == 0:
        
        comm = MPI.COMM_WORLD
        if comm.rank == 0:
            migrator = ga_engine.migrationAdapter
            if migrator is not None and migrator.all_stars is not None:                
                print "All stars: ", [i.score for i in migrator.all_stars]                
                if ga_engine.getMinimax() == Consts.minimaxType["maximize"]:                
                    best = max(migrator.all_stars, key = lambda x:x.score)
                else:
                    best = min(migrator.all_stars, key = lambda x:x.score)
                    
                dirname = ga_engine.getParam("dirname", "")
                filename = "bestgenome_" + str(generation) + ".dat"
                wholepath = os.path.normpath(os.path.join(os.path.abspath(dirname),                                             
                                             filename)
                                            )
                file = open(wholepath,"wb")
                cPickle.dump((best, names_mapping, transitions_mapping), 
                             file, 
                             protocol=cPickle.HIGHEST_PROTOCOL)
                file.close()

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
    

    num_nodes= 100
    pop_size = 10
#    poolsize = int(pop_size / 10.)
    poolsize = pop_size
    migration_size = 1
    migration_rate = 1
    elitism_size = 1
    generations = 5000
    num_trials = 300
    stop_elitism = True
    
    factory = smach_explore.ExplorerFactory()
    node_degrees = factory.node_degrees
    node_params = factory.node_params
    names_mapping = factory.names_mapping
    transitions_mapping = factory.transitions_mapping
    
    genome = graph_genome.GraphGenome(num_nodes, node_degrees, node_params)
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
    genome.setParams(p_del=0.01, p_add=0.3)

    ga.setMinimax(Consts.minimaxType["maximize"])

    if comm.size > 1:
        migrator = MpiMigration.MPIMigrator()
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
        print "Number of island: ", comm.size
        print "Tot number of individuals: ", pop_size * comm.size
        print "Tournament Size: ", poolsize
        print "Migration Size: ", migration_size
        print "Migration frequency: ", migration_rate
        if not stop_elitism:
            print "Elitism Size: ", elitism_size
        else:
            print "NO ELITISM"
        print "Number of Generations: ", generations
        print "Number of trials: ", num_trials
        

    # Do the evolution
    if comm.rank == 0:
        ga.evolve(freq_stats = freq_stats)
    else:
        ga.evolve(freq_stats = 0)

    if comm.rank == 0:
        stepCallback(ga)
    
    
    