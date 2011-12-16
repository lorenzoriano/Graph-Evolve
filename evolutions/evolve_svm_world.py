import sys
sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src')
sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src\\pyevolve')

sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src')
sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src/pyevolve')

from pyevolve import GSimpleGA
from pyevolve import Selectors
from pyevolve import Consts

from simulators import multi_svm 
sys.modules["multi_svm"] = multi_svm
from graph_evolve import graph_genome
import smach
from graph_evolve import  smach_svm_grasp

import cPickle
import random
import time
import os
import datetime
from graph_evolve import chromosome_smach

def null_print(msg): pass
smach.set_loggers(null_print,
                  null_print,
                  null_print,
                  null_print)


experiment = smach_svm_grasp.ExperimentSetup()

experiment.max_transitions = 15

#experiment.tableSVR_path = "/home/pezzotto/Logs/TryToPushAndActionConsequence/table_changesSVR.pkl"
#experiment.objectSVR_path = "/home/pezzotto/Logs/TryToPushAndActionConsequence/object_changesSVR.pkl"
#experiment.reachableSVC_path = "/home/pezzotto/Logs/TryToPushAndActionConsequence/reachableSVC.pkl"
#experiment.testingdata_path = "/home/pezzotto/Logs/TryToPushAndActionConsequence/given_fsm_2011-11-25-18-23-28.pkl"

experiment.pop_size = 200
experiment.stop_elitism = False
experiment.poolsize = 20
experiment.generations = 10000
experiment.elitism_size = 1
experiment.multiprocessing = False
experiment.num_nodes = 100

experiment.crossover_prob = 0.1
experiment.add_prob = 0.1
experiment.del_prob = 0.1
experiment.new_start_prob = 0.1
experiment.big_change_prob = 0.1
experiment.change_edge_prob = 0.1
experiment.mutate_par_sigma = 0.05
experiment.mutate_par_prob= 0.1

experiment.drifting_sigma = 0.01

experiment.initialize()
factory = smach_svm_grasp.SVMWorldFactory(experiment)

def single_try(genome, sm, inpt, world):
    obj = inpt[:3]
    table = inpt[3:]
    world.create_object(obj)
    world.create_table(table)
    sm.userdata.clear() 

    return sm.execute()

def eval_func(genome, **args):    
    factory = smach_svm_grasp.SVMWorldFactory(experiment)
    world = factory.world
    world.reset()
    sm = chromosome_smach.convert_chromosome_smach(genome, 
                                                  factory.names_mapping, 
                                                  factory.transitions_mapping, 
                                                  factory.classes_mapping, 
                                                  factory.data_mapping)

    fitness = 0.0
    for inpt, outp in zip(experiment.test_input, experiment.test_output):
        outcome = single_try(genome, sm, inpt, world)
        score = world.grasp_probability()
        if outcome == "timeout":
            score = score / 2

        if outp and (outcome != "success"):
            score = 0
        
        fitness += score
    
    return fitness

def print_nodes(ga_engine, wholepath):
    pop = ga_engine.getPopulation()
    file = open(wholepath, "w")
    elements = ("".join( str(n) for n in genome.nodes)
                for genome in pop)
    
    for e in sorted(elements, key=len):
        file.write(e+"\n")
    file.close()

def check_not_none(ga):
    for genome in ga.getPopulation():
        d = genome.__dict__
        for v in d.values():
            assert d is not None
        
        #for k,v in ((k,v) for (k,v) in d.iteritems() if k.find("prob")>0):
        #    print k, " --> ", v
        #print genome.mutate_par_sigma 
        #print genome.drifting_sigma
    elements = ("\n".join( str(n) for n in genome.nodes))
    print elements, "\n\n",


def stepCallback(ga_engine):
    generation = ga_engine.getCurrentGeneration()
    if generation % experiment.freq_stats == 0:
        best = ga_engine.bestIndividual()
        dirname = ga_engine.getParam("dirname", "")
        filename = "bestgenome_" + str(generation) + ".dat"
        wholepath = os.path.normpath(os.path.join(os.path.abspath(dirname),
                                     filename))
        file = open(wholepath,"wb")
        cPickle.dump((best, factory, experiment), 
                     file, 
                     protocol=cPickle.HIGHEST_PROTOCOL)
        file.close()

        ga_engine.printTimeElapsed()
        #check_not_none(ga_engine)
    
        #filename = "collection_types" + str(generation) + ".dat"
        #wholepath = os.path.normpath(os.path.join(os.path.abspath(dirname),
        #                             filename))
	#print_nodes(ga_engine, wholepath)

    return False

def main():
    random.seed(time.time())

    node_degrees = factory.node_degrees
    node_params = factory.node_params
    
    genome = graph_genome.GraphGenome(experiment.num_nodes, node_degrees, node_params)
    genome.evaluator.set(eval_func)
    genome.crossover_prob = experiment.crossover_prob
    genome.add_prob = experiment.add_prob
    genome.del_prob = experiment.del_prob
    genome.new_start_prob = experiment.new_start_prob 
    genome.big_change_prob = experiment.big_change_prob
    genome.change_edge_prob = experiment.change_edge_prob 
    genome.mutate_par_sigma = experiment.mutate_par_sigma
    genome.mutate_par_prob = experiment.mutate_par_prob

    genome.drifting_sigma = experiment.drifting_sigma
   
    ga = GSimpleGA.GSimpleGA(genome)
    
    if experiment.stop_elitism:
        ga.setElitism(False)
    else:
        ga.setElitism(True)
        ga.setElitismReplacement(experiment.elitism_size)
    
    ga.selector.set(Selectors.GRouletteWheel)
#    ga.setSortType(Consts.sortType["raw"])
    
    ga.setGenerations(experiment.generations)
    ga.setPopulationSize(experiment.pop_size)
    ga.setCrossoverRate(1.0)
    ga.setMutationRate(1.0)
    ga.getPopulation().setParams(tournamentPool = experiment.poolsize)

    ga.setMinimax(Consts.minimaxType["maximize"])
    ga.stepCallback.set(stepCallback)
    
    dirname = datetime.datetime.now().strftime("activity-%Y-%m-%d-%H-%M-%S/")
    os.mkdir(dirname)
    ga.setParams(dirname=dirname)
  
    if experiment.multiprocessing: 
        print "Using MultiProcessing"
        ga.setMultiProcessing(True, full_copy=False) 
    else:
        print "No multiprocessing"
    print "Initial num nodes: ", experiment.num_nodes
    print "Pop Size: ", experiment.pop_size
    print "Tournament Size: ", experiment.poolsize
    if not experiment.stop_elitism:
        print "Elitism Size: ", experiment.elitism_size
    else:
        print "NO ELITISM"
    print "Number of Generations: ", experiment.generations
        

    # Do the evolution
    ga.evolve(freq_stats = experiment.freq_stats)

    stepCallback(ga)
    
if __name__ == "__main__":
    main()
