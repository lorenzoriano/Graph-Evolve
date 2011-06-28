import sys

sys.path.append("/home/pezzotto/PythonStuff/graph-evolve/src")


from pyevolve import GSimpleGA
from pyevolve import G1DList
from pyevolve import Selectors
from pyevolve import Initializators, Mutators,  Consts
from pyevolve import DBAdapters
from pyevolve import Crossovers

import math
import random
import pickle

import pyrnn.libreaching
from pyrnn.libreaching import eval_func

from pyrnn import chromosome_convert
from pyrnn.libreaching import evaluate_out

short_version = False

def ga_eval_func(chromosome, **arg):
    net = chromosome_convert(chromosome)
    num_trials = 500
    num_steps = 20
    
    fitness = pyrnn.libreaching.eval_func(net, 
                                          num_trials, 
                                          num_steps,
                                          short_version)
    return fitness
    

def stepCallback(ga_engine):
    generation = ga_engine.getCurrentGeneration()
    if generation % 10 == 0:
        best = ga_engine.bestIndividual()
#        print best[:]
        net = chromosome_convert(best)
        #pickle_file = open("/home/pezzotto/tmp/best.txt", "w")
        #pickle.dump(net,  pickle_file, 0)
        #pickle_file.close()
        
        num_trials = 2000
        num_steps = 20
        
        print "Testing over ", num_trials, " trials"
        fitness = pyrnn.libreaching.eval_func(net, 
                                          num_trials, 
                                          num_steps,
                                          short_version)
        
        print "Score: ", fitness
        pyrnn.libreaching.evaluate_out(net, short_version)
        
    return False

def main():
    # Genome instance
    if short_version:
        input_size = 2
    else:
        input_size = 6
    hidden_size = 6
    output_size = 3
    bias_size = hidden_size + output_size
    total_size = input_size + hidden_size + output_size
    genome_size = (total_size - input_size)*total_size + bias_size
    
    genome = G1DList.G1DList(genome_size)
    genome.setParams(rangemin=-3, rangemax=3)
    genome.setParams(input_size=input_size,  hidden_size=hidden_size,  output_size=output_size)
    genome.setParams(gauss_mu=0., gauss_sigma=0.2)
    
    genome.initializator.set(Initializators.G1DListInitializatorReal)
    genome.mutator.set(Mutators.G1DListMutatorRealGaussian)
    genome.evaluator.set(ga_eval_func)
    genome.crossover.set(Crossovers.G1DListCrossoverTwoPoint)
    
    ga = GSimpleGA.GSimpleGA(genome)
    #ga.selector.set(Selectors.GRouletteWheel)
    ga.selector.set(Selectors.GRankSelector)
    ga.setGenerations(10)
    ga.setPopulationSize(100)
    ga.setCrossoverRate(0.0)
    ga.setMutationRate(0.2)
    ga.setMinimax(Consts.minimaxType["maximize"])
    
    ga.stepCallback.set(stepCallback)
    print "STOPPING ELITISM"
    ga.setElitism(False)
#    ga.setElitismReplacement(10)
    ga.setMultiProcessing(False, False)
    
    # Do the evolution
    ga.evolve(freq_stats = 10)
    
    # Best individual
    best = ga.bestIndividual()
    net = chromosome_convert(best)
    
    #pickle_file = open("/home/pezzotto/tmp/best.txt", "w")
    #pickle.dump(net,  pickle_file, 0)
    #pickle_file.close()
    
    print "Done"

if __name__ == "__main__":
    main()
    

