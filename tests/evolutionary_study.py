from pyevolve import GSimpleGA
from pyevolve import G1DList
from pyevolve import Selectors
from pyevolve import Initializators, Mutators,  Consts
from pyevolve import DBAdapters
from pyevolve import Crossovers

from graph_evolve import graph_genome
import networkx
import matplotlib.pylab as pylab
import math
import random
from structural_tests import sanity_checks


def eval_func(chromosome, **args):
    return len(chromosome)
#    return 1.0

class CountNumberElements(object):


    def __init__(self):
        out_degrees=[1,2,0]
        node_params = [0,0,0]
        genome = graph_genome.GraphGenome(20, out_degrees, node_params)
        genome.evaluator.set(eval_func)
        genome.setParams(p_del=0.1, p_add=0.5)
        
        self.ga = GSimpleGA.GSimpleGA(genome)
        self.ga.setElitism(False)
        
        self.ga.selector.set(Selectors.GRouletteWheel)
        self.ga.getPopulation().setParams(tournamentPool = 200)
        
        self.ga.setGenerations(50)
        self.ga.setPopulationSize(200)
        self.ga.setCrossoverRate(0.1)
        self.ga.setMutationRate(0.1)
        self.ga.setMinimax(Consts.minimaxType["maximize"])
        self.ga.setMultiProcessing(False, False)

    def run(self):
        self.ga.initialize()
        prevalues = [len(genome) for genome in self.ga.getPopulation()]
        pylab.hist(prevalues, bins = len(prevalues))
        pylab.title("Before evolution")
        
        # Do the evolution        
        self.ga.evolve(freq_stats = 10)
        
        values = [len(genome) for genome in self.ga.getPopulation()]
        
        pylab.figure()
        pylab.hist(values, bins = len(values))
        pylab.title("After evolution")
        pylab.show()        
            
    

if __name__ == "__main__":
    test = CountNumberElements()
    test.run()
    