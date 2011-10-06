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
#    return len(chromosome)
    return 1.0

def stepCallback(ga_engine):
    ga_engine.getPopulation()
    lens = [len(g) for g in ga_engine.getPopulation()]
    print "average len: ", sum(lens) / float(len(lens))

class CountNumberElements(object):


    def __init__(self):
        out_degrees=[2,2,1,2]
        node_params = [0,0,0,0]
        genome = graph_genome.GraphGenome(20, out_degrees, node_params)
        genome.evaluator.set(eval_func)
        genome.setParams(p_del=00, p_add=0.5)
        
        self.ga = GSimpleGA.GSimpleGA(genome)
        self.ga.setElitism(True)
        
        self.ga.selector.set(Selectors.GRouletteWheel)
        self.ga.getPopulation().setParams(tournamentPool = 200)
        
        self.ga.setGenerations(500)
        self.ga.setPopulationSize(200)
        self.ga.setCrossoverRate(0.1)
        self.ga.setMutationRate(0.1)
        self.ga.setMinimax(Consts.minimaxType["maximize"])
        self.ga.setMultiProcessing(False, False)
        self.ga.stepCallback.set(stepCallback)

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
    