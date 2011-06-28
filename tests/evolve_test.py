from pyevolve import GSimpleGA
from pyevolve import G1DList
from pyevolve import Selectors
from pyevolve import Initializators, Mutators,  Consts
from pyevolve import DBAdapters
from pyevolve import Crossovers

from graph_evolve import graph_genome
import networkx
import matplotlib.pyplot as plt
import math
import unittest
from structural_tests import sanity_checks

def follow_graph(chromosome, **args):
    
    max_steps = 20
    goal = 5.0
    
    node_id = 0
    pose = 0.0
    step = 0
    while step != max_steps:
        node = chromosome[node_id]
        pose = pose + (node.params[0] - 0.5)
        node_id = node.out_edges[0]        
        dist = math.fabs(pose - goal)
        if dist < 0.1:
            break
        step = step + 1    
    return dist
    

def eval_func(chromosome, **args):
    dict_genome = chromosome.create_dict()
    G = networkx.MultiDiGraph(dict_genome)

#    labels = {}
#    for i in xrange(chromosome.num_nodes):
#        labels[i] = str(chromosome.node_ids[i])
    centr = networkx.closeness_centrality(G)
    c_sum = sum(centr.values())
#    print "centr: ", centr, " c_sum: ", c_sum

    p_sum = sum ([node.params[0] for node in chromosome.nodes]) / len(G)
    return c_sum * p_sum
     

class TestEvolution(unittest.TestCase):

    def step_checks(self, ga_engine):
        for genome in ga_engine.getPopulation():
            sanity_checks(self, genome)
        

    def test_evolution(self):        
        out_degrees=[1]
        node_params = [1]
        genome = graph_genome.GraphGenome(5, out_degrees, node_params)
    #    genome.evaluator.set(eval_func)
        genome.evaluator.set(follow_graph)
        genome.setParams(p_del=1.0, p_add=0.1)
        
        ga = GSimpleGA.GSimpleGA(genome)
        print "STOPPING ELITISM"
        ga.setElitism(False)
        
        ga.selector.set(Selectors.GRouletteWheel)
        
        ga.setGenerations(50)
        ga.setPopulationSize(200)
        ga.setCrossoverRate(0.5)
        ga.setMutationRate(0.5)
    #    ga.setMinimax(Consts.minimaxType["maximize"])
        ga.setMinimax(Consts.minimaxType["minimize"])
        ga.stepCallback.set(self.step_checks)
            
        # Do the evolution
        ga.evolve(freq_stats = 10)
    
        # Best individual
        best = ga.bestIndividual()
        print "Best: ", best
    
        G = best.graph
    
    #    labels = best.create_labels()
    #    pos = networkx.graphviz_layout(G)
    #    networkx.draw_networkx_nodes(G, pos)
    #    networkx.draw_networkx_edges(G, pos)
    #    networkx.draw_networkx_labels(G, pos, labels)
        
        networkx.draw(G)
    
        plt.show()
        
    