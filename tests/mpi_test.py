from pyevolve import GSimpleGA
from pyevolve import G1DList
from pyevolve import Selectors
from pyevolve import Initializators, Mutators,  Consts
from pyevolve import DBAdapters
from pyevolve import Crossovers

from pyevolve import mpi_migration

from graph_evolve import graph_genome
import networkx
import matplotlib.pyplot as plt
import math
import unittest

from mpi4py import MPI

def follow_graph(chromosome, **args):
    
    max_steps = 20
    goal = 10.0
    
    node_id = chromosome.starting_node
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

def test_evolution():        
    out_degrees=[1]
    node_params = [1]
    genome = graph_genome.GraphGenome(10, out_degrees, node_params)
#    genome.evaluator.set(eval_func)
    genome.evaluator.set(follow_graph)
    genome.setParams(p_del=0.1, p_add=0.1)
    
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
    
    comm = MPI.COMM_WORLD
    if comm.size > 1:
        migrator = mpi_migration.MPIMigrator()
        migrator.setGAEngine(ga)
        migrator.setNumReplacement(10)
        migrator.setMigrationRate(5)
        
        ga.setMigrationAdapter(migrator)    
    
    # Do the evolution
    if comm.rank == 0:
        ga.evolve(freq_stats = 10)
    else:
        ga.evolve(freq_stats = 0)

    if comm.rank == 0:
        # Best individual
        best = ga.bestIndividual()
        print "Best: ", best
    
        G = best.graph            
        networkx.draw(G)        
        plt.show()
 
if __name__ == "__main__":
    test_evolution() 
