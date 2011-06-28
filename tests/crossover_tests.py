import networkx as nx
#import graph_evolve
from graph_evolve import graph_genome
import random
import unittest

class TestCrossover(unittest.TestCase):

    def test_fixed(self):        
        nodes_degrees = [1, 1, 1, 1, 0, 0, 0, 0]
        nodes_params = [0] * len(nodes_degrees)
        num_node_types = len(nodes_degrees)
        
        num_nodes_1 = 4
        dad = graph_genome.GraphGenome(num_nodes_1, nodes_degrees, nodes_params)
        dad[0] = graph_genome.NodeGenome(0 ,dad, nodes_degrees,
                                                nodes_params)
        dad[1] = graph_genome.NodeGenome(1 ,dad, nodes_degrees,
                                                nodes_params)
        dad[2] = graph_genome.NodeGenome(2 ,dad, nodes_degrees,
                                                nodes_params)
        dad[3] = graph_genome.NodeGenome(3 ,dad, nodes_degrees,
                                                nodes_params)
        dad[0].out_edges[:] = [1]
        dad[1].out_edges[:] = [2]
        dad[2].out_edges[:] = [3]
        dad[3].out_edges[:] = [0]
        
        num_nodes_2 = 4
        mom = graph_genome.GraphGenome(num_nodes_2, nodes_degrees, nodes_params)
        mom[0] = graph_genome.NodeGenome(4 ,mom, nodes_degrees,
                                                nodes_params)
        mom[1] = graph_genome.NodeGenome(5 ,mom, nodes_degrees,
                                                nodes_params)
        mom[2] = graph_genome.NodeGenome(6 ,mom, nodes_degrees,
                                                nodes_params)
        mom[3] = graph_genome.NodeGenome(7 ,mom, nodes_degrees,
                                                nodes_params)
        mom[0].out_edges[:] = [2]
        mom[1].out_edges[:] = [3]
        mom[2].out_edges[:] = [0]
        mom[3].out_edges[:] = [1]
        
        args = {}
        args['mom'] = mom
        args['dad'] = dad
       
        sister, brother = graph_genome.graph_cross(None, **args)    
        print "Mom; ", mom
        print "Sister: ", sister
    
        print    
        print "Dad; ", dad
        print "Brother: ", brother
    
    def test_random(self):
        num_types = 80
        nodes_degrees = [3 for _ in xrange(num_types)]
        nodes_params = [0] * len(nodes_degrees)
        num_node_types = len(nodes_degrees)
        
        num_nodes_1 = random.randint(2, 20)
        for _ in xrange(50):            
            dad = graph_genome.GraphGenome(num_nodes_1, nodes_degrees, nodes_params)
            dad.initialize()
            
            num_nodes_2 = random.randint(2, 30)
            mom = graph_genome.GraphGenome(num_nodes_2, nodes_degrees, nodes_params)
            mom.initialize()
            
            args = {}
            args['mom'] = mom
            args['dad'] = dad
            
           
            sister, brother = graph_genome.graph_cross(None, **args)    

if __name__ == "__main__":
    unittest.main()
