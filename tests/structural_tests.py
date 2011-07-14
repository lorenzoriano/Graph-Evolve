import networkx as nx
import pyevolve.G1DList
from graph_evolve import graph_genome
import random
import unittest
import cPickle

def sanity_checks(tester, genome):
    sanity_check_out_edges(tester, genome)
    sanity_check_node_parameters(tester, genome)
    sanity_check_number_nodes(tester, genome)
    sanity_check_node_zero(tester, genome)
        
def sanity_check_out_edges(tester, genome):
    '''
    Check that every node has the right number of out_edges and with the
    right action number
    @param tester:
    @param genome:
    '''
    nodes_degrees = genome.node_degrees
    
    for node, n_dict in genome.graph.nodes_iter(data=True):
        edges_pool = set()
        type_id = n_dict["type_id"]
        required_out_degree = nodes_degrees[type_id]
        tester.assertEqual(required_out_degree, genome.graph.out_degree(node))
        for _, _ , e_dict in genome.graph.out_edges_iter(node, data=True):
            
            edges_pool.add( e_dict["action_number"] )
        tester.assertEqual(len(edges_pool), required_out_degree)

def sanity_check_number_nodes(tester, genome):
    '''
    Check that every graph has at least 2 nodes
    @param tester:
    @param genome:
    '''
    tester.assertTrue( len(genome.graph) >= 2)

def sanity_check_node_parameters(tester, genome):
    '''
    Check that every node has the right number of parameters
    @param tester:
    @param genome:
    '''
    node_params = genome.node_params
    
    for node, n_dict in genome.graph.nodes_iter(data=True):        
        params = n_dict["parameters"]
        tester.assertTrue(isinstance(params,pyevolve.G1DList.G1DList))
        type_id = n_dict["type_id"]
        tester.assertEqual(len(params), node_params[type_id])

def sanity_check_node_zero(tester, genome):
    '''
    Starting node
    @param tester:
    @param genome:
    '''
    
    node = genome[genome.starting_node]

def sanity_check_all_connected(tester, genome):
    
    tree = nx.dfs_tree(genome.graph, source=genome.starting_node)
    
    
    if len(tree) > 2:
        tester.assertEqual(len(tree), len(genome.graph))
    
    

class FunctionalTests(unittest.TestCase):
    def setUp(self):
        random.seed()
        self.num_types = 10
        self.nodes_degrees = [5 for _ in xrange(self.num_types)]
        self.nodes_params = [0] * len(self.nodes_degrees)
        self.num_node_types = len(self.nodes_degrees)
    
    def test_iterator(self):
        num_nodes = 100
        genome = graph_genome.GraphGenome(num_nodes, 
                                  self.nodes_degrees, 
                                  self.nodes_params)

        genome.initialize()
        all_nodes = [node for node in genome.nodes]
        self.assertEqual(len(all_nodes), len(genome))
        sanity_checks(self, genome)

    def test_puning(self):
        for _ in xrange(100):
            num_nodes = 100
            genome = graph_genome.GraphGenome(num_nodes, 
                                      self.nodes_degrees, 
                                      self.nodes_params)
    
            genome.initialize()
            genome.prune_non_connected()
            sanity_checks(self, genome)
            sanity_check_all_connected(self, genome)
    
    def test_pickling(self):
        num_nodes = 100
        genome = graph_genome.GraphGenome(num_nodes, 
                                      self.nodes_degrees, 
                                      self.nodes_params)
        genome.initialize()
        state = cPickle.dumps(genome)
        newgenome = cPickle.loads(state)
        self.assertEqual(len(genome), len(newgenome))

class TestMutation(unittest.TestCase):
    def setUp(self):
        random.seed()
        self.num_types = 10
        self.nodes_degrees = [5 for _ in xrange(self.num_types)]
        self.nodes_params = [0] * len(self.nodes_degrees)
        self.num_node_types = len(self.nodes_degrees)
        
    def test_single_add_node(self):
        num_nodes = 10
        genome = graph_genome.GraphGenome(num_nodes, 
                                          self.nodes_degrees, 
                                          self.nodes_params)
        
        genome.initialize()
        length = len(genome)  
        sanity_checks(self, genome)
        genome.add_random_node()
        
        self.assertEqual(len(genome), length + 1)
        sanity_checks(self, genome)
        
    def test_single_remove_node(self):
        num_nodes = 20
        genome = graph_genome.GraphGenome(num_nodes, 
                                          self.nodes_degrees, 
                                          self.nodes_params)
        genome.initialize()
        sanity_checks(self, genome)
        
        genome.setParams(p_del=0.5, p_add=0.5)
        length = len(genome)
        genome.remove_random_node()        
        
        self.assertEqual(len(genome), length - 1)
        sanity_checks(self, genome)
    
    def mutation_test(self):
        num_nodes = 20
        genome = graph_genome.GraphGenome(num_nodes, 
                                          self.nodes_degrees, 
                                          self.nodes_params)
        genome.initialize()
        sanity_checks(self, genome)
        genome.setParams(p_del=0.5, p_add=0.5)
        
        for _ in xrange(100):
            genome.mutate(pmut=0.2)
            sanity_checks(self, genome)
            sanity_check_all_connected(self, genome)
        
    def test_adding_removing(self):
        num_nodes = 20
        genome = graph_genome.GraphGenome(num_nodes, 
                                          self.nodes_degrees, 
                                          self.nodes_params)
        genome.initialize()
        sanity_checks(self, genome)
        genome.setParams(p_del=0.5, p_add=0.5)
        
        for _ in xrange(100):
            
            length = len(genome)
            genome.remove_random_node()
            self.assertEqual(len(genome), length - 1)
            sanity_checks(self, genome)
            
            length = len(genome)
            genome.add_random_node()
            self.assertEqual(len(genome), length + 1)
            sanity_checks(self, genome)            
            
        
class TestCrossover(unittest.TestCase):
    def setUp(self):
        random.seed()
        self.num_types = 1
        self.nodes_degrees = [5 for _ in xrange(self.num_types)]
        self.nodes_params = [0] * len(self.nodes_degrees)
        self.num_node_types = len(self.nodes_degrees)
    
    def test_crossover(self):
        num_nodes = 15
        dad = graph_genome.GraphGenome(num_nodes, 
                                          self.nodes_degrees, 
                                          self.nodes_params                                          
                                          )
        mom = graph_genome.GraphGenome(num_nodes, 
                                          self.nodes_degrees, 
                                          self.nodes_params
                                          )
        
        mom.setParams(p_del=0.0, p_add=0.0)
        dad.setParams(p_del=0.0, p_add=0.0)
        mom.initialize()
        dad.initialize()
        sanity_checks(self, mom)
        sanity_checks(self, dad)
        
        for step in xrange(100):
            mom, dad = graph_genome.graph_crossover(None, mom=mom, dad=dad)
            sanity_checks(self, mom)
            sanity_checks(self, dad)
    
    def test_mutation_crossover(self):
        num_nodes_dad = random.randint(5,100)
        num_nodes_mom = random.randint(5,100)
        
        num_nodes_types = random.randint(1, 100)
        nodes_degrees = [random.randint(0,50) for _ in xrange(num_nodes_types)]
        nodes_params = [random.randint(0,50) for _ in xrange(num_nodes_types)]
        
        dad = graph_genome.GraphGenome(num_nodes_dad, 
                                       nodes_degrees, 
                                       nodes_params                                          
                                       )
        mom = graph_genome.GraphGenome(num_nodes_mom, 
                                       nodes_degrees, 
                                       nodes_params
                                       )
        
        mom.setParams(p_del=0.5, p_add=0.5)
        dad.setParams(p_del=0.5, p_add=0.5)
        mom.initialize()
        dad.initialize()
        sanity_checks(self, mom)
        sanity_checks(self, dad)
        
        for step in xrange(300):
            sister, brother = graph_genome.graph_crossover(None, mom=mom, dad=dad)
            sanity_checks(self, sister)
            sanity_checks(self, brother)
            
            sanity_check_all_connected(self, sister)
            sanity_check_all_connected(self, brother)
                                
            sister.mutate(pmut=0.5, ga_engine = None)
            brother.mutate(pmut=0.5, ga_engine = None)
            
            sanity_checks(self, sister)
            sanity_checks(self, brother)
            
            sanity_check_all_connected(self, sister)
            sanity_check_all_connected(self, brother)
            
            dad = brother
            mom = sister
            
    def test_fixed(self):
        G1 = nx.MultiDiGraph()        
        G1.add_edge(0, 0, action_number=0)
        G1.add_edge(0, 1, action_number=1)
        G1.add_edge(1, 2, action_number=0)
        G1.add_edge(2, 1, action_number=0)
        G1.add_edge(2, 0, action_number=1)
        G1.add_edge(2, 5, action_number=2)
        
        for node in G1:
            G1.node[node]['type_id'] = node
            G1.node[node]['parameters'] = pyevolve.G1DList.G1DList(0)
        
        G2 = nx.MultiDiGraph()
        G2.add_edge(0, 1, action_number=0)
        G2.add_edge(0, 3, action_number=1)
        G2.add_edge(3, 4, action_number=0)
        G2.add_edge(1, 4, action_number=0)
        for node in G2:
            G2.node[node]['type_id'] = node
            G2.node[node]['parameters'] = pyevolve.G1DList.G1DList(0)
        
        nodes_degrees = [2, 1, 3, 1, 0, 0]
        nodes_params = [0] * len(nodes_degrees)
        
        genome1 = graph_genome.GraphGenome(len(G1), 
                                          nodes_degrees, 
                                          nodes_params
                                          )
        genome1.graph = G1
        genome2 = graph_genome.GraphGenome(len(G2), 
                                          nodes_degrees, 
                                          nodes_params
                                          )
        genome2.graph = G2
        
        genome1.prune_non_connected()
        self.assertEqual(len(genome1), 4)
        self.assertEqual(len(genome2), 4)
        
        sanity_checks(self, genome1)
        sanity_checks(self, genome2)
        
        bogus = str(genome1)
        bogus = str(genome2)

        genome1, genome2 =  graph_genome.graph_crossover(None, 
                                                         mom=genome1, 
                                                         dad=genome2)
        sanity_checks(self, genome1)
        sanity_checks(self, genome2)
        