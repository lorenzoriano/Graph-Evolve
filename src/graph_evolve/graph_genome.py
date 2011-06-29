import pyevolve.GenomeBase
import pyevolve.G1DList
import pyevolve.Mutators
import pyevolve.Initializators

import networkx as nx
from distutils import version

if version.LooseVersion(nx.__version__) < version.LooseVersion("1.5"):
    raise RuntimeError("Networkx should have at least version 1.5")

import random
import exceptions

class NodeRepresentation():
    '''
    An object instance of NodeRepresentation will be created on the fly by
    GraphGenome to compact information about a single node. It has no other
    use apart from being a representation
    
    '''
    def __init__(self):
        self.number = None
        self.type_id = None
        self.out_edges = {}
        self.params = None
        
    def __repr__(self):
        out_str = "[%d (id: %d) : %s" % (self.number,
                                         self.type_id,
                                         repr(self.out_edges), 
                                         )
        if len(self.params.genomeList):
            out_str += "(%s)] " % repr(self.params.genomeList)
        else:
            out_str += "] "
        return out_str

def get_bfs_neighbours(G, source, n):
    
    nodes = set()
    for edge in nx.bfs_edges(G.to_undirected(),source):
        nodes.add(edge[0])
        if len(nodes) >= n:
            break
        nodes.add(edge[1])
        if len(nodes) >= n:
            break
    return list(nodes), G.subgraph(nodes)

def graph_crossover(_, **args):

#    print "----------CROSSING--------------"

    mom = args["mom"]
    dad = args["dad"]
    
    sister = mom.clone()
    brother = dad.clone()
    G1 = sister.graph
    G2 = brother.graph    

    try:
        subnodes_G1 = args["subnodes_G1"]
        subnodes_G2 = args["subnodes_G2"]
        subgraph_G1 = G1.subgraph(subnodes_G1)
        subgraph_G2 = G2.subgraph(subnodes_G2)
        num_nodes = len(subnodes_G1)
    except KeyError:
        max_num_nodes = min(len(G1),len(G2)) / 2
        if max_num_nodes < 1:
            max_num_nodes = 1
        num_nodes = random.randint(1, max_num_nodes)
        
        source_G1 = random.sample(G1.nodes(), 1)[0]
        subnodes_G1, subgraph_G1 = get_bfs_neighbours(G1, source_G1, num_nodes)
        if len(subnodes_G1) != num_nodes:
            #Crossover not possible!
            return (sister, brother)
    
        source_G2 = random.sample(G2.nodes(), 1)[0]
        subnodes_G2, subgraph_G2 = get_bfs_neighbours(G2, source_G2, num_nodes)
        if len(subnodes_G2) != num_nodes:
            #Crossover not possible!
            return (sister, brother)

#    G1.remove_edges_from(subgraph_G1.edges())
    for node in subgraph_G1:
        G1.remove_edges_from(G1.out_edges(node))
#    G2.remove_edges_from(subgraph_G2.edges())
    for node in subgraph_G2:
        G2.remove_edges_from(G2.out_edges(node))

    for i in xrange(num_nodes):
        #swapping the node attributes
        G1.node[subnodes_G1[i]], G2.node[subnodes_G2[i]] = (
                        G2.node[subnodes_G2[i]], G1.node[subnodes_G1[i]])
        
        #re-wiring
        source = subnodes_G1[i]
        for e in subgraph_G2.out_edges_iter(subnodes_G2[i], data = True):            
            dest = subnodes_G1[subnodes_G2.index(e[1])]
            attr = e[2]            
            G1.add_edge(source, dest, attr_dict = attr)
        
        source = subnodes_G2[i]
        for e in subgraph_G1.out_edges_iter(subnodes_G1[i], data = True):            
            dest = subnodes_G2[subnodes_G1.index(e[1])]
            attr = e[2]
            G2.add_edge(source,dest, attr_dict = attr)
    
    #now fix the missing edges
    sister.fix_out_edges(subnodes_G1)
    brother.fix_out_edges(subnodes_G2)
    return (sister, brother)

def graph_mutator(genome, **args):
    '''
    Mutate the whole gene
    '''
    pmut = args["pmut"]
    p_add = genome.getParam("p_add", 0)
    p_del = genome.getParam("p_del", 0)
    
    mutated = 0
    #in-place mutations
    for node in genome.graph:
        if random.random() < pmut:
            mutated += 1
            genome.mutate_node(node, pmut)
            
    #adding a node
    if random.random() < p_add:
        genome.add_random_node()
    #removing a node
    if random.random() < p_del:
        genome.remove_random_node()
    
    #changing the starting node
    if (random.random() < pmut) or (genome.starting_node not in genome.graph):
        genome.starting_node = random.sample(genome.graph.nodes(), 1)[0]
    
    return mutated

def graph_initializer(genome, **_):
    for node in xrange(genome.initial_num_nodes):
            
        type_id = random.randint(0,  genome.num_node_types)
        num_params = genome.node_params[type_id]
        genome.graph.add_node(node, type_id = type_id, 
                              parameters=pyevolve.G1DList.G1DList(num_params))
        #initializing the properties of a node
        genome.setup_node(node)            
        
    #creating the edges
    for node in xrange(genome.initial_num_nodes):
        #every node has a fixed set of out_edges
        type_id = genome.graph.node[node]["type_id"]
        for n in xrange(genome.node_degrees[type_id]):
            destination = random.sample(genome.graph.nodes(), 1)[0]
            genome.graph.add_edge(node, destination, action_number=n)
            
    #setting up the starting node
    genome.starting_node = random.sample(genome.graph.nodes(), 1)[0]

class GraphGenome(pyevolve.GenomeBase.GenomeBase):
    
    def __init__(self,  num_nodes, node_degrees, node_params):
        """
        This is the genome representation of a graph. 
        
        @param num_nodes: the number of nodes in the graph
        @param node_degrees: a list that specifies the number of outgoing edges 
        for every node. node_degrees[i] = j means that node with type i has j 
        outgoing edges
        @param node_params: node_params[i]=j means that node i has j real-valued
        parameters. It needs to have the same length of node_degrees.
        @param p_add: the probability of adding a new node
        @param o_del: the probability of deleting a new node  
        """
        
        pyevolve.GenomeBase.GenomeBase.__init__(self)
        self.node_degrees = node_degrees
        self.node_params = node_params
        self.num_node_types = len(node_degrees) - 1
        self.initial_num_nodes = num_nodes        
        self.__graph = nx.MultiDiGraph()
        self.generation = 0
        self.starting_node = 0
        
        self.crossover.set(graph_crossover)
        self.mutator.set(graph_mutator)
        self.initializator.set(graph_initializer)

    def setup_node(self, node):
        '''
        Initialize the parameters list for a node
        @param node:
        '''
        params = self.graph.node[node]['parameters']
        params.mutator.set(pyevolve.Mutators.G1DListMutatorRealGaussian)
        params.initializator.set(pyevolve.Initializators.G1DListInitializatorReal)
        params.setParams(rangemin = 0, rangemax=1.0, 
                         gauss_mu = 0.0, gauss_sigma=0.05)
        params.initialize()
                
    def mutate_node(self, node, pmut):
        '''
        Mutate a single node
        @param node: the node to mutate
        @param pmut: the probability to mutate the whole node structure (type
        and out-edges
        '''
        
        #Changing a node id is very involved, so the probability of this
        #to happen is squared
        if random.random() < pmut:
            #The attributes change
            attrs = {}
            type_id = random.randint(0,  self.num_node_types)
            attrs['type_id'] = type_id
            num_params = self.node_params[type_id]
            attrs["parameters"] = pyevolve.G1DList.G1DList(num_params)
            self.graph.node[node] = attrs
            self.setup_node(node)
            
            #now remove and recreate all the edges
            self.graph.remove_edges_from(self.graph.out_edges_iter(node))
            for n in xrange(self.node_degrees[type_id]):
                destination = random.sample(self.graph.nodes(), 1)[0]
                self.graph.add_edge(node, destination, action_number=n)
        else:
            #change only a random edge:
            
            if self.graph.out_degree(node) == 0: #really nothing to change here
                return
            u,v,key,attrs = random.sample(self.graph.edges(
                                                    node,
                                                    data=True,
                                                    keys=True),1)[0]
            self.graph.remove_edge(u, v, key)
            destination = random.sample(self.graph.nodes(), 1)[0]
            self.graph.add_edge(node, destination, attr_dict = attrs)
        
            #mutate the parameters
            self.graph.node[node]["parameters"].mutate(pmut=pmut)
            
    
    def add_random_node(self):
        '''
        Add a random node, including its rewiring. The new node will have
        outgoing connections but not incoming ones
        '''
        
        #creating the node and the parameters
        
        node = len(self.graph)
        while node in self.graph:
            node += 1
        
        type_id = random.randint(0,  self.num_node_types)
        num_params = self.node_params[type_id]
        self.graph.add_node(node, type_id = type_id, 
                              parameters=pyevolve.G1DList.G1DList(num_params))
        #initializing the properties of a node
        self.setup_node(node)
        
        #adding the edges
        for n in xrange(self.node_degrees[type_id]):
            destination = random.sample(self.graph.nodes(), 1)[0]
            self.graph.add_edge(node, destination, action_number=n)
    
    def remove_random_node(self):
        '''
        Remove a random node. All the incident nodes will be rewired
        Node zero will never be deleted
        '''        
        
        if len(self.graph) <= 2:
            return
        node_to_delete = random.sample(self.graph.nodes()[1:], 1)[0]
        saved_edges = [edge 
                       for edge in self.graph.in_edges_iter(node_to_delete,
                                                              data=True)
                       if edge[0] != node_to_delete]
        
        self.graph.remove_node(node_to_delete)
        for edge in saved_edges:
            destination = random.sample(self.graph.nodes(), 1)[0]
            self.graph.add_edge(edge[0], destination, attr_dict=edge[2]) 
        
    
    def fix_out_edges(self, nodes):
        '''
        Fix a set of nodes so that each of them has the number of outgoing 
        edges specified in self.node_degrees
        @param nodes: an iterable of nodes to check
        '''
        
        for node in nodes:
            type_id = self.graph.node[node]["type_id"]
            num_outcomes = self.node_degrees[type_id]
            required_outcomes = set(xrange(num_outcomes))
            if self.graph.out_degree(node) > num_outcomes:
                print "\nBIG NUMBER: Node: ", node
                print "BIG NUMBER: Graph edges: ", self.graph.edges(data=True)
                raise exceptions.ValueError
            for out_edge in self.graph.out_edges_iter(node, data=True):
                #remove a found action
                try:
                    outcome = out_edge[2]["action_number"]
                    required_outcomes.remove(outcome)
                except KeyError:
                    print "KeyError: Graph edges: ", self.graph.out_edges(node, data=True)
                    raise
                
            #adding the missing outcomes
            for missing_outcome in required_outcomes:
                destination = random.sample(self.graph.nodes(), 1)[0]
                self.graph.add_edge(node, 
                                      destination, 
                                      action_number = missing_outcome)
        
    
    def __len__(self):
        return len(self.graph)
    
    def __getitem__(self, i):
        return self.get_node(i)
    
    def get_node(self, i):
        '''
        Returns a NodeRepresentation of the node indexed by i
        '''
        node = NodeRepresentation()
        node.number = i
        node.type_id = self.graph.node[i]["type_id"]
        node.params = self.graph.node[i]["parameters"]
        for (_, v, d) in self.graph.out_edges_iter(i, data=True):
            node.out_edges[ d['action_number'] ] = v
        return node
    
    def __repr__(self):
        out_str = ""
        for i in self.graph.nodes():
            if i == self.starting_node:
                out_str += "!!%s " % repr(self.get_node(i))
            else:
                out_str += "%s " % repr(self.get_node(i))
            
        return out_str
    
    @property
    def nodes(self):
        for node in self.graph.nodes_iter():
            yield self.get_node(node)
    
    def copy(self, genome):
        '''
        Copies self into genome.
        '''
        
        pyevolve.GenomeBase.GenomeBase.copy(self,genome)
        genome.node_degrees = self.node_degrees
        genome.node_params = self.node_params
        genome.num_node_types = self.num_node_types
        genome.graph = self.graph.copy()
        genome.generation = self.generation + 1
        
    def clone(self):
        '''
        Clones self into a new genome
        '''
#        if len(self) == 0:
#            self.initialize()
        newcopy = GraphGenome(self.initial_num_nodes,
                              self.node_degrees,
                              self.node_params)
        self.copy(newcopy)

        return newcopy
                
    def graph(self):
        return self.__graph
    def set_graph(self, g):
        self.__graph = g
        
    graph = property(graph, set_graph)
            