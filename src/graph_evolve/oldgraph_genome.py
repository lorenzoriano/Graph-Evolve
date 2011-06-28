import pyevolve.GenomeBase
import pyevolve.G1DList
import pyevolve.Mutators
import pyevolve.Initializators

import random
import exceptions

def edges_initializator(genome, **args):
    range_min = genome.getParam("rangemin")
    range_max = genome.getParam("rangemax")
    graph_genome = genome.getParam("graph_genome")
    
    genome.genomeList =[]
    num_trials = 0 
    while len(genome.genomeList) != genome.getListSize():
        
        if num_trials > 1000:
            raise exceptions.RuntimeError("too many iterations!")
        num_trials += 1
        
        edge_to_node = random.randint(range_min, range_max)        
        if ( (graph_genome[edge_to_node] is None) or 
             (not graph_genome[edge_to_node].disabled)):
            genome.genomeList.append(edge_to_node)

def edges_mutator(genome, **args):

    pmut = args["pmut"]
    range_min = genome.getParam("rangemin")
    range_max = genome.getParam("rangemax")
    graph_genome = genome.getParam("graph_genome")

    if pmut <= 0.0: 
        return 0
    
    mutations = 0
    for i in xrange(len(genome)):
        if random.random() < pmut:
            new_dest = random.randint(range_min, range_max)
            while graph_genome[new_dest].disabled:
                new_dest = random.randint(range_min, range_max)
            genome[i] = new_dest
            mutations += 1
    
    return mutations




class NodeGenome(object):
    def __init__(self, id_, graph_genome, nodes_degrees, nodes_params,):
        '''
        
        @param id: The id of the node
        @param node_degrees: The number of outgoing edges for every node.
        node_degrees[i] = j means that node with id i has j outgoing edges
        @param nodes_params: The number of parameters for every node.
        node_params[i] = j means that node with id i has j parameters
        @param num_node_types: the number of different node types        
        '''
        self.id = id_
        self.node_degrees = nodes_degrees
        self.nodes_params = nodes_params
        self.num_node_types = len(nodes_degrees) - 1
        self.graph_genome = graph_genome 
        self.params = self.create_params(nodes_params[id_])
        self.out_edges = self.create_edges(nodes_degrees[id_])
        self.disabled = False
    
    def initialize(self):
        self.out_edges.initialize()
        self.params.initialize()
    
    def randomize(self):
        self.params = self.create_params(self.nodes_params[self.id])
        self.out_edges = self.create_edges(self.node_degrees[self.id])
        self.initialize()
    
    def mutate(self, pmut):
        if random.random() < pmut:
            #new node id, generate a new set of outgoing edges
            newid = random.randint(0, self.num_node_types)            
            new_out_edges = self.create_edges(self.node_degrees[newid])
            new_out_edges.initialize()
            newparams = self.create_params(self.nodes_params[newid])
            newparams.initialize()
        
            self.id = newid
            self.out_edges = new_out_edges
            self.params = newparams
        else:
            #changing the out_edges only
            self.out_edges.mutate(pmut=pmut)
            self.params.mutate(pmut=pmut)
    
    def create_params(self,  num_params):
        params = pyevolve.G1DList.G1DList(num_params)
        params.mutator.set(pyevolve.Mutators.G1DListMutatorRealGaussian)
        params.initializator.set(pyevolve.Initializators.G1DListInitializatorReal)
        params.setParams(rangemin = 0, rangemax=1.0, gauss_mu = 0.0, gauss_sigma=0.1)
        return params
    
    def create_edges(self,  out_degree):
        edges = pyevolve.G1DList.G1DList(out_degree)
        edges.mutator.set(edges_mutator)
        edges.initializator.set(edges_initializator)
        edges.setParams(rangemin = 0, 
                        rangemax=self.graph_genome.num_nodes - 1,
                        graph_genome = self.graph_genome)
        return edges
    
    def fix_connectivity(self, edge_number):
        '''
        Fixes the connection to a disabled node pointed by @param edge_number
        '''
        
        new_node = random.randint(0, self.graph_genome.num_nodes - 1)
        
        num_trials = 100
        while self.graph_genome[new_node].disabled:
            if num_trials > 1000:
                raise exceptions.RuntimeError("too many iterations!")
            num_trials += 1
            new_node = random.randint(0, self.graph_genome.num_nodes - 1)
        self.out_edges[edge_number] = new_node
    
    def __repr__(self):
        if self.disabled:
            out_str = "[**%d : %s" % (self.id, 
                                      repr(self.out_edges.genomeList), 
                                      )
        else:
            out_str = "[%d : %s" % (self.id, 
                                    repr(self.out_edges.genomeList), 
                                    )
        if len(self.params.genomeList):
            out_str += "(%s)] " % repr(self.params.genomeList)
        else:
            out_str += "] "
        return out_str
    
    def copy(self,  g):  
        g.id = self.id      
        g.node_degrees = self.node_degrees
        g.node_params = self.nodes_params
        g.graph_genome = self.graph_genome
        g.num_node_types = self.num_node_types
        g.out_edges = self.out_edges.clone()
        g.params = self.params.clone()    
        g.disabled = self.disabled
    
    def clone(self):
        newcopy = NodeGenome(self.id,  
                             self.graph_genome,
                             self.node_degrees,
                             self.nodes_params,
                             )
        self.copy(newcopy)
        newcopy.initialize()
        return newcopy

class GraphGenome(pyevolve.GenomeBase.GenomeBase):
    def __init__(self,  num_nodes, node_degrees, node_params,
                 p_add = 0.0, p_del = 0.0):
        """
        This is the genome representation of a graph. A genome is a collection
        of @NodeGenome.
        
        @param num_nodes: the number of nodes in the graph
        @param node_degrees: a list that specifies the number of outgoing edges 
        for every node. node_degrees[i] = j means that node with id i has j outgoing edges
        @param node_params: node_params[i]=j means that node i has j real-valued
        parameters. It needs to have the same length of node_degrees.   
        """
        self.p_add = p_add
        self.p_del = p_del
        pyevolve.GenomeBase.GenomeBase.__init__(self)
        self.node_degrees = node_degrees
        self.node_params = node_params
        self.num_node_types = len(node_degrees) - 1
        self.disabled_nodes = []
        
        #this is necessary otherwise at the beginning it will report the wrong
        #number of nodes
        self.nodes = [None]*num_nodes
        
        for i in xrange(num_nodes):
            node = NodeGenome(random.randint(0,  self.num_node_types),
                              self,
                              node_degrees,
                              node_params
                              )
            node.initialize()
            self.nodes[i] = node
        
        self.crossover.set(graph_cross)
        self.mutator.set(self.mutate)
        self.initializator.set(self.initialize)
    
    @property
    def num_nodes(self):
#        return len(self.nodes) - len(self.disabled_nodes)
        return len(self.nodes)
    
    def effective_length(self):
        return len(self.nodes) - len(self.disabled_nodes)
    
    def initialize(self,  **args):
        for node in self.nodes:
            node.initialize()
        
    def mutate(self,  **args):        
        '''
        Mutate the whole gene
        '''
        pmut = args["pmut"]
        mutated = 0
        for node in self.nodes:            
            if not (node.disabled) and (random.random() < pmut):                
                mutated =  mutated+1
                node.mutate(pmut)
        
        if random.random() < self.p_add:
            self.add_random_node()
        if random.random() < self.p_del:
            self.remove_random_node()
        
        return mutated
    
    def add_random_node(self):
        '''
        Creates a random node
        '''
        newnode = NodeGenome(random.randint(0,  self.num_node_types),
                                            self,
                                            self.node_degrees,
                                            self.node_params
                                            )
        newnode.initialize()
        if self.disabled_nodes:
            #reuse a disabled node
            index = self.disabled_nodes.pop()
            self.nodes[index] = newnode
        else:            
            self.nodes.append(newnode)
   
    def remove_random_node(self):
        '''
        Removes a random node and fix the connections
        '''
        if self.effective_length() <= 2:
            #do not allow genomes shorter than 2
            return
        index_to_remove = random.randint(0, self.num_nodes - 1)
        
        num_trials = 0
        while self.nodes[index_to_remove].disabled:
            if num_trials > 1000:
                raise exceptions.RuntimeError("too many iterations!")
            num_trials += 1
            index_to_remove = random.randint(0, self.num_nodes - 1)
        
        self.disabled_nodes.append(index_to_remove)
        self.nodes[index_to_remove].disabled = True
        self.nodes[index_to_remove].out_edges[:] = []
        incident_nodes = self.get_incident_nodes(index_to_remove)
        for (node, edge_number) in incident_nodes:
            node.fix_connectivity(edge_number)
        
    def get_incident_nodes(self, node_number):
        '''
        Returns a list of all the nodes that link to node_number
        @param node_number: index in the list of nodes
        @return: a list of tuples node, edge_number
        '''
        
        incident = []
        for node in self.nodes:
            for i in xrange(len(node.out_edges)):
                if node.out_edges[i] == node_number:
                    incident.append((node, i))
        return incident
           
    def node_disabled(self, index):
        return self.nodes[index].disabled     
   
    def __len__(self):
        return self.num_nodes
    
    def __getitem__(self, key):
        return self.nodes[key]
    
    def __setitem__(self, key, value):
        if not isinstance(value, NodeGenome):
            raise exceptions.ValueError("Wrong type: %s"%value.__class__)
        self.nodes[key] = value        
    
    def __repr__(self):
        out_str = ""
        for node in self.nodes:
            out_str += "%s " % repr(node)
        return out_str
    
    def __iter__(self):
        for node in self.nodes:
            yield node

    def copy(self,  g):
        pyevolve.GenomeBase.GenomeBase.copy(self, g)
        
        g.p_add = self.p_add
        g.p_del = self.p_del
        g.node_degrees = self.node_degrees
        g.node_params = self.node_params
        g.num_node_types = self.num_node_types
        g.disabled_nodes = self.disabled_nodes[:]
        
        for i in xrange(len(self.nodes) - 1):
            self.nodes[i].copy(g.nodes[i])        
    
    def clone(self):
        newcopy = GraphGenome(self.num_nodes,  
                              self.node_degrees, 
                              self.node_params)
        self.copy(newcopy)
        return newcopy
    
    def state_clone(self):
        return self.clone()
    
    def create_dict(self):
        '''
        Returns a dictionary {n:[e1,e2..]} where n is a node and e_i are the outgoing edges
        This is useful to create a graph using networkx
        '''
        out = {}
        for i in xrange(self.num_nodes):
            out[i] = self.nodes[i].out_edges[:]
        return out
    
    def create_labels(self):
        '''
        Returns a list of labels, each label being "number (id)". Useful for
        adding labels to a networkx graph.
        '''
        labels = {}
        for i in xrange(self.num_nodes):
            labels[i] = str(i) + "(" + str(self.nodes[i].id)+")" 
        return labels

import networkx as nx

def get_subnodes(multi_digraph, node, num):
    '''
    Get the n closest nodes to node
    @param multi_graph: a networkx graph or anything nx.Graph takes to construct
    @param node: the source node
    @param num: the number of adjacent nodes to return
    
    @return: a list of extracted nodes        
    '''
    
    undirected_G = nx.Graph(multi_digraph)
    edges_set = set()
    for edge in nx.bfs_edges(undirected_G, node):        
        edges_set.add(edge[0])
        if len(edges_set) >=  num:
            break
        edges_set.add(edge[1])
        if len(edges_set) >=  num:
            break
        
    return list(edges_set)

def graph_cross(_, **args):
    gMom = args["mom"]
    gDad = args["dad"]
    
    sister = gMom.clone()
    brother = gDad.clone()
    
    num_nodes_to_swap = random.randint(0, min(len(gMom), len(gDad)))
        
    mom_dict = gMom.create_dict()
    mom_swapping_point = random.randint(0, len(gMom)-1)
    mom_subnodes = get_subnodes(mom_dict, mom_swapping_point, num_nodes_to_swap)
    if len(mom_subnodes) != num_nodes_to_swap:        
        #crossover is not possible here, probably the graph is disjoint
        return (sister, brother)
    
    dad_dict = gDad.create_dict()    
    dad_swapping_point = random.randint(0, len(gDad)-1)    
    dad_subnodes = get_subnodes(dad_dict, dad_swapping_point, num_nodes_to_swap)
    if len(dad_subnodes) != num_nodes_to_swap:        
        #crossover is not possible here, probably the graph is disjoint
        return (sister, brother)
    
    for index in xrange(num_nodes_to_swap):        
        #fixing the outgoing edges for sister
        
        try:
            sister_node = sister[mom_subnodes[index]]
        except exceptions.IndexError:
            print "Sister: ", sister            
            print "Sister Len: ", len(sister)
            print "Brother Len: ", len(brother)
            print "Subnodes: ", mom_subnodes
            print "Dict: ", mom_dict
            print "Index: ", index
            raise
        
        for i,out_edge in enumerate(sister_node.out_edges):  
            if out_edge in mom_subnodes:                                
                out_edge_swapping_pos = mom_subnodes.index(out_edge)
                newpos = dad_subnodes[out_edge_swapping_pos]
                sister_node.out_edges[i] = newpos
            if out_edge >= len(brother):
                sister_node.fix_connectivity(i)
        
        #fixing the outgoing edges for brother
        try:
            brother_node = brother[dad_subnodes[index]]
        except exceptions.IndexError:
            print "Brother: ", brother
            print "Sister Len: ", len(sister)
            print "Brother Len: ", len(brother)
            print "Subnodes: ", dad_subnodes
            print "Dict: ", dad_dict
            print "Index: ", index
            raise
        
        for i,out_edge in enumerate(brother_node.out_edges):  
            if out_edge in dad_subnodes:
                
                out_edge_swapping_pos = dad_subnodes.index(out_edge)
                newpos = mom_subnodes[out_edge_swapping_pos]
                brother_node.out_edges[i] = newpos
            if out_edge >= len(sister) or sister[i].disabled:
                brother_node.fix_connectivity(i) 
        
        #swapping
        mom_index = mom_subnodes[index] 
        dad_index = dad_subnodes[index]
        sister[mom_index], brother[dad_index] = brother[dad_index], sister[mom_index]
    
    if not sanity_check_outside_edges(sister) or not sanity_check_outside_edges(brother):
        print "Brother: ", brother
        print "Sister: ", sister
        print "Dad Subnodes: ", dad_subnodes
        print "Mom subnodes: ", mom_subnodes
        print "Dad Dict: ", dad_dict
        print "Mom Dict: ", mom_dict
        
    
    return (sister, brother)

def sanity_check_outside_edges(genome):
    for node in genome:
        for edge in node.out_edges:
            return (edge < len(genome))    
    

def old_graph_cross(_, **args):        
    '''
    Crossover between two graph genomes    
    '''
    sister = None
    brother = None
    gMom = args["mom"]
    gDad = args["dad"]
    
    cut = random.randint(1, gMom.num_nodes - 1)

    if args["count"] >= 1:
        sister = gMom.clone()
        sister.resetStats()
        
        sister.nodes[cut:] = gDad.nodes[cut:]

    if args["count"] == 2:
        brother = gDad.clone()
        brother.resetStats()
        
        brother.nodes[cut:] = gMom.nodes[cut:]

    return (sister, brother)