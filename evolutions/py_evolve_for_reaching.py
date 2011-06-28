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

def RAND_RANGE(a,b):
    return ((b-a)*random.random() + a)
     
def NORMALIZE(x,a,b):
    return ((x-a)/(b-a))
 
def DENORMALIZE(x,a,b):
    return ( x*(b-a) + a )

#from graph_evolve import rnn
from graph_evolve.rnn import chromosome_convert

class WorldState(object):
    def __init__(self):
        self.robot_pos = (0,0,0)
        self.obj_pos = (2.3,3.5)
        self.table_dims = (1.40, 1.00)
        self.table_pos = (2.5,2.5)            
        self.max_x = 5
        self.max_y = 5
        self.min_x = 0
        self.min_y = 0    
    
    def move_robot_to(self, pos):
        if not self.item_in_table(pos):
            self.robot_pos = pos
        
    def test_graspable(self):
        obj = self.obj_pos
        robot = self.robot_pos
        dx = obj[0] - robot[0]
        dy = obj[1] - robot[1]

        angle = math.atan2(dy,dx) - robot[2]                
        dist = math.sqrt(dx*dx + dy*dy)
        
        return ((angle > -math.pi/4) and 
                (angle < math.pi/4) and
                dist < 1.0)
    
    def item_in_table(self, obj):
        in_x =( obj[0] < self.table_pos[0] + self.table_dims[0]/2. and
                obj[0] > self.table_pos[0] - self.table_dims[0]/2.)
        in_y =( obj[1] < self.table_pos[1] + self.table_dims[1]/2. and
                obj[1] > self.table_pos[1] - self.table_dims[1]/2.)
        return in_x and in_y
    
    def random_robot_pos(self):        
        self.robot_pos = (RAND_RANGE(self.min_x, self.max_x),
                          RAND_RANGE(self.min_y,self.max_y),
                          RAND_RANGE(0., 2.0*math.pi))
        
        while self.item_in_table(self.robot_pos):
            self.robot_pos = (RAND_RANGE(self.min_x, self.max_x),
                              RAND_RANGE(self.min_y,self.max_y),
                              RAND_RANGE(0., 2.0*math.pi))
    
    def random_object_pos(self):
        max_x = self.table_pos[0] + self.table_dims[0]/2.
        min_x = self.table_pos[0] - self.table_dims[0]/2.
        max_y = self.table_pos[1] + self.table_dims[1]/2.
        min_y = self.table_pos[1] - self.table_dims[1]/2.
        
        self.obj_pos = (RAND_RANGE(min_x,max_x),
                        RAND_RANGE(min_y,max_y))
        
        assert self.item_in_table(self.obj_pos)       
    
    def random_table_pos(self):
        lx = self.min_x + self.table_dims[0]
        ux = self.max_x - self.table_dims[0]
        ly = self.min_y + self.table_dims[1]
        uy = self.max_y - self.table_dims[1]
        
        self.table_pos = (RAND_RANGE(lx,ux),
                          RAND_RANGE(ly,uy))
    
    def random_table_dims(self):
        self.table_dims = (RAND_RANGE(0.5, 1.4),
                           RAND_RANGE(0.5, 1.4))
    
    def randomize(self):
        self.random_table_dims()
        self.random_table_pos()
        self.random_object_pos()
        self.random_robot_pos()
        
    def normalised_input(self):        
        input = [0]*6
        input[0] = NORMALIZE(self.table_pos[0], self.min_x, self.max_x)
        input[1] = NORMALIZE(self.table_pos[1], self.min_y, self.max_y)
        input[2] = NORMALIZE(self.table_dims[0], self.min_x, self.max_x)
        input[3] = NORMALIZE(self.table_dims[1], self.min_y, self.max_y)
        input[4] = NORMALIZE(self.obj_pos[0], self.min_x, self.max_x)
        input[5] = NORMALIZE(self.obj_pos[1], self.min_y, self.max_y)
        
        return input
    
    def denormalise_pos(self, vect):
        out = [0]*3
        out[0] = DENORMALIZE(vect[0], self.min_x, self.max_x)
        out[1] = DENORMALIZE(vect[1], self.min_y, self.max_y)
        out[2] = DENORMALIZE(vect[2], 0, 2.*math.pi)
        return out

def eval_func(chromosome,  **args):
    
    net = chromosome_convert(chromosome)
    world = WorldState()
    num_trials = 100
    num_steps = 20
    succ = 0
    input = world.normalised_input()
    
    for i in xrange(num_trials):
        world.randomize()
        while world.test_graspable():
            world.randomize()
        for j in xrange(num_steps):
            netout = net(input)
        newpose = world.denormalise_pos(netout)
        world.move_robot_to(newpose)
        if world.test_graspable():
            succ += 1        
    fitness = float(succ) / float(num_trials)
    
    return fitness
    

def stepCallback(ga_engine):
    generation = ga_engine.getCurrentGeneration()
    if generation % 10 == 0:
        best = ga.bestIndividual()
        net = chromosome_convert(best)
        
        pickle_file = open("/home/pezzotto/tmp/best.txt", "w")
        pickle.dump(net,  pickle_file, 0)
        pickle_file.close()
        score = eval_func(best)
        print "Score: ", score
        
        
    return False

if __name__ == "__main__":
    
    
    # Genome instance
    input_size = 6
    hidden_size = 6
    output_size = 3
    bias_size = hidden_size + output_size
    total_size = input_size + hidden_size + output_size
    genome_size = (total_size - input_size)*total_size + bias_size
    
    genome = G1DList.G1DList(genome_size)
    genome.setParams(rangemin=-1, rangemax=1)
    genome.setParams(input_size=input_size,  hidden_size=hidden_size,  output_size=output_size)
    genome.setParams(gauss_mu=0., gauss_sigma=0.2)
    
    genome.initializator.set(Initializators.G1DListInitializatorReal)
    genome.mutator.set(Mutators.G1DListMutatorRealGaussian)
    genome.evaluator.set(eval_func)
    genome.crossover.set(Crossovers.G1DListCrossoverTwoPoint)
    
    ga = GSimpleGA.GSimpleGA(genome)
    #ga.selector.set(Selectors.GRouletteWheel)
    ga.selector.set(Selectors.GRankSelector)
    ga.setGenerations(500)
    ga.setPopulationSize(1000)
    ga.setCrossoverRate(0.0)
    ga.setMutationRate(0.3)
    ga.setMinimax(Consts.minimaxType["maximize"])
    
    ga.stepCallback.set(stepCallback)
#    print "STOPPING ELITISM"
#    ga.setElitism(False)
    ga.setElitismReplacement(20)
    ga.setMultiProcessing(True, False)
    
    # Do the evolution
    ga.evolve(freq_stats = 10)
    
    # Best individual
    best = ga.bestIndividual()
    print best
    net = chromosome_convert(best)
    
    pickle_file = open("/home/pezzotto/tmp/best.txt", "w")
    pickle.dump(net,  pickle_file, 0)
    pickle_file.close()
    
    print "Done"
