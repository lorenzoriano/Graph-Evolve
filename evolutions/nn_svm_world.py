import sys
sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src')
sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src\\pyevolve')

sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src')
sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src/pyevolve')

from pyevolve import GSimpleGA
from pyevolve import Selectors
from pyevolve import Consts
from pyevolve import G1DList

from simulators import multi_svm 
sys.modules["multi_svm"] = multi_svm

import cPickle
import random
import time
import pybrain
from pybrain.tools.shortcuts import buildNetwork
from simulators.svm_grasping_world import GraspingWorld
import pyevolve

from math import pi
pi2 = pi/2

tableSVR_path = "/home/pezzotto/Logs/TryToPushAndActionConsequence/table_changesSVR.pkl"
objectSVR_path = "/home/pezzotto/Logs/TryToPushAndActionConsequence/object_changesSVR.pkl"
reachableSVC_path = "/home/pezzotto/Logs/TryToPushAndActionConsequence/reachableSVC.pkl"
testingdata_path = "/home/pezzotto/Logs/TryToPushAndActionConsequence/given_fsm_2011-11-25-18-23-28.pkl"
data = cPickle.load(open(testingdata_path))
test_input = data[0]
test_input[:,[4,5]] = test_input[:,[5,4]]
#objx, objy, objz, table_minx, table_miny, table_maxx, table_maxy
test_output = data[1]

def single_try(genome, net, inpt, world):
    obj = inpt[:3]
    table = inpt[3:]
    world.create_object(obj)
    world.create_table(table)
    
    inpt = world.net_input()
    dx, dy, dth = net.activate(inpt)
    dx = dx 
    dy = 2*dy - 1.0
    dth = pi * dth - pi2

    world.move_robot((dx,dy,dth))
    return 

def eval_func(genome, **args):    
    net = buildNetwork(7,2,3, 
                        outclass = pybrain.structure.modules.SigmoidLayer)
    maxw = 3
    minw = -3
    net.params[:] = [(maxw-minw)*genome[i] + minw for i in xrange(len(genome))]

    objectSVR = cPickle.load(open(objectSVR_path))
    tableSVR = cPickle.load(open(tableSVR_path))
    reachableSVC = cPickle.load(open(reachableSVC_path))


    world = GraspingWorld(objectSVR, tableSVR, 
                             reachableSVC)

    fitness = 0.0
    for inpt, outp in zip(test_input, test_output):
        single_try(genome, net, inpt, world)
        
        if outp and (not world.can_grasp()):
            score = 0
        else:
            score = world.grasp_probability()
        
        fitness += score
    
    return fitness


def stepCallback(ga_engine):
    generation = ga_engine.getCurrentGeneration()
    if generation % 10 == 0:
        ga_engine.printTimeElapsed()
    return False

def main():
    random.seed(time.time())
    sigma = 0.05

    num_params = buildNetwork(7,2,3,).params.shape[0]

    genome = pyevolve.G1DList.G1DList(num_params)
    genome.evaluator.set(eval_func)
    genome.mutator.set(pyevolve.Mutators.G1DListMutatorRealGaussian)
    genome.initializator.set(pyevolve.Initializators.G1DListInitializatorReal)
    genome.setParams(rangemin = 0, rangemax=1.0, 
                     gauss_mu = 0.0, gauss_sigma=sigma)
    genome.initialize()

    ga = GSimpleGA.GSimpleGA(genome)
    
    ga.setElitism(True)
    ga.setElitismReplacement(1)
    
    ga.selector.set(Selectors.GRouletteWheel)
    
    ga.setGenerations(1000)
    ga.setPopulationSize(1000)
    ga.setCrossoverRate(0.1)
    ga.setMutationRate(0.1)
    ga.getPopulation().setParams(tournamentPool = 100)

    ga.setMinimax(Consts.minimaxType["maximize"])
    ga.stepCallback.set(stepCallback)
    
    #print "Using MultiProcessing"
    ga.setMultiProcessing(False, full_copy=False) 
        

    # Do the evolution
    ga.evolve(freq_stats = 10)

    
if __name__ == "__main__":
    main()
