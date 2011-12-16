import smach
import math
from simulators import svm_grasping_world
import functools
import datetime
import cPickle
from graph_evolve.chromosome_smach import convert_chromosome_smach
import pybrain
from pybrain.tools.shortcuts import buildNetwork


#this is for pickle satisfaction only        
def eval_func(chromosome, **args):
    pass
import sys
try:
    sys.modules["__main__"].eval_func
except AttributeError:
    sys.modules["__main__"].eval_func = eval_func

pi = math.pi
pi2 = pi/2
class ExperimentSetup(object):
    def __init__(self):
        
        self.max_transitions = 10
        self.net_evolutions = 30
        
        self.max_w = 4.
        self.min_w = -4.
        
        self.date = ""
        self.note = ""
        self.name = ""
    
        self.tableSVR_path = "/media/isrc_private/Logs/TryToPushAndActionConsequence/table_changesSVR.pkl"
        self.objectSVR_path = "/media/isrc_private/Logs/TryToPushAndActionConsequence/object_changesSVR.pkl"
        self.reachableSVC_path = "/media/isrc_private/Logs/TryToPushAndActionConsequence/reachableSVC.pkl"
        self.testingdata_path = "/media/isrc_private/Logs/TryToPushAndActionConsequence/given_fsm_2011-11-25-18-23-28.pkl"

        #actual evolution bits        
        self.num_nodes= 100
        self.pop_size = 10
        
        self.poolsize = self.pop_size
        self.elitism_size = 1
        self.generations = 5000
        self.stop_elitism = False
        self.freq_stats = 10
        self.crossover_rate = 0.1
        self.mutation_rate = 0.1
        self.p_del = 0.1
        self.p_add = 0.1
        
    
    def initialize(self):
        self.date = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S/")
        self.tableSVR = cPickle.load(open(self.tableSVR_path))
        self.objectSVR = cPickle.load(open(self.objectSVR_path))
        self.reachableSVC = cPickle.load(open(self.reachableSVC_path))
        
        data = cPickle.load(open(self.testingdata_path))
        self.test_input = data[0]
        self.test_input[:,[4,5]] = self.test_input[:,[5,4]]
        #objx, objy, objz, table_minx, table_miny, table_maxx, table_maxy
        self.test_output = data[1]

class NeuralNetwork(smach.State):
    outcomes = ["success", "timeout"]
    input_keys = []
    output_keys = ["network_out"]
    
    def __init__(self, nhidden, params,  world, experiment):
        smach.State.__init__(self,
                             outcomes = self.outcomes,
                             input_keys = self.input_keys,
                             output_keys=self.output_keys)
        self.world = world
        assert(len(params) == self.nparams)
        
        newparams = [ (experiment.max_w-experiment.min_w)*p +
                     experiment.min_w for p in params ]
       
        self.net = buildNetwork(7,nhidden,3, 
                        outclass = pybrain.structure.modules.SigmoidLayer)
        self.net.params[:] = newparams
        self.experiment = experiment
    
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= self.experiment.max_transitions:
            return "timeout"
        
        
        inpt = self.world.net_input()
        #print "INPUT: ", inpt
        dx, dy, dth = self.net.activate(inpt)
        dx = dx 
        dy = 2*dy - 1.0
        dth = pi * dth - pi2
        #print "Output: ", (dx, dy, dth)
            
        userdata.network_out = (dx, dy, dth)
        return "success"
 
class NeuralNetwork2(NeuralNetwork):
    
    nparams = 25
    
    def __init__(self, params,  world, experiment):
        NeuralNetwork.__init__(self, 2, params, world, experiment)
       
       
class RobotMove(smach.State):
    outcomes = ['success', "failure", "timeout"]
    input_keys = ["network_out"]
    nparams = 0

    def __init__(self, world, experiment):
        smach.State.__init__(self,
                             outcomes = self.outcomes,
                             input_keys=self.input_keys,
                             )
        self.world = world
        self.experiment = experiment
        
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= self.experiment.max_transitions:
            return "timeout"
       
        try:
            pos = userdata.network_out
        except KeyError:
            return "failure"
        
        if self.world.move_robot(pos):
            return "success"
        else:
            return "failure"

class ExitSuccess(smach.State):
    outcomes = ['success', "failure", "timeout"]
    input_keys = []
    output_keys = []
    nparams = 0
    
    def __init__(self, world, experiment):
        smach.State.__init__(self,
                             outcomes = self.outcomes,
                             input_keys = self.input_keys
                             )
        self.world = world
        self.experiment = experiment
    
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= self.experiment.max_transitions:
            return "timeout"
        
        if self.world.can_grasp():
            return "success"
        else:
            return "failure"

class SVMWorldFactory(object):
    def __init__(self, experiment):
        self.world = svm_grasping_world.GraspingWorld(
                experiment.objectSVR,
                experiment.tableSVR,
                experiment.reachableSVC)
       
        

        self.names_mapping = ["NeuralNetwork2", 
                              "RobotMove", 
                              "ExitSuccess", 
                              ]
        self.experiment = experiment
       
        all_classes = dict((name,globals()[name]) for name in self.names_mapping)

        self.classes_mapping = dict( (name, functools.partial(class_, 
                                                        world = self.world,
                                                        experiment = experiment)
                                      ) 
                    for name, class_ in all_classes.iteritems()
                    )

        
        self.transitions_mapping = dict( (name, class_.outcomes[:-1]) #discard timeout
                                    for name, class_ in  all_classes.iteritems()
                                       )
        self.transitions_mapping["ExitSuccess"]= ["failure"] #success it the end of the FSM
        self.data_mapping = dict((name, {}) for name in self.names_mapping)
        
        self.node_degrees = [len(self.transitions_mapping[name]) 
                            for name in self.names_mapping]
        
        self.node_params = [all_classes[name].nparams 
                            for name in self.names_mapping]

        #print
        #print "names_mapping: ", self.names_mapping
        #print "classes_mapping: ", self.classes_mapping
        #print "transitions_mapping: ", self.transitions_mapping
        #print "data_mapping: ", self.data_mapping
        #print "node_params: ", self.node_params
        #print
    
    def __new__(cls, explorer):
        instance  = super(SVMWorldFactory, cls).__new__(cls)
        instance.__init__(explorer)
        return instance        
    
    def __getnewargs__(self):
        return (self.experiment,)
    
    def __getinitargs__(self):
        return (self.experiment,)
    
    def __getstate__(self):
        return False
    
    def __setstate__(self, state):
        pass
        
