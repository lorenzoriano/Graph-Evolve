import pyrecurrentnet
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


class ExperimentSetup(object):
    def __init__(self):
        self.network_input_size = 4
        self.network_output_size = 4
        self.network_hidden_size = 3
        
        self.network_bias_size = None
        self.network_total_size = None
        self.params_size = None
        
        self.max_transitions = 40 
        self.net_evolutions = 30
        
        self.max_w = 3.
        self.min_w = -3.
        
        self.date = ""
        self.note = ""
        self.name = ""
    
        self.rbfnetworkpath = ""
        self.rbfnet = None
        self.normalizer = None
    
        self.explorer_path = ""
        self.explorer_genome = None
        self.explorer_experiment = None    
        
        #actual evolution bits        
        self.num_nodes= 100
        self.pop_size = 10
        
        self.poolsize = self.pop_size
        self.migration_size = 1
        self.migration_rate = 1
        self.elitism_size = 1
        self.generations = 5000
        self.num_trials = 300
        self.stop_elitism = False
        self.freq_stats = 10
        self.crossover_rate = 0.1
        self.mutation_rate = 0.1
        self.p_del = 0.1
        self.p_add = 0.1
        
    
    def initialize(self):
        self.date = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S/")
        self.network_bias_size = (self.network_hidden_size + 
                                  self.network_output_size)
        self.network_total_size = (self.network_input_size + 
                                   self.network_hidden_size + 
                                   self.network_output_size)
        self.params_size = ((self.network_total_size - self.network_input_size)*
                            self.network_total_size + self.network_bias_size)
       
        f = open(self.rbfnetworkpath, "rb")
        net, normalizer = cPickle.load(f)
        self.rbfnet = net
        self.normalizer = normalizer
        
        f = open(self.explorer_path, "rb")        
        genome, factory, experiment = cPickle.load(f)
        
        self.explorer_genome = genome
        self.explorer_experiment = experiment

class NeuralNetwork(smach.State):
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= self.experiment.max_transitions:
            return "timeout"
        
        
        inpt = self.world.net_input()        
        dx, dy, dth = 2*self.net.activate(inpt) - 1
            
        userdata.network_out = (dx, dy, dth)
        return "success"
 
class NeuralNetwork2(NeuralNetwork):
    
    outcomes = ["success", "timeout"]
    input_keys = []
    output_keys = ["network_out"]
    nparams = 25
    
    def __init__(self, params,  world, experiment):
        smach.State.__init__(self,
                             outcomes = self.outcomes,
                             input_keys = self.input_keys,
                             output_keys=self.output_keys)
        self.world = world
        assert(len(params) == self.nparams)
        
        newparams = [ (experiment.max_w-experiment.min_w)*p +
                     experiment.min_w for p in params ]
        
        self.net = buildNetwork(7,2,3, 
                        outclass = pybrain.structure.modules.SigmoidLayer)
        self.net.params[:] = newparams
        self.experiment = experiment
        
       
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

class ExplorerFactory(object):
    def __init__(self, experiment):
        self.world = svm_grasping_world.GraspingWorld(
                experiment.object_transformation,
                experiment.table_transformation,
                experiment.grasping_tester)
        
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
        self.data_mapping = {}
        
        self.node_degrees = [len(self.transitions_mapping[name]) 
                            for name in self.names_mapping]
        
        self.node_params = [self.classes_mapping[name].nparams 
                            for name in self.names_mapping]
    
    def __new__(cls, explorer):
        instance  = super(ExplorerFactory, cls).__new__(cls)
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
        
        
