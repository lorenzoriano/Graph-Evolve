import pyrecurrentnet
import smach
import random
import math
from simulators import grasping_world
import functools
import datetime
import rbfnetwork
import cPickle
import smach_explore
from graph_evolve.chromosome_smach import convert_chromosome_smach


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

class Explorer(smach.State):
    def __init__(self, world, experiment):
        smach.State.__init__(self,
                             outcomes=['success', "failure", "timeout"],
                             output_keys = ['object_found'])
        factory = smach_explore.ExplorerFactory(experiment.explorer_experiment,
                                                world = world)
        
        self.sm = convert_chromosome_smach(experiment.explorer_genome, 
                                  factory.names_mapping, 
                                  factory.transitions_mapping, 
                                  factory.classes_mapping,
                                  factory.data_mapping)
        
        self.experiment = experiment
        self.world = world
    
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= self.experiment.max_transitions:
            return "timeout"
         
        outcome = self.sm.execute()
        if outcome == "success":
            userdata.object_found = True
        return outcome
        

class NeuralNetwork(smach.State):
    
    def __init__(self, params,  world, experiment):
        smach.State.__init__(self,
                             outcomes=['success', "failure", "timeout"],
                             input_keys = ['object_found'],                           
                             output_keys=['network_out'])
        self.world = world
        assert(len(params) == experiment.params_size)
        
        newparams = [ (experiment.max_w-experiment.min_w)*p +
                     experiment.min_w for p in params ]
        
        self.net = pyrecurrentnet.list_convert(newparams, 
                                               experiment.network_input_size, 
                                               experiment.network_output_size, 
                                               experiment.network_hidden_size)
        self.experiment = experiment
        
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= self.experiment.max_transitions:
            return "timeout"
        
        try:
            if not userdata.object_found:
                return "failure"
        except KeyError:
            return "failure" 
        
        
        robot = self.world.robot
        obj = self.world.objects[0]
        table = self.world.table
        
        if not robot.sees(obj):
            return "failure"         
        
        obj = obj.create_roto_translation(robot.pos, robot.th)
        table = table.create_roto_translation(robot.pos, robot.th)
        
        bias = 0.0
        lx = 0.0 - bias
        rx = 1.5 + bias
        ly = -1.5 - bias
        ry = 1.5 + bias
        
        
        inpt = ((obj.pos[0]-lx) / (rx-lx),
                (obj.pos[1]-ly) / (ry-ly),
                (table.pos[0]-lx) /(rx-lx),
                (table.pos[1]-ly) /(ry-ly)
                )
        
        newinpt = tuple()
        for i in inpt:
            if i > 1.0:
                newinpt += (1.0,)
            elif i < 0.0:
                newinpt += (-0.0,)
            else:
                newinpt += (i,)
        inpt = newinpt
        
        self.net.randomiseState()
#        print "\n\nPEZZZZ input= ",inpt
#        print "OBJ: ", obj.pos
#        print "TABLE: ", table.pos
#        print "Sees it: ", robot.sees(self.world.objects[0])
#        print "\n\n"
        
        try:
            netout = self.net.evolve(inpt, self.experiment.net_evolutions)
        except RuntimeError:
            print "Object: ", self.world.objects[0].pos
            print "Table: ", self.world.table.pos
            print "Robot: ", robot.pos, " th: ", robot.th
            print "\n"
            print "Translated obj: ", obj.pos
            print "Translated table: ", table.pos
            print "Net input: ", inpt 
        
        userdata.network_out = netout
        return "success"
        
class RobotMove(smach.State):    
    def __init__(self, world, experiment):
        smach.State.__init__(self,
                             outcomes=['success', "failure", "timeout"],
                             input_keys=['move_pos'],
                             )
        self.world = world
        self.experiment = experiment
        
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= self.experiment.max_transitions:
            return "timeout"
       
        try:
            pos = userdata.move_pos
        except KeyError:
            return "failure"
        
        #denormalizing the network output
#        lx = self.world.min_x
#        rx = self.world.max_x
#        ly = self.world.min_y
#        ry = self.world.max_y
        
        lx = 0
        rx = 5
        ly = -2.5
        ry = 2.5
        
        lt = -math.pi
        rt = math.pi
        
        newpos = (lx + pos[0] *(rx-lx),
                  ly + pos[1] *(ry-ly),  
                  lt + pos[2] *(rt-lt))
        
        if self.world.move_robot(newpos):
            return "success"
        else:
            return "failure"

class ExitSuccess(smach.State):
    def __init__(self, world, experiment):
        smach.State.__init__(self,
                             outcomes = ['success', 'failure', "timeout"],
                             input_keys=['object_found']
                             )
        self.world = world
        self.experiment = experiment
    
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= self.experiment.max_transitions:
            return "timeout"
        
        try:
            if not userdata.object_found:
                return "failure"
        except KeyError:
            return "failure" 
        
        robot = self.world.robot
        obj = self.world.objects[0]
        
        if robot.can_grasp(obj, self.experiment.rbfnet,
                           self.experiment.normalizer):
            return "success"
        else:
            return "failure"

class ExplorerFactory(object):
    def __init__(self, experiment):
        self.world = grasping_world.GraspingWorld()
        self.world.create_with_fixed_robot()
        
        self.names_mapping = ["NeuralNetwork", "RobotMove", "ExitSuccess", "Explorer"]
        self.experiment = experiment
        
        self.classes_mapping = {}        
        for name in self.names_mapping:
            class_ = globals()[name]
            self.classes_mapping[name] = functools.partial(class_, 
                                                           world = self.world,
                                                           experiment = experiment)
                   
        self.transitions_mapping = {"NeuralNetwork" : ['success', "failure"],
                                    "RobotMove" : ['success', "failure",],
                                    "Explorer": ['success', "failure"],
                                    "ExitSuccess": ["failure"]}
        data_mapping = {}
        data_mapping["NeuralNetwork"] = {"network_in" : "net_value",
                                         "network_out" : "net_value"}
        data_mapping["RobotMove"] = {"move_pos": "net_value"}
        data_mapping["Explorer"] = {"object_found" : "object_found"}
        data_mapping["ExitSuccess"] = {"object_found" : "object_found"}
        self.data_mapping = data_mapping
        
        self.node_degrees = [2, 2, 1, 2]
        self.node_params = [experiment.params_size, 0, 0, 0]
    
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
        
        