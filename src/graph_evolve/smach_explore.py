import pyrecurrentnet
import smach
import random
import math
from simulators import grasping_world
import functools

network_input_size = 3
network_output_size = 3
network_hidden_size = 3
network_bias_size = network_hidden_size + network_output_size
network_total_size = network_input_size + network_hidden_size + network_output_size
params_size = (network_total_size - network_input_size)*network_total_size + network_bias_size

max_transitions = 40 
net_evolutions = 30

max_w = 3.
min_w = -3.

class NeuralNetwork(smach.State):
    
    def __init__(self, params,  world):
        smach.State.__init__(self,
                             outcomes=['success', "timeout"],
                             input_keys=['network_in'],
                             output_keys=['network_out'])
        self.world = world
        assert(len(params) == params_size)
        
        newparams = [ (max_w-min_w)*p +min_w for p in params ]
        
        self.net = pyrecurrentnet.list_convert(newparams, 
                                               network_input_size, 
                                               network_output_size, 
                                               network_hidden_size)
        
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= max_transitions:
            return "timeout"
        
        try:
            inpt = userdata.network_in
        except KeyError:
            inpt = [random.random() for _ in xrange(network_input_size)] 
        
        self.net.randomiseState()
        netout = self.net.evolve(inpt, net_evolutions)
        
        userdata.network_out = netout
        return "success"
        
class RobotMove(smach.State):    
    def __init__(self, world):
        smach.State.__init__(self,
                             outcomes=['success', "failure", "timeout"],
                             input_keys=['move_pos'],
                             )
        self.world = world
        
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= max_transitions:
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

class RandomMove(smach.State):    
    def __init__(self, params, world):
        smach.State.__init__(self,
                             outcomes=['success', "failure", "timeout"],
                             input_keys=['move_pos'],
                             )
        self.world = world
        
        max_si = 0.1
        min_si = 0.01       
        
        self.mu_x = params[0]
        self.mu_y = params[1]
        self.mu_th = params[2]
        
        self.si_x = (max_si-min_si)*params[3] + min_si
        self.si_y = (max_si-min_si)*params[4] + min_si
        self.si_th = (max_si-min_si)*params[5] + min_si
        
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= max_transitions:
            return "timeout"
       
        newx = random.gauss(self.mu_x, self.si_x)
        newy = random.gauss(self.mu_y, self.si_y)
        newth = random.gauss(self.mu_th, self.si_th)
        
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
        
        newpos = (lx + newx *(rx-lx),
                  ly + newy *(ry-ly),  
                  lt + newth *(rt-lt))
               
        if self.world.move_robot(newpos):
            return "success"
        else:
            return "failure"
    
class ExitSuccess(smach.State):
    def __init__(self, world):
        smach.State.__init__(self,
                             outcomes = ['success', 'failure', "timeout"],
                             )
        self.world = world
    
    def execute(self, _):
        self.world.inc_time()
        if self.world.time_step >= max_transitions:
            return "timeout"
        
        if self.world.robot_sees_objects():
            return "success"
        else:
            return "failure"

class ExplorerFactory2(object):
    def __init__(self):
        self.world = grasping_world.GraspingWorld()
        self.world.create_with_fixed_robot()
        
        self.names_mapping = ["RandomMove", "ExitSuccess"]
        
        self.classes_mapping = {}
        for name in self.names_mapping:
            class_ = globals()[name]
            self.classes_mapping[name] = functools.partial(class_, 
                                                           world = self.world)
        
        self.transitions_mapping = {"RandomMove" : ['success', "failure",],
                                    "ExitSuccess": ["failure"]}
        data_mapping = {}
        data_mapping["ExitSuccess"] = {}
        data_mapping["RandomMove"] = {}
        self.data_mapping = data_mapping
        
        self.node_degrees = [2, 1]
        self.node_params = [6, 0]
        
class ExplorerFactory(object):
    def __init__(self):
        self.world = grasping_world.GraspingWorld()
        self.world.create_with_fixed_robot()
        
        self.names_mapping = ["NeuralNetwork", "RobotMove", "ExitSuccess"]
        
        self.classes_mapping = {}
        for name in self.names_mapping:
            class_ = globals()[name]
            self.classes_mapping[name] = functools.partial(class_, 
                                                           world = self.world)
                   
        self.transitions_mapping = {"NeuralNetwork" : ['success'],
                                    "RobotMove" : ['success', "failure",],
                                    "ExitSuccess": ["failure"]}
        data_mapping = {}
        data_mapping["NeuralNetwork"] = {"network_in" : "net_value",
                                         "network_out" : "net_value"}
        data_mapping["RobotMove"] = {"move_pos": "net_value"}
        data_mapping["ExitSuccess"] = {}
        self.data_mapping = data_mapping
        
        self.node_degrees = [1, 2, 1]
        self.node_params = [params_size, 0, 0]