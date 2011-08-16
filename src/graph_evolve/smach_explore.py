import pyrecurrentnet
import smach
import random
import math

network_input_size = 3
network_output_size = 3
network_hidden_size = 0
network_bias_size = network_hidden_size + network_output_size
network_total_size = network_input_size + network_hidden_size + network_output_size
params_size = (network_total_size - network_input_size)*network_total_size + network_bias_size

max_transitions = 20 
net_evolutions = 10

class NeuralNetwork(smach.State):
    
    def __init__(self, params,  world):
        smach.State.__init__(self,
                             outcomes=['success', "timeout"],
                             input_keys=['network_in'],
                             output_keys=['network_out'])
        self.world = world
        assert(len(params) == params_size)
        
        self.net = pyrecurrentnet.list_convert(params, 
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
        lx = self.world.min_x
        rx = self.world.max_x
        ly = self.world.min_y
        ry = self.world.max_y
        lt = -math.pi
        rt = math.pi
        
        newpos = (lx + pos[0] *(rx-lx),
                  ly + pos[1] *(ry-ly),  
                  lt + pos[2] *(rt-lt))
        
        if self.world.move_robot(newpos):
            return "success"
        else:
            return "failure"
        
class CheckSuccess(smach.State):
    def __init__(self, world):
        smach.State.__init__(self,
                             outcomes = ['success', 'failure', "timeout"],
                             )
        self.world = world
    
    def execute(self, userdata):
        self.world.inc_time()
        if self.world.time_step >= max_transitions:
            return "timeout"
        
        if self.world.robot_sees_objects():
            return "success"
        else:
            return "failure"
        
        
        