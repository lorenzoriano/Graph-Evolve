import smach
import math
import random
import copy

#WARNING: THE AXIS ARE WEIRD!!

table_dim = (0.5, 0.3) #width, height
table_dist_x = 0.2
table_pos_z = 0.5

success_prob = 1.0

class Object(object):
    def __init__(self, pos=None, dims = None):
        if pos is not None and dims is not None:
            self.starting_pos = pos
            self.pos = pos
            self.height = dims[0]
            self.width = dims[1]
            calculate_boundaries(self)
            self.initialized = True
            return

        min_x = table_dist_x
        max_x = min_x + table_dim[1]
        
        min_y = - table_dim[0] 
        max_y =  table_dim[0]
        
        self.starting_pos = (random.uniform(min_x, max_x),
                             random.uniform(min_y, max_y),
                             table_pos_z)
        
        self.pos = (None, None, None)
        self.height = None
        self.width = None
        
        self.x_boundaries = None
        self.y_boundaries = None
        self.z_boundaries = None
        self.initialized = False

def clone_object(obj):
    
    if obj is None:
        return None
    return copy.copy(obj)
#    ret = Object()
#    ret.pos = obj.pos
#    ret.height = obj.height
#    ret.width = obj.width
#    ret.starting_pos = obj.starting_pos
#    
#    ret.x_boundaries = obj.x_boundaries
#    ret.y_boundaries = obj.y_boundaries
#    ret.z_boundaries = obj.z_boundaries
#    ret.initialized = obj.initialized
    

def dist_poses(pos1, pos2):
    
    return math.sqrt( (pos1[0]-pos2[0])**2 +
                      (pos1[1]-pos2[1])**2 +
                      (pos1[2]-pos2[2])**2)
            

def initialize(obj):
    obj.initialized = True        

    obj.pos = obj.starting_pos

    obj.height = random.uniform(0.01, 0.02)
    obj.width = random.uniform(0.01, 0.03)
    
    calculate_boundaries(obj)


def calculate_boundaries(obj):
    obj.x_boundaries = (obj.pos[0] - obj.height/2.,
                         obj.pos[0] + obj.height/2)
                         
    obj.y_boundaries = (obj.pos[1] - obj.width/2.,
                         obj.pos[1] + obj.width/2)
    
    obj.z_boundaries = (obj.pos[2], obj.pos[2] + obj.height)

def point_inside(obj, point):
    
    inside_x = obj.x_boundaries[0] <= point[0] <= obj.x_boundaries[1]
    inside_y = obj.y_boundaries[0] <= point[1] <= obj.y_boundaries[1]
    inside_z = obj.z_boundaries[0] <= point[2] <= obj.z_boundaries[1]
    
    return inside_x and inside_y and inside_z

def __objects_collide(obj1, obj2):
    '''
    True if at least one of the vertex of ob1 is inside the volume of obj2
    '''
    
    x_bnd = (obj1.x_boundaries[1] > obj2.x_boundaries[0] and
             obj1.x_boundaries[0] < obj2.x_boundaries[1])
    
    y_bnd = (obj1.y_boundaries[1] > obj2.y_boundaries[0] and
             obj1.y_boundaries[0] < obj2.y_boundaries[1] )
    
    z_bnd = (obj1.z_boundaries[1] > obj2.z_boundaries[0] and
             obj1.z_boundaries[0] < obj2.z_boundaries[1])
    
    return ( x_bnd and y_bnd and z_bnd)
            
    
    
#    pos = (obj1.x_boundaries[0], obj1.y_boundaries[0], obj1.z_boundaries[0])
#    if point_inside(obj2, pos):
#        return True
#    pos = (obj1.x_boundaries[0], obj1.y_boundaries[0], obj1.z_boundaries[1])
#    if point_inside(obj2, pos):
#        return True
#    pos = (obj1.x_boundaries[0], obj1.y_boundaries[1], obj1.z_boundaries[0])
#    if point_inside(obj2, pos):
#        return True
#    pos = (obj1.x_boundaries[0], obj1.y_boundaries[1], obj1.z_boundaries[1])
#    if point_inside(obj2, pos):
#        return True
#    
#    pos = (obj1.x_boundaries[1], obj1.y_boundaries[0], obj1.z_boundaries[0])
#    if point_inside(obj2, pos):
#        return True
#    pos = (obj1.x_boundaries[1], obj1.y_boundaries[0], obj1.z_boundaries[1])
#    if point_inside(obj2, pos):
#        return True
#    pos = (obj1.x_boundaries[1], obj1.y_boundaries[1], obj1.z_boundaries[0])
#    if point_inside(obj2, pos):
#        return True
#    pos = (obj1.x_boundaries[1], obj1.y_boundaries[1], obj1.z_boundaries[1])
#    if point_inside(obj2, pos):
#        return True
    
#    return False

def objects_collide(obj1, obj2):
    return __objects_collide(obj1, obj2) or __objects_collide(obj2, obj1) 

def denormalize(x,a,b):
    return x*(b-a) + a            

def null_print(_):
    pass

class RobotWorldState(object):
    def __init__(self):
        self.obj1 = Object()
        self.obj2 = Object()
        
        self.gripper_pos = (0,0,0)
        self.gripper_closed = False
        self.object_in_gripper = None
        self.number_of_transitions = 0
        self.max_number_of_transitions = 10

def object_in_gripper(state):
    if state.object_in_gripper is None:
        return None
    elif state.object_in_gripper == 0:
        return state.obj1
    elif state.object_in_gripper == 1:
        return state.obj2

def clone_robot_world_state(state):
    ret = RobotWorldState()
    ret.obj1 = clone_object(state.obj1)
    ret.obj2 = clone_object(state.obj2)
    
    ret.gripper_pos = state.gripper_pos
    ret.gripper_closed = state.gripper_closed
    ret.object_in_gripper = state.object_in_gripper
    ret.number_of_transitions = state.number_of_transitions
    ret.max_number_of_transitions = state.max_number_of_transitions
    
    return ret

class DetectObjects(smach.State):
    
    nparams = 0
    
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', "timeout"],
                             input_keys=['state_in'],
                             output_keys=['state_out'])
        
    def execute(self, userdata):
        
        robot_state = clone_robot_world_state(userdata.state_in)        
        robot_state.number_of_transitions +=1
        
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            userdata.state_out = robot_state       
            return "timeout"
        
        initialize(robot_state.obj1)
        initialize(robot_state.obj2)
        
        while objects_collide(robot_state.obj1, robot_state.obj2):
            robot_state.obj1 = Object()
            robot_state.obj2 = Object()
            initialize(robot_state.obj1)        
            initialize(robot_state.obj2)
        
        userdata.state_out = robot_state
        return "success"

class MoveGripperTo(smach.State):
    
    nparams = 4
    
    def __init__(self, target):
        smach.State.__init__(self,
                             outcomes=['success', 'failure',"timeout"],
                             input_keys=['state_in'],
                             output_keys=['state_out'])

        self.which_obj = target[0]
        
        maxw = 2.0
        minw = -2.0
        self.weights = (denormalize(target[1], minw, maxw),
                       denormalize(target[2], minw, maxw),
                       denormalize(target[3], minw, maxw))
        
    def execute(self, userdata):
        
        robot_state = clone_robot_world_state(userdata.state_in)
        robot_state.number_of_transitions += 1
        
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            userdata.state_out = robot_state
            return "timeout"
        
        if not robot_state.obj1.initialized:
            userdata.state_out = robot_state
            return "failure"
        if not robot_state.obj2.initialized:
            userdata.state_out = robot_state
            return "failure"
        if random.random() > success_prob:
            userdata.state_out = robot_state
            return "failure"
        
        if self.which_obj > 0.5:
            target_pos = robot_state.obj1.pos
            check_collision = robot_state.obj1
        else:
            target_pos = robot_state.obj2.pos
            check_collision = robot_state.obj2
        
        if target_pos is None:
            return "failure"
        
        newpos = (self.weights[0] * target_pos[0],
                  self.weights[1] * target_pos[1],
                  self.weights[2] * target_pos[2])
        
        if point_inside(check_collision, newpos):
            userdata.state_out = robot_state
            return "failure"
        
        robot_state.gripper_pos = newpos

        in_gripper = object_in_gripper(robot_state)        
        
        if in_gripper is not None:
            in_gripper.pos = newpos
            calculate_boundaries(in_gripper)

        
        userdata.state_out = robot_state
        return "success"

class GraspObject(smach.State):
    
    nparams = 1
    
    def __init__(self, target):
        smach.State.__init__(self,
                             outcomes=['success', 'failure',"timeout"],
                             input_keys=['state_in'],
                             output_keys=['state_out'])
        self.target = target[0]

    def execute(self, userdata):
        
        robot_state = clone_robot_world_state(userdata.state_in)
        robot_state.number_of_transitions += 1
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            userdata.state_out = robot_state
            return "timeout"
        
        if not robot_state.obj1.initialized:
            userdata.state_out = robot_state
            return "failure"
        if not robot_state.obj2.initialized:
            userdata.state_out = robot_state
            return "failure"
        if robot_state.gripper_closed:      
            userdata.state_out = robot_state      
            return "failure"
        if robot_state.object_in_gripper:
            userdata.state_out = robot_state
            return "failure"
        if random.random() > success_prob:
            userdata.state_out = robot_state
            return "failure"
        
        if self.target > 0.5:
            robot_state.object_in_gripper = 1
        else:
            robot_state.object_in_gripper = 0
        
        in_gripper = object_in_gripper(robot_state)       
        robot_state.gripper_closed = True
        robot_state.gripper_pos = in_gripper.pos        

        userdata.state_out = robot_state        
        return "success"        

class OpenGripper(smach.State):
    
    nparams = 0
    
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', "timeout"],
                             input_keys=['state_in'],
                             output_keys=['state_out'])
        

    def execute(self, userdata):
        robot_state = clone_robot_world_state(userdata.state_in)
        
        robot_state.number_of_transitions += 1
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            userdata.state_out = robot_state
            return "timeout"
        
        robot_state.gripper_closed = False
        in_gripper = object_in_gripper(robot_state)
        
        if in_gripper is not None:
            in_gripper.pos = (in_gripper.pos[0],
                              in_gripper.pos[1],
                              table_pos_z) 
            
            calculate_boundaries(in_gripper)
                
        userdata.state_out = robot_state
        return "success"


class CheckSuccess(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['success', 'failure', "timeout"],
                             input_keys = ['state_in'],
                             output_keys = ['state_out'])
        
    def execute(self, userdata):
        
        robot_state = clone_robot_world_state(userdata.state_in)
        
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            userdata.state_out = robot_state
            return "timeout"        
        robot_state.number_of_transitions += 1
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            userdata.state_out = robot_state
            return "timeout"
        
        if not robot_state.obj1.initialized:
            userdata.state_out = robot_state
            return "failure"
        if not robot_state.obj2.initialized:
            userdata.state_out = robot_state
            return "failure"
        if objects_collide(robot_state.obj2, robot_state.obj1):
            userdata.state_out = robot_state
            return "success"
        else:
            userdata.state_out = robot_state
            return "failure"
            
names_mapping = ["DetectObjects",
                 "MoveGripperOver",
                 "GraspObject",
                 "CheckSuccess",
                 "OpenGripper",
                 ]
classes_mapping = {"DetectObjects" : DetectObjects,
                 "MoveGripperOver": MoveGripperTo,
                 "GraspObject": GraspObject,
                 "CheckSuccess": CheckSuccess,
                 "OpenGripper" : OpenGripper,
                 }

transitions_mapping = {'DetectObjects':["success",],
                       'MoveGripperOver':["success", "failure",],
                       'GraspObject':["success", "failure",],
                       "CheckSuccess":[],
                       "OpenGripper":["success"],
                       }

default_mapping = {'state_in':'robot_state',
                   'state_out':'robot_state',
                   }
data_mapping = {'DetectObjects': default_mapping,
                'MoveGripperOver': default_mapping,
                'GraspObject' :default_mapping,
                "CheckSuccess": default_mapping,
                "OpenGripper": default_mapping,
                }
 
node_degrees = [1,2,2,0,1]
nodes_params = [0,4,1,0,0]
