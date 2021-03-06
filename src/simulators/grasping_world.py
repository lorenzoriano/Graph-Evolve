import math
import random
import operator
import functools
import itertools
import copy

class Object(object):
    def __init__(self, 
                 max_x = None, 
                 min_x = None, 
                 max_y = None, 
                 min_y = None,
                 max_z = None,
                 min_z = None,
                 pos = None, 
                 dims = None):
        if pos is not None and dims is not None:
            self.starting_pos = pos
            self.__pos = pos
            self.width = dims[0]
            self.length = dims[1]
            self.height = dims[2]
            self.calculate_boundaries()
            self.initialized = True
            return

        self.starting_pos = (random.uniform(min_x, max_x),
                             random.uniform(min_y, max_y),
                             random.uniform(min_z, max_z))
        
        self.__pos = (None, None, None)
        self.height = None
        self.width = None
        self.length = None
        
        self.x_boundaries = None
        self.y_boundaries = None
        self.z_boundaries = None
        self.initialized = False
    
    def get_pos(self):
        return self.__pos
    
    def set_pos(self, newpos):
        self.__pos = newpos
        self.calculate_boundaries()
    
    pos = property(get_pos, set_pos)
    
    def calculate_boundaries(self):
        self.x_boundaries = (self.pos[0] - self.width/2.,
                             self.pos[0] + self.width/2)
                             
        self.y_boundaries = (self.pos[1] - self.length/2.,
                             self.pos[1] + self.length/2)
        
        self.z_boundaries = (self.pos[2], self.pos[2] + self.height)
    
    def initialize(self,                   
                   max_w = 0.03,
                   min_w = 0.03,
                   max_l = 0.03,
                   min_l = 0.03,
                   max_h = 0.02,
                   min_h = 0.01,):
        self.initialized = True        
    
        self.__pos = self.starting_pos
    
        self.height = random.uniform(min_h, max_h)
        self.width = random.uniform(min_w, max_w)
        self.length = random.uniform(min_l, max_l)
        
        self.calculate_boundaries()

    def point_inside(self, point):
    
        inside_x = self.x_boundaries[0] <= point[0] <= self.x_boundaries[1]
        inside_y = self.y_boundaries[0] <= point[1] <= self.y_boundaries[1]
        inside_z = self.z_boundaries[0] <= point[2] <= self.z_boundaries[1]
        
        return inside_x and inside_y and inside_z

    def dist(self, obj):
        return math.sqrt( (self.pos[0] - obj.pos[0])**2 + 
                          (self.pos[1] - obj.pos[1])**2
                        )
    
    def create_roto_translation(self, trans, rot):
        newobj = copy.copy(self)
        newobj.pos = (self.pos[0] - trans[0],
                      self.pos[1] - trans[1],
                      self.pos[2] - trans[2])
        newobj.pos = (newobj.pos[0]*math.cos(rot) + newobj.pos[1]*math.sin(rot),
                      -newobj.pos[0]*math.sin(rot) + newobj.pos[1]*math.cos(rot),
                      newobj.pos[2])
        newobj.calculate_boundaries()
        return newobj

class Robot(Object):
    def __init__(self, 
                 max_x = None, 
                 min_x = None, 
                 max_y = None, 
                 min_y = None,
                 max_th = math.pi,
                 min_th = -math.pi,
                 pos = None,
                 th = None,
                 ):
        
        super(Robot, self).__init__(max_x,
                                    min_x,
                                    max_y,
                                    min_y,
                                    0,
                                    0,
                                    pos,
                                    (0.5, 0.5, 1.5),
                                    )
        if th is not None:
            self.th = th
        else:
            self.starting_th = random.uniform(max_th, min_th)
            
        self._torso_height = random.uniform(0.012, 0.3)        
    
    def get_torso(self):
        return self._torso_height
    def set_torso(self, value):
        if value < 0.012:
            value = 0.012
        elif value > 0.3:
            value = 0.3
        self._torso_height = value
    
    torso_height =  property(get_torso, set_torso)
    
    def initialize(self,
                   max_w = 0.5,
                   min_w = 0.5,
                   max_l = 0.5,
                   min_l = 0.5,
                   max_h = 1.5,
                   min_h = 1.5,):
        super(Robot, self).initialize(max_w,
                                      min_w,
                                      max_l,
                                      min_l,
                                      max_h,
                                      min_h)
    
        self.th = self.starting_th
    
        self.height = 1.5
        self.width = 0.5
        self.length = 0.5
        
        self.calculate_boundaries()
    
    def sees(self, obj):
        
        new_x = obj.pos[0] - self.pos[0]
        new_y = obj.pos[1] - self.pos[1]
        
        newth = math.atan2(new_y, new_x) - self.th
        
        if newth > math.pi:
            newth -= 2.*math.pi 
        if newth < -math.pi:
            newth += 2.*math.pi
        angle_vis = -math.pi/4. <= newth <= math.pi/4.
        
        
        
        dist = math.sqrt( (self.pos[0] - obj.pos[0])**2 + 
                          (self.pos[1] - obj.pos[1])**2
                        )
        
        return angle_vis and (dist < 1.5)

    def sees_any(self, obj_list):
        
        saw_list = (self.sees(o) for o in obj_list)
        for seen in saw_list:
            if seen:
                return True
        return False

    def can_grasp(self, obj, net, normalizer):
        if not self.sees(obj):
            return False
        
        obj = obj.create_roto_translation(self.pos, self.th)
        netinput = obj.pos + (self.torso_height,)
        netinput = normalizer.normalize(netinput)
        netout = net.safe_output(netinput, default_class = 0, thr = 0.5)
        ret = netout[0] != 0
        return ret[0]
        

class GraspingWorld(object):
    def __init__(self,
                 max_x = 5.0,
                 min_x = 0,
                 max_y = 1.5,
                 min_y = -1.5,
                 max_z = 0,
                 min_z = 2.0):
                
        self.robot = None
        self.table = None
        self.objects = []
        
#        self.min_x = min_x
#        self.max_x = max_x
        self.min_x = 0
        self.max_x = random.uniform(3.3, 5)
        
#        self.min_y = min_y 
#        self.max_y = max_y

        length = random.uniform(3.3, 5.)
        self.min_y = -length/2.
        self.max_y = length/2.
        
        self.max_z = max_z
        self.min_z = min_z
        self.time_step = 0
        
        self.robot_positions = []
    
    def inc_time(self):
        self.time_step +=  1

    def create_fixed_table(self, pos):
        self.table = Object(max_x = pos[0],
                            min_x = pos[0],
                            max_y = pos[1],
                            min_y = pos[1],
                            max_z = 0,
                            min_z = 0)
        
        width = (0.3, 1.2)
        height = (0.3, 1.0)
        length = (0.3,  1.2)
        
        self.table.initialize(width[1], 
                              width[0], 
                              length[1],
                              length[0], 
                              height[1],
                              height[0])

    def create_table(self):
        self.table = Object(max_x = self.max_x - 1.1,
                            min_x = self.min_x + 1.5,
                            max_y = self.max_y - 1.1,
                            min_y = self.min_y + 1.1,
                            max_z = 0,
                            min_z = 0)
        
        width = (0.3, 1.2)
        height = (0.70, 0.80)
        length = (0.3,  1.2)
        
        self.table.initialize(width[1], 
                              width[0], 
                              length[1],
                              length[0], 
                              height[1],
                              height[0])
    
    def create_object_on_table(self):
        obj = Object(max_x = self.table.x_boundaries[1],
                     min_x = self.table.x_boundaries[0],
                     max_y = self.table.y_boundaries[1],
                     min_y = self.table.y_boundaries[0],
                     max_z = self.table.height,
                     min_z = self.table.height
                     )
        obj.initialize(max_w = 0.03,
                       min_w = 0.03,
                       max_l = 0.03,
                       min_l = 0.03,
                       max_h = 0.02,
                       min_h = 0.01,
                       )
        self.objects.append(obj)
        
    def create_robot(self, require_not_visible = True):        
        
        while True:
            self.robot = Robot(max_x = self.max_x,
                               min_x = self.min_x,
                               max_y = self.max_y,
                               min_y = self.min_y)
            self.robot.initialize()            
            
            if not (self.objects_collide(self.robot, self.table) or
                    (self.robot.sees_any(self.objects) and require_not_visible)):
                break
        
        self.robot_positions = [self.robot.pos]
    def create_with_fixed_robot(self):
        
        self.robot = Robot( pos = (0., 0., 0.), th = 0.)
        self.robot_positions = [self.robot.pos]
        
        while True:            
            self.create_table()
            self.create_object_on_table()
            if not (self.robot.sees_any(self.objects) or
                    self.robot.sees(self.table)):
                break
            
            self.table = None
            self.objects.pop()
    
    def move_robot(self, pos):
        
        self.robot_positions.append(pos)
        x,y,th = pos
        
        if not (self.min_x <= x <= self.max_x):
            return False
        if not (self.min_y <= y <= self.max_y):
            return False
        
        newrobot = copy.copy(self.robot)
        newrobot.pos = (x, y, newrobot.pos[2])
        newrobot.th = th
        
        if self.objects_collide(newrobot, self.table):
            return False
        for o in self.objects:
            if self.objects_collide(newrobot, o):
                return False 
         
        self.robot = newrobot
        return True
    
    def robot_sees_objects(self):
        return self.robot.sees_any(self.objects)
    
    @staticmethod
    def __objects_collide(obj1, obj2):
        '''
        True if at least one of the vertex of ob1 is inside the volume of obj2
        '''
        
        x_bnd = (obj1.x_boundaries[1] >= obj2.x_boundaries[0] and
                 obj1.x_boundaries[0] <= obj2.x_boundaries[1])
        
        y_bnd = (obj1.y_boundaries[1] >= obj2.y_boundaries[0] and
                 obj1.y_boundaries[0] <= obj2.y_boundaries[1] )
        
        z_bnd = (obj1.z_boundaries[1] >= obj2.z_boundaries[0] and
                 obj1.z_boundaries[0] <= obj2.z_boundaries[1])
        
        return ( x_bnd and y_bnd and z_bnd)       
    
    @staticmethod
    def objects_collide(obj1, obj2):
        return (GraspingWorld.__objects_collide(obj1, obj2) or 
                GraspingWorld.__objects_collide(obj2, obj1))
    
    
    def robot_collides(self):        
        all_items = itertools.chain( (self.table,),
                                     (o for o in self.objects)
                                    )
        for item in all_items:
            if self.objects_collide(self.robot, item):
                return True
        
        return False

def visibility_stats(num_trials = 1000):
   
    num_visibles = 0
    
    for _ in xrange(num_trials):
        world = GraspingWorld()
        world.create_table()
        world.create_object_on_table()
        world.create_robot(require_not_visible=False)
        
        if world.robot.sees_any(world.objects):
            num_visibles += 1
    
    return float(num_visibles) / float(num_trials)

def graspability_stats(net, normalizer, num_trials = 1000):
   
    num_visibles = 0
    
    for _ in xrange(num_trials):
        world = GraspingWorld()
        world.create_table()
        world.create_object_on_table()
        world.create_robot(require_not_visible=False)
        world.robot.torso_height = 0.176
        
        if world.robot.can_grasp(world.objects[0], net, normalizer):
            num_visibles += 1
            print "ROBOT: ", world.robot.pos, " th: ", world.robot.th
            print "Object: ", world.objects[0].pos
    
    return float(num_visibles) / float(num_trials)

def visibility_wheel(num_trials=1000):
    
    num_visible = 0
    robot_th = random.uniform(-math.pi, math.pi)
    for _ in xrange(num_trials):
        angle = random.uniform(-math.pi, math.pi)
        radius = random.uniform(0, 1.5)
        obj_pos = (radius*math.cos(angle), radius*math.sin(angle), 0)
        obj_dims = (0.1, 0.1, 0.1)
        
        robot_pos = (0,0,0)
#        robot_th = 0
        obj = Object(pos = obj_pos, dims = obj_dims)
        robot = Robot(pos = robot_pos, th = robot_th)
        
        if robot.sees(obj):
            num_visible += 1
    
    return float(num_visible) / float(num_trials)

def return_visible_points(num_trials = 1000, robot_th = None):    
    
    if robot_th is None:
        robot_th = random.uniform(-math.pi, math.pi)
    
    visibles = []
    invisibles = []
    
    for _ in xrange(num_trials):
        angle = random.uniform(-math.pi, math.pi)
        radius = random.uniform(0, 1.5)
#        radius = 1.0
        obj_pos = (radius*math.cos(angle), radius*math.sin(angle), 0)
        obj_dims = (0.1, 0.1, 0.1)
        
        robot_pos = (0,0,0)        
        obj = Object(pos = obj_pos, dims = obj_dims)
        robot = Robot(pos = robot_pos, th = robot_th)
        
        if robot.sees(obj):
            visibles.append(( obj_pos[0], obj_pos[1] ))
        else:
            invisibles.append(( obj_pos[0], obj_pos[1] ))
    
    print "robot angle is: ", robot_th
    print "rations is: ", float(len(visibles)) / float(num_trials)
    return visibles, invisibles
            