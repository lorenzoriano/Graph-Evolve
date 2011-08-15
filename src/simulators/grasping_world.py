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
            self.pos = pos
            self.width = dims[0]
            self.length = dims[1]
            self.height = dims[2]
            self.calculate_boundaries()
            self.initialized = True
            return

        self.starting_pos = (random.uniform(min_x, max_x),
                             random.uniform(min_y, max_y),
                             random.uniform(min_z, max_z))
        
        self.pos = (None, None, None)
        self.height = None
        self.width = None
        self.length = None
        
        self.x_boundaries = None
        self.y_boundaries = None
        self.z_boundaries = None
        self.initialized = False
    
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
    
        self.pos = self.starting_pos
    
        self.height = random.uniform(min_h, max_h)
        self.width = random.uniform(min_w, max_w)
        self.length = random.uniform(min_l, max_l)
        
        self.calculate_boundaries()

    def point_inside(self, point):
    
        inside_x = self.x_boundaries[0] <= point[0] <= self.x_boundaries[1]
        inside_y = self.y_boundaries[0] <= point[1] <= self.y_boundaries[1]
        inside_z = self.z_boundaries[0] <= point[2] <= self.z_boundaries[1]
        
        return inside_x and inside_y and inside_z

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
            return
        
        self.starting_th = random.uniform(max_th, min_th)
    
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

class GraspingWorld(object):
    def __init__(self,
                 max_x = 5.0,
                 min_x = 0,
                 max_y = 5.0,
                 min_y = 0,
                 max_z = 0,
                 min_z = 2.0):
                
        self.robot = None
        self.table = None
        self.objects = []
        
        self.min_x = min_x
        self.max_x = max_x
        
        self.min_y = min_y 
        self.max_y = max_y
        
        self.max_z = max_z
        self.min_z = min_z

    def create_table(self):
        self.table = Object(max_x = self.max_x,
                            min_x = self.min_x,
                            max_y = self.max_y,
                            min_y = self.min_y,
                            max_z = 0,
                            min_z = 0)
        
        width = (0.3, 1.2)
        height = (0.3, 1.0)
        length = (0.2,  1.5)
        
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
            