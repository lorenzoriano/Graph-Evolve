import math
import copy
import numpy as np


class Object(object):
    def __init__(self, pos, dims):
        self.starting_pos = pos
        self.__pos = pos
        self.width = dims[0]
        self.length = dims[1]
        self.height = dims[2]
        self.calculate_boundaries()
        self.initialized = True

    def __repr__(self):
        return "pos: %s, dims: %s, boundaries %s" % (self.__pos,
			(self.width, self.length, self.height),
            (self.x_boundaries, self.y_boundaries, self.z_boundaries)
			)

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
    def __init__(self, pos,
                 ):
        
        super(Robot, self).__init__(pos, (0.5, 0.5, 1.5))
            
class GraspingWorld(object):
    def __init__(self,
                object_transformation,
                table_transformation,
                grasping_tester):
                
        self.table = None
        self.table_vec= None
        self.obj = None
        self.obj_vec = None
        self.time_step = 0
        self.robot_positions = []
        self.object_transformation = object_transformation
        self.table_transformation = table_transformation
        self.grasping_tester = grasping_tester
  
    def reset(self):
        self.table = None
        self.table_vec= None
        self.obj = None
        self.obj_vec = None
        self.time_step = 0
        self.robot_positions = []


    def inc_time(self):
        self.time_step +=  1

    def __create_table(self, pos, dims):
        self.table = Object(pos, dims)

    def create_table(self, dims):
    	x_min, y_min, x_max, y_max = dims
        self.table_vec = np.array(dims)
        x = x_min + (x_max - x_min)/2.
        y = y_min + (y_max - y_min)/2.
        z = 0.76

        width = x_max - x_min
        length = y_max - y_min
        height = 0.0 #very thin table

        self.__create_table( (x,y,z), (width, length, height))

    def create_object(self, pos):
        self.obj_vec = np.array(pos)
        obj = Object( pos, (0.1,0.1,0.1))
        self.obj = obj
    
    def enlarged_table(self):
        newtable = Object( self.table.pos, (self.table.width + 0.5,
                                            self.table.length + 0.5,
                                            self.table.height + 1.5,
                                           )
                         )
        return newtable

    def move_robot(self, movement):

        if self.line_intersect_obj((0,0,0), movement,
                                    self.enlarged_table()):
            return False

        #robot = Robot(movement)
        #if self.objects_collide(robot, self.table):
        #    return False

        self.robot_positions.append(movement)
        inpt_vec_table = np.hstack( (self.table_vec, movement) )
        newtable = self.table_transformation.predict( inpt_vec_table).ravel()
        self.create_table((newtable[0], newtable[1], 
                                    newtable[2], newtable[3]))

        inpt_vec_obj = np.hstack( (self.obj_vec, movement) )
        newobj = self.object_transformation.predict(inpt_vec_obj).ravel()
        self.create_object( (newobj[0], newobj[1], newobj[2]) )
        
        return True

    def net_input(self):
        if self.obj_vec is None:
            raise ValueError("Object is none!")
        elif self.table_vec is None:
            raise ValueError("Table is none!")

         #I HATE MYSELF FOR THIS!
        svm_input = (
         self.obj_vec[0],
         self.obj_vec[1],
         self.obj_vec[2],
         self.table_vec[0],
         self.table_vec[2],
         self.table_vec[1],
         self.table_vec[3],		     
        )
        return np.array(svm_input)
       #return np.hstack((self.obj_vec, self.table_vec))

    def can_grasp(self):
        #TODO add robot sees object
        svm_input = self.net_input()

        svm_out = self.grasping_tester.predict(svm_input)
        return svm_out[0] == 1.0

    def grasp_probability(self):
        #TODO add robot sees object
        svm_input = self.net_input()

        probs = self.grasping_tester.predict_proba(svm_input)[0]

        #class 1
        return probs[1]

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


    @staticmethod
    def line_intersect_obj(l_start, l_end, obj):
        def are_intervals_intersecting(a0, a1, b0, b1):
            if (a1 < a0):
                a1, a0 = a0, a1
    
            if (b1 < b0):
                b1, b0 = b0, b1

            if b0 < a0:
                return a0 <= b1
            else:
                return b0 <= a1

        if not are_intervals_intersecting(l_start[0], l_end[0], 
                obj.x_boundaries[0], obj.x_boundaries[1]):
            return False
        if not are_intervals_intersecting(l_start[1], l_end[1], 
                obj.y_boundaries[0], obj.y_boundaries[1]):
            return False
        return True
        


    
