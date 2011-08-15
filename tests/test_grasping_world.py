import unittest
import smach
from simulators import grasping_world
import math

from grasping_world import (Object, 
                            Robot, 
                            GraspingWorld, 
                            visibility_stats,
                            visibility_wheel) 

#smach.set_loggers(smach.loginfo,
#                  smach_stack.null_print,
#                  smach_stack.null_print,
#                  smach_stack.null_print)

class BasicTests(unittest.TestCase):
    def test_in_collision(self):
        pos1 = (0,0,0)
        dims1 = (1.0, 1.0, 1.0,)
        obj1 = Object(pos = pos1, dims = dims1)
        
        pos2 = (0.1,0,0)
        dims2 = (0.1, 0.1, 0.5)
        obj2 = Object(pos = pos2, dims = dims2)        
        self.assertTrue(GraspingWorld.objects_collide(obj1, obj2))
        
        pos2 = (0.1,0.5,0)
        dims2 = (0.1, 0.1, 0.5)
        obj2 = Object(pos = pos2, dims = dims2)
        self.assertTrue(GraspingWorld.objects_collide(obj1, obj2))
        
        pos2 = (0.1,0,0.3)
        dims2 = (0.1, 0.1, 0.5)
        obj2 = Object(pos = pos2, dims = dims2)
        self.assertTrue(GraspingWorld.objects_collide(obj1, obj2))
        
        pos2 = (-0.1,0,0)
        dims2 = (0.1, 0.1, 0.5)
        obj2 = Object(pos = pos2, dims = dims2)
        self.assertTrue(GraspingWorld.objects_collide(obj1, obj2))
        
        pos1 = (0.33524283935165333, -0.059357905570512348, 0.5)
        dims1 =  (0.193690329274, 0.193690329274, 0.101006366107)
        obj2 = Object(pos = pos1, dims = dims1)
        pos2 = (0.33524283935165333, -0.059357905570512348, 0.5)
        dims2 = (0.108117871605, 0.108117871605, 0.102062692143)
        obj2 = Object(pos = pos2, dims = dims2)
        self.assertTrue(GraspingWorld.objects_collide(obj1, obj2))
    
    
    
    def test_notin_collision(self):
        pos1 = (0,0,0)
        dims1 = (1.0, 1.0, 1.0)
        obj1 = Object(pos = pos1, dims = dims1)
        
        pos2 = (0.6,0,0)
        dims2 = (0.1, 0.1, 0.1)
        obj2 = Object(pos = pos2, dims = dims2)
        self.assertFalse(GraspingWorld.objects_collide(obj1, obj2))
        
        pos2 = (0.0,0.6,0)
        dims2 = (0.1, 0.1, 0.1)
        obj2 = Object(pos = pos2, dims = dims2)
        self.assertFalse(GraspingWorld.objects_collide(obj1, obj2))
        
        pos2 = (0.0,0,1.1)
        dims2 = (0.1, 0.1, 0.1)
        obj2 = Object(pos = pos2, dims = dims2)
        self.assertFalse(GraspingWorld.objects_collide(obj1, obj2))
        
        pos2 = (-0.6,0,0)
        dims2 = (0.1, 0.1, 0.1)
        obj2 = Object(pos = pos2, dims = dims2)
        self.assertFalse(GraspingWorld.objects_collide(obj1, obj2))
    
    def test_initialize(self):
        
        max_x = 1.0
        min_x = 0
        max_y = 1.0
        min_y = 0.0
        max_z = 1.0
        min_z = 0.0
        
        obj1 = Object(max_x = max_x,
                      min_x = min_x,
                      max_y = max_y,
                      min_y = min_y,
                      max_z = max_z,
                      min_z = min_z)
        obj1.initialize()
         
        self.assertNotEqual(obj1.pos[0], None )
        self.assertTrue(obj1.initialized)
        
    def test_object_on_table_collide(self):
        world = GraspingWorld()
        
        world.create_table()
        world.create_object_on_table()
        
        self.assertTrue(world.objects_collide(world.table, 
                                              world.objects[0]))
    
    def test_object_on_table_inside(self):
        
        for _ in xrange(100):
            world = GraspingWorld()
            
            world.create_table()
            world.create_object_on_table()
                    
            self.assertTrue(world.table.point_inside(world.objects[0].pos))
    
    def test_robot_creation(self):
        
        for _ in xrange(100):
            world = GraspingWorld()
            
            world.create_table()
            world.create_object_on_table()
            world.create_robot()
            
            self.assertFalse(world.objects_collide(world.robot, world.table))
            self.assertFalse(world.robot.sees(world.objects[0]))
            self.assertFalse(world.robot_collides())
        
    def test_object_visible_1(self):
        obj_pos = (0.5, 1., 0)
        obj_dims = (0.1, 0.1, 0.1)
        
        robot_pos = (0,0,0)
        robot_th = math.pi/2.
        
        obj = Object(pos = obj_pos, dims = obj_dims)
        robot = Robot(pos = robot_pos, th = robot_th)
        
        self.assertTrue(robot.sees(obj))
    
    def test_object_visible_2(self):
        obj_pos = (-0.5, 1., 0)
        obj_dims = (0.1, 0.1, 0.1)
        
        robot_pos = (0,0,0)
        robot_th = math.pi/2.
        
        obj = Object(pos = obj_pos, dims = obj_dims)
        robot = Robot(pos = robot_pos, th = robot_th)
        
        self.assertTrue(robot.sees(obj))    
    
    def test_object_visible_3(self):
        obj_pos = (1.2*math.cos(math.pi/4), 1.2*math.sin(math.pi/4), 0)
        obj_dims = (0.1, 0.1, 0.1)
        
        robot_pos = (0,0,0)
        robot_th = math.pi/2.
        
        obj = Object(pos = obj_pos, dims = obj_dims)
        robot = Robot(pos = robot_pos, th = robot_th)
        
        self.assertTrue(robot.sees(obj))

    def test_object_visible_4(self):
        angle = math.radians(170)
        obj_pos = (math.cos(angle), math.sin(angle), 0)
        obj_dims = (0.1, 0.1, 0.1)
        
        robot_pos = (0,0,0)
        robot_th = -math.pi        
        obj = Object(pos = obj_pos, dims = obj_dims)
        robot = Robot(pos = robot_pos, th = robot_th)
        
        self.assertTrue(robot.sees(obj))

    def test_object_visible(self):
        obj_pos = (1.2*math.cos(math.pi/4), 1.2*math.sin(math.pi/4), 0)
        obj_dims = (0.1, 0.1, 0.1)
        
        robot_pos = (0,0,0)
        robot_th = math.pi/2.
        
        obj = Object(pos = obj_pos, dims = obj_dims)
        robot = Robot(pos = robot_pos, th = robot_th)
        
        self.assertTrue(robot.sees(obj))
    
    def test_object_not_visible_2(self):       
        obj_pos = (1.,3.,0)
        obj_dims = (0.1, 0.1, 0.1)
        
        robot_pos = (3,2,0)
        robot_th = math.pi/2.
        
        obj = Object(pos = obj_pos, dims = obj_dims)
        robot = Robot(pos = robot_pos, th = robot_th)
        
        self.assertFalse(robot.sees(obj))
    
    def test_visibility_stats(self):
        
        print "Visibility stats: ", visibility_stats(1000)
    
    def test_visibility_wheel(self):
        print "Visibility wheel: ", visibility_wheel(1000)
        
if __name__ == "__main__":
    unittest.main()

