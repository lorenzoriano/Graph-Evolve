import unittest
import smach
from simulators import grasping_world
import math
import random
import smach_explore

from grasping_world import (Object, 
                            Robot, 
                            GraspingWorld, 
                            visibility_stats,
                            visibility_wheel) 

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
        
    def test_fixed_robot(self):
        for _ in xrange(1000):
            world = GraspingWorld()
            world.create_with_fixed_robot()
            
            self.assertFalse(world.robot.sees(world.table))
            self.assertFalse(world.robot_sees_objects())
    
    def test_robot_move_safe(self):
        world = GraspingWorld()
        world.create_with_fixed_robot()
        
        x,y,_ = world.robot.pos        
        th = world.robot.th
        
        x += 0.1
        y += 0.1
        
        res = world.move_robot((x, y, th))
        self.assertTrue(res)
        
        self.assertEqual(world.robot.pos[0], x)
        self.assertEqual(world.robot.pos[1], y)
        
    
    def test_robot_cant_move(self):
        world = GraspingWorld()
        world.create_with_fixed_robot()
        initial_pos = world.robot.pos
        
        x,y,_ = world.table.pos
        th = world.robot.th
        
        res = world.move_robot((x, y, th))
        self.assertFalse(res)
        self.assertEqual(world.robot.pos, initial_pos)
        
        x,y,_ = (5.1,0,0)
        th = world.robot.th
        
        res = world.move_robot((x, y, th))
        self.assertFalse(res)
        self.assertEqual(world.robot.pos, initial_pos)
    
        x,y,_ = (-5.1,0,0)
        th = world.robot.th
        
        res = world.move_robot((x, y, th))
        self.assertFalse(res)
        self.assertEqual(world.robot.pos, initial_pos)
        
        x,y,_ = (1.3,-5.2,0)
        th = world.robot.th
        
        res = world.move_robot((x, y, th))
        self.assertFalse(res)
        self.assertEqual(world.robot.pos, initial_pos)
        
        x,y,_ = (0.5,5.1,0)
        th = world.robot.th
        
        res = world.move_robot((x, y, th))
        self.assertFalse(res)
        self.assertEqual(world.robot.pos, initial_pos)
    
    def test_print_fixed_robot(self):
        world = GraspingWorld()
        world.create_with_fixed_robot()
        
        print
        print "Table pos: ", world.table.pos
        print "Table dims: ", (world.table.width, 
                               world.table.length, 
                               world.table.height)
        
        print "Object pos: ", world.objects[0].pos
        print "Object dims: ", (world.objects[0].width, 
                                world.objects[0].length, 
                                world.objects[0].height)
        
        print "Robot pos: ", world.robot.pos
        print "Robot theta: ", world.robot.th
    
    def test_visibility_stats(self):        
        print "Visibility stats: ", visibility_stats(1000)
    
    def test_visibility_wheel(self):
        print "Visibility wheel: ", visibility_wheel(1000)



def null_print(_): pass
smach.set_loggers(smach.loginfo,
                  null_print,
                  null_print,
                  null_print)

class StateMachineTests(unittest.TestCase):
    def test_simple(self):
        sm = smach.StateMachine(outcomes=["success", 
                                          "failure", 
                                          "timeout"])
        world = GraspingWorld()
        world.create_with_fixed_robot()
        params = [2*random.random()-1.0 for _ in  xrange(smach_explore.params_size)]
        with sm:
            smach.StateMachine.add('N1', smach_explore.NeuralNetwork(params,world),
                                   transitions={'success' : 'N2',
                                                "timeout": "timeout"},
                                   remapping = {'network_in' : 'net_value',
                                                'network_out' :  'net_value'}
                                   )
            smach.StateMachine.add('N2', smach_explore.NeuralNetwork(params,world),
                                   transitions={'success' : 'Mover',
                                                "timeout": "timeout"},
                                   remapping = {'network_in' : 'net_value',
                                                'network_out' :  'net_value'}
                                   )
            smach.StateMachine.add('Mover', smach_explore.RobotMove(world),
                                   transitions={'success' : 'CheckSuccess',
                                                "timeout":  "timeout",
                                                "failure" : "N1"},
                                   remapping = {'move_pos' : 'net_value',}
                                   )
            smach.StateMachine.add('CheckSuccess', smach_explore.CheckSuccess(world),
                                   transitions={'success' : 'success',
                                                "timeout":  "timeout",
                                                "failure" : "failure"},                                   
                                   )
            
        outcome = sm.execute()
        print "UserData: ", sm.userdata.net_value
        print "RobotState: ", world.robot.pos, " ", world.robot.th
            
            
if __name__ == "__main__":
    unittest.main()

