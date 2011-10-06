import sys
import os
if os.name == "posix":
    sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src')
    sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src/pyevolve')
    sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src/rbfnetwork')
else:
    sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src')
    sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src\\pyevolve')
from graph_evolve import smach_explore_grasp
import graph_evolve
from graph_evolve.chromosome_smach import convert_chromosome_smach

from pyevolve import GSimpleGA
from pyevolve import G1DList
from pyevolve import Selectors
from pyevolve import Initializators, Mutators,  Consts
from pyevolve import DBAdapters
from pyevolve import Crossovers
from pyevolve import Consts

import time

import unittest
import smach
from simulators import grasping_world
import math
import random
from graph_evolve import  smach_explore
from graph_evolve import chromosome_smach
from graph_evolve import graph_genome
import pyrecurrentnet
import rbfnetwork
import cPickle

from simulators.grasping_world import (Object, 
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
    
    def test_change_frame(self):
        obj_pos = (4.0, 3., 0)
        obj_dims = (0.1, 0.1, 0.1)
        
        robot_pos = (2,2,0)
        robot_th = math.pi/4.
        
        obj = Object(pos = obj_pos, dims = obj_dims)
        robot = Robot(pos = robot_pos, th = robot_th)
        
        newobj = obj.create_roto_translation(robot.pos, robot.th)
        print "\nObject pos: %s" % str(newobj.pos)
    
    def test_angles(self):
        
        trials = 1000
        for _ in xrange(trials):
            world = GraspingWorld()
            
            world.create_table()
            world.create_object_on_table()
            world.create_robot(require_not_visible = False)
            
            robot = world.robot
            obj = world.objects[0]
            
            new_x = obj.pos[0] - robot.pos[0]
            new_y = obj.pos[1] - robot.pos[1]
            
            newth = math.atan2(new_y, new_x) - robot.th
            if newth > math.pi:
                newth -= 2.*math.pi 
            if newth < -math.pi:
                newth += 2.*math.pi
            
            newobj = obj.create_roto_translation(robot.pos, robot.th)
            roto_th = math.atan2(newobj.pos[1], newobj.pos[0])
            self.assertAlmostEqual(newth, roto_th)
    
    def test_angles_visibility(self):
        
        trials = 1000
        for _ in xrange(trials):
            world = GraspingWorld()
            
            world.create_table()
            world.create_object_on_table()
            world.create_robot(require_not_visible = False)
            
            robot = world.robot
            obj = world.objects[0]
                        
            newobj = obj.create_roto_translation(robot.pos, robot.th)
            if robot.sees(obj):
                self.assertTrue(newobj.pos[0] > 0)
            
    
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
        
        x += 0.01
        y += 0.01
        
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
    
    def test_visibility_stats(self):        
        print "Visibility stats: ", visibility_stats(1000)
    
    def test_visibility_wheel(self):
        print "Visibility wheel: ", visibility_wheel(1000)

    def test_visible_after_translation(self):
        obj_pos = (1.703970159175764, -0.3550906629552664, 0.0)
        obj_dims = (0.1, 0.1, 0.1)
        
        robot_pos = (2.2530791607800742, -0.23572578723278204, 0.0)
        robot_th = 1.6455576319
        
        obj = Object(pos = obj_pos, dims = obj_dims)
        robot = Robot(pos = robot_pos, th = robot_th)
        
#        self.assertTrue(robot.sees(obj))
        obj = obj.create_roto_translation(robot.pos, robot.th)
        
        print "new object pos: ", obj.pos
        


def null_print(_): pass

#smach.set_loggers(null_print,
#                  null_print,
#                  null_print,
#                  smach.log.logerr)
smach.set_loggers(null_print,
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
        experiment = smach_explore.ExperimentSetup()
        experiment.initialize()
        params = [2*random.random()-1.0 for _ in  xrange(experiment.params_size)]
        with sm:
            smach.StateMachine.add('N1', smach_explore.NeuralNetwork(params,world,experiment),
                                   transitions={'success' : 'N2',
                                                "timeout": "timeout"},
                                   remapping = {'network_in' : 'net_value',
                                                'network_out' :  'net_value'}
                                   )
            smach.StateMachine.add('N2', smach_explore.NeuralNetwork(params,world,experiment),
                                   transitions={'success' : 'Mover',
                                                "timeout": "timeout"},
                                   remapping = {'network_in' : 'net_value',
                                                'network_out' :  'net_value'}
                                   )
            smach.StateMachine.add('Mover', smach_explore.RobotMove(world,experiment),
                                   transitions={'success' : 'ExitSuccess',
                                                "timeout":  "timeout",
                                                "failure" : "N1"},
                                   remapping = {'move_pos' : 'net_value',}
                                   )
            smach.StateMachine.add('ExitSuccess', smach_explore.ExitSuccess(world,experiment),
                                   transitions={'success' : 'success',
                                                "timeout":  "timeout",
                                                "failure" : "N1"},                                   
                                   )
            
        sm.execute()
#        print "UserData: ", sm.userdata.net_value
#        print "RobotState: ", world.robot.pos, " ", world.robot.th
    
    def test_double_execution(self):
        sm = smach.StateMachine(outcomes=["success", 
                                          "failure", 
                                          "timeout"])
        world = GraspingWorld()
        world.create_with_fixed_robot()
        experiment = smach_explore.ExperimentSetup()
        experiment.initialize()
        params = [2*random.random()-1.0 for _ in  xrange(experiment.params_size)]
        with sm:
            smach.StateMachine.add('N1', smach_explore.NeuralNetwork(params,world,experiment),
                                   transitions={'success' : 'N2',
                                                "timeout": "timeout"},
                                   remapping = {'network_in' : 'net_value',
                                                'network_out' :  'net_value'}
                                   )
            smach.StateMachine.add('N2', smach_explore.NeuralNetwork(params,world,experiment),
                                   transitions={'success' : 'Mover',
                                                "timeout": "timeout"},
                                   remapping = {'network_in' : 'net_value',
                                                'network_out' :  'net_value'}
                                   )
            smach.StateMachine.add('Mover', smach_explore.RobotMove(world,experiment),
                                   transitions={'success' : 'ExitSuccess',
                                                "timeout":  "timeout",
                                                "failure" : "N1"},
                                   remapping = {'move_pos' : 'net_value',}
                                   )
            smach.StateMachine.add('ExitSuccess', smach_explore.ExitSuccess(world,experiment),
                                   transitions={'success' : 'success',
                                                "timeout":  "timeout",
                                                "failure" : "N1"},                                   
                                   )
        
        print "First Run"    
        sm.execute()        
#        print "UserData: ", sm.userdata.net_value
#        print "RobotState: ", world.robot.pos, " ", world.robot.th
        
        print "Second Run"
        world.time_step = 0
        sm.execute()        
#        print "UserData: ", sm.userdata.net_value
#        print "RobotState: ", world.robot.pos, " ", world.robot.th
        
        
    
    def test_random_mover(self):
        sm = smach.StateMachine(outcomes=["success", 
                                          "failure", 
                                          "timeout"])
        world = GraspingWorld()
        world.create_with_fixed_robot()
        experiment = smach_explore.ExperimentSetup()
        experiment.initialize()
        nstates = 10
        with sm:
            
            for sgh in xrange(nstates-1):
                params = [random.random() for _ in xrange(6)]
            
                smach.StateMachine.add('R_'+str(sgh), smach_explore.RandomMove(params,world,experiment),
                                       transitions={'success' : 'E_'+str(sgh),
                                                    'failure' : 'E_'+str(sgh),
                                                    "timeout": "timeout"},
                                       )
                smach.StateMachine.add('E_'+str(sgh), smach_explore.ExitSuccess(world,experiment),
                                       transitions={'success' : 'success',
                                                    "timeout":  "timeout",
                                                    "failure" : "R_"+str(sgh+1)},                                   
                                   )
            
            params = [random.random() for _ in xrange(6)]
        
            smach.StateMachine.add('R_'+str(nstates-1), smach_explore.RandomMove(params,world,experiment),
                                   transitions={'success' : 'E_'+str(nstates-1),
                                                "timeout": "timeout"},
                                   )
            smach.StateMachine.add('E_'+str(nstates-1), smach_explore.ExitSuccess(world,experiment),
                                   transitions={'success' : 'success',
                                                "timeout":  "timeout",
                                                "failure" : "failure"},                                   
                               )
            
        outcome = sm.execute()
        print "Random mover outcome: ", outcome
    
    def test_factory(self):
        experiment = smach_explore.ExperimentSetup()
        experiment.initialize()
        factory = smach_explore.ExplorerFactory(experiment)
        world = factory.world
        genome = graph_genome.GraphGenome(50, factory.node_degrees,
                                          factory.node_params)
        genome.initialize()
        
        sm = chromosome_smach.convert_chromosome_smach(genome, 
                                                       factory.names_mapping, 
                                                       factory.transitions_mapping, 
                                                       factory.classes_mapping, 
                                                       factory.data_mapping)
        
        sm.execute()
#        try:
#            print "UserData: ", sm.userdata.net_value
#        except KeyError:
#            pass
#        print "RobotState: ", world.robot.pos, " ", world.robot.th
    
    def test_factory2(self):
        experiment = smach_explore.ExperimentSetup()
        experiment.initialize()
        factory = smach_explore.ExplorerFactory(experiment)
        world = factory.world
        genome = graph_genome.GraphGenome(50, factory.node_degrees,
                                          factory.node_params)
        genome.initialize()
        
        sm = chromosome_smach.convert_chromosome_smach(genome, 
                                                       factory.names_mapping, 
                                                       factory.transitions_mapping, 
                                                       factory.classes_mapping, 
                                                       factory.data_mapping)
        
        sm.execute()
    
    def test_sample_genomes(self):
        
        steps = 10
        successess = 0
        
        for trial in xrange(steps):
            experiment = smach_explore.ExperimentSetup()
            experiment.initialize()
            factory = smach_explore.ExplorerFactory(experiment)
            world = factory.world
            genome = graph_genome.GraphGenome(50, factory.node_degrees,
                                              factory.node_params)
            genome.initialize()
            
            sm = chromosome_smach.convert_chromosome_smach(genome, 
                                                           factory.names_mapping, 
                                                           factory.transitions_mapping, 
                                                           factory.classes_mapping, 
                                                           factory.data_mapping)
            
            outcome = sm.execute()
            if outcome == "success":
                successess += 1
                
        print "success rate: ", float(successess) / float(steps)

    
    def test_sample_genomes2(self):
        
        steps = 10
        successess = 0
        
        for trial in xrange(steps):
            experiment = smach_explore.ExperimentSetup()
            experiment.initialize()
            factory = smach_explore.ExplorerFactory(experiment)
            world = factory.world
            genome = graph_genome.GraphGenome(50, factory.node_degrees,
                                              factory.node_params)
            genome.initialize()
            
            sm = chromosome_smach.convert_chromosome_smach(genome, 
                                                           factory.names_mapping, 
                                                           factory.transitions_mapping, 
                                                           factory.classes_mapping, 
                                                           factory.data_mapping)
            
            outcome = sm.execute()
            if outcome == "success":
                successess += 1
                
        print "success rate: ", float(successess) / float(steps)

    def test_net(self):
        network_input_size = 3
        network_output_size = 3
        network_hidden_size = 0
        network_bias_size = network_hidden_size + network_output_size
        network_total_size = network_input_size + network_hidden_size + network_output_size
        params_size = (network_total_size - network_input_size)*network_total_size + network_bias_size
        
        max_w = 3.
        min_w = -3.
        
        params = [(max_w-min_w)*random.random() +min_w for _ in xrange(params_size)]
        
        net =  pyrecurrentnet.list_convert(params, 
                                           network_input_size, 
                                           network_output_size, 
                                           network_hidden_size)

class GraspingTests(unittest.TestCase):
    def setUp(self):
        if os.name == "posix":
            net, normalizer = cPickle.load(file("/media/isrc_private/Logs/ObjectDiscovery/net_norm_v3.dat"))
        else:
            net, normalizer = cPickle.load(file("\\\\isrchn1\\userdata\\se15005594\\net_norm_v3.dat"))
        self.net = net
        self.normalizer = normalizer
    
    def test_with_sampling(self):
        robot = Robot( pos = (0., 0., 0.), th = 0.)
        
        netout = [0]
        while netout[0] == 0:
            sample = self.net.sample_inputs(1)
            netout = self.net(sample)
        
        
        sample = self.normalizer.denormalize(sample)
        
        robot.torso_height = sample[0,3]
        
        pos = sample[0,0], sample[0,1], sample[0,2]
        dims = (0.1, 0.1, 0.1)
        
        obj = Object(pos = pos, dims = dims)
        can_grasp = robot.can_grasp(obj, self.net, self.normalizer)
        
        self.assertTrue(can_grasp)

    def test_count_graspable(self):
        val = grasping_world.graspability_stats(self.net, self.normalizer, 
                                          num_trials=1000)
        print "\nPercentage of grasps: ", val, "\n"

    def test_sample_genomes(self):
        
        steps = 1000
        successess = 0
        objects_found = 0
        
        succ_len = 0.0
        
        for trial in xrange(steps):
            experiment = smach_explore_grasp.ExperimentSetup()
            experiment.max_transitions = 40
            experiment.network_hidden_size = 3
            experiment.net_evolutions = 20
            experiment.max_w = 3.0
            experiment.min_w = -3.0
            experiment.num_nodes = 20
            
            experiment.pop_size = 200
            experiment.num_trials = 1
            experiment.stop_elitism = False
            experiment.poolsize = 50
            experiment.migration_rate = 1
            experiment.migration_size = 1
            
            if os.name == "posix":
                experiment.rbfnetworkpath = "/media/cluster_space/net_norm_v3.dat"
                experiment.explorer_path = "/media/cluster_space/explorer_genome.dat"
            else:
                experiment.rbfnetworkpath = "\\\\isrchn1\\userdata\\se15005594\\net_norm_v3.dat"
                experiment.explorer_path = "\\\\isrchn1\\userdata\\se15005594\\explorer_genome.dat"
            
            experiment.name = "Evolve for exploring grasping"
            experiment.note = ""
            
            experiment.initialize()
            factory = smach_explore_grasp.ExplorerFactory(experiment)            
            genome = graph_genome.GraphGenome(experiment.num_nodes, 
                                              factory.node_degrees,
                                              factory.node_params)
            genome.initialize()
            genome = genome.clone()
            genome.mutate(pmut = experiment.mutation_rate, ga_engine=None)
            
            sm = chromosome_smach.convert_chromosome_smach(genome, 
                                                           factory.names_mapping, 
                                                           factory.transitions_mapping, 
                                                           factory.classes_mapping, 
                                                           factory.data_mapping)
            
            outcome = sm.execute()
            if outcome == "success":
                successess += 1
                succ_len += float(len(genome))
            try:
                if sm.userdata.object_found:
                    objects_found += 1
            except KeyError:
                pass
                        
                
        print "\n\nExpplore Grasp success rate: ", float(successess) / float(steps)
        print "Expplore Grasp objects found: ", float(objects_found) / float(steps)
        if successess > 0:
            print "Average successfull length: ", succ_len / float(successess)
    
    def test_evolution(self):
        
        def eval_func(chromosome, **args):
    
            factory = smach_explore_grasp.ExplorerFactory(experiment)
            
            sm = convert_chromosome_smach(chromosome, 
                                          factory.names_mapping, 
                                          factory.transitions_mapping, 
                                          factory.classes_mapping,
                                          factory.data_mapping)
            try:
                outcome = sm.execute()
            except smach.InvalidUserCodeError, e:
                print "\nERROR: ", e.message
                raise
        
            if outcome == "success":
                return 1.0
            else:
                return 0.0
        
        experiment = smach_explore_grasp.ExperimentSetup()

        experiment.max_transitions = 20
        experiment.network_hidden_size = 3
        experiment.net_evolutions = 20
        experiment.max_w = 3.0
        experiment.min_w = -3.0
        experiment.num_nodes = 20
        
        experiment.pop_size = 1000
        experiment.num_trials = 1
        experiment.stop_elitism = False
        experiment.poolsize = 50
        experiment.migration_rate = 1
        experiment.migration_size = 1
        experiment.crossover_rate = 0.1
        experiment.mutation_rate = 0.1
        experiment.p_add = 0.5
        experiment.p_del = 0.1
        
        experiment.freq_stats = 1
        experiment.generations = 2
        if os.name == "posix":
            experiment.rbfnetworkpath = "/media/cluster_space/net_norm_v3.dat"
            experiment.explorer_path = "/media/cluster_space/explorer_genome.dat"
        else:
            experiment.rbfnetworkpath = "\\\\isrchn1\\userdata\\se15005594\\net_norm_v3.dat"
            experiment.explorer_path = "\\\\isrchn1\\userdata\\se15005594\\explorer_genome.dat"
        experiment.initialize()
    
        random.seed(time.time())
        factory = smach_explore_grasp.ExplorerFactory(experiment)
        node_degrees = factory.node_degrees
        node_params = factory.node_params
        names_mapping = factory.names_mapping
        transitions_mapping = factory.transitions_mapping
        
        genome = graph_genome.GraphGenome(experiment.num_nodes, node_degrees, node_params)
        genome.evaluator.set(eval_func)
        
        ga = GSimpleGA.GSimpleGA(genome)
        
        if experiment.stop_elitism:
            ga.setElitism(False)
        else:
            ga.setElitism(True)
            ga.setElitismReplacement(experiment.elitism_size)
        
        ga.selector.set(Selectors.GRouletteWheel)
    #    ga.setSortType(Consts.sortType["raw"])
        
        ga.setGenerations(experiment.generations)
        ga.setPopulationSize(experiment.pop_size)
        ga.setCrossoverRate(experiment.crossover_rate)
        ga.setMutationRate(experiment.mutation_rate)
        ga.getPopulation().setParams(tournamentPool = experiment.poolsize)
        genome.setParams(p_del=experiment.p_del, p_add=experiment.p_del)
    
        ga.setMinimax(Consts.minimaxType["maximize"])
        ga.evolve(freq_stats = experiment.freq_stats)
    
if __name__ == "__main__":
    unittest.main()

