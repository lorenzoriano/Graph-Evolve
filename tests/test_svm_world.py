import sys
import os
if os.name == "posix":
    sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src')
    sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src/pyevolve')
else:
    sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src')
    sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src\\pyevolve')

import unittest
from simulators import multi_svm 
sys.modules["multi_svm"] = multi_svm
import cPickle
import pybrain
from pybrain.tools.shortcuts import buildNetwork
import numpy as np

from simulators.svm_grasping_world import Object, Robot, GraspingWorld
import random
from graph_evolve import  smach_svm_grasp
import smach
from graph_evolve import chromosome_smach
from graph_evolve import graph_genome

def null_print(_): pass
smach.set_loggers(null_print, #info
                  null_print, #warn
                  null_print, #debug
                  null_print) #error
import math
pi = math.pi
pi2 = pi/2

import heapq

class BasicTests(unittest.TestCase):
    def setUp(self):
        self.tableSVR = cPickle.load(open("/media/isrc_private/Logs/TryToPushAndActionConsequence/table_changesSVR.pkl"))
        self.objectSVR = cPickle.load(open("/media/isrc_private/Logs/TryToPushAndActionConsequence/object_changesSVR.pkl"))
        self.reachableSVC = cPickle.load(open("/media/isrc_private/Logs/TryToPushAndActionConsequence/reachableSVC.pkl"))

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
   
    def test_object_changes(self):
        simulator = GraspingWorld(self.objectSVR, self.tableSVR, 
                                 self.reachableSVC)

        object_0 = (0.56755231,  0.07264199,  0.76972392)
        table_0 = (0.5606665 , -0.24615364,  1.39404476,  0.42626768)
        
        movement = (-0.92924633, -0.0606179 , 0.00423581)

        simulator.create_table(table_0)
        simulator.create_object(object_0)

        ret = simulator.move_robot(movement)
        self.assertTrue(ret)

        print "Object next is: ", simulator.obj_vec
        print "Table next is: ", simulator.table_vec

    def test_move_straight(self):
        simulator = GraspingWorld(self.objectSVR, self.tableSVR, 
                                 self.reachableSVC)

        object_0 = (0.56755231,  0.07264199,  0.76972392)
        table_0 = (0.5606665 , -0.24615364,  1.39404476,  0.42626768)
        simulator.create_table(table_0)
        simulator.create_object(object_0)
        
        movement = (0.1, 0 , 0.0)

        for i in xrange(10):
            
            ret = simulator.move_robot(movement)
            #print "Object next is: ", simulator.obj_vec
            #print "Table next is: ", simulator.table_vec

            if not ret:
                print "movement stopped"
                print "Final Table is ", simulator.table
                print "Final Table vec is ", simulator.table_vec
                break


    def test_collision(self):
        simulator = GraspingWorld(self.objectSVR, self.tableSVR, 
                             self.reachableSVC)

        table = ( 0.249, -0.24329093,  0.97609358,  0.37783455)
        simulator.create_table(table)
        
        table_obj = simulator.table
        robot = Robot((0,0,0))
        self.assertTrue( GraspingWorld.objects_collide(robot, table_obj) )
         


    def test_object_push_not_collision(self):

        data = cPickle.load(open("/media/isrc_private/Logs/TryToPushAndActionConsequence/training_trypush.pkl"))
        locs = data[0]
        res = data[1]
        nonzerolocs = locs[res.nonzero()[0],:]
        
        collisions = 0
        non_grasps = 0
        simulator = GraspingWorld(self.objectSVR, self.tableSVR, 
                             self.reachableSVC)
        for loc in nonzerolocs:

            obj = loc[:3]
            table = (loc[3], loc[5], loc[4], loc[6])
            simulator.create_table(table)
            simulator.create_object(obj)

            if not simulator.can_grasp():
                non_grasps += 1

            if not simulator.move_robot((0,0,0)):
                print "Location ", table, " not good"
                collisions += 1

        self.assertEqual(collisions,0)
        self.assertEqual(non_grasps,1)
   
    def test_scoring(self):
        data = cPickle.load(open("/media/isrc_private/Logs/TryToPushAndActionConsequence/training_trypush.pkl"))
        locs = data[0]
        res = data[1]
        simulator = GraspingWorld(self.objectSVR, self.tableSVR, 
                             self.reachableSVC)
        nonzerolocs = locs[res.nonzero()[0],:]
        collisions = 0
        non_grasps = 0

        for loc in nonzerolocs:

            obj = loc[:3]
            table = (loc[3], loc[5], loc[4], loc[6])
            simulator.create_table(table)
            simulator.create_object(obj)

            score = simulator.grasp_probability()
            if score < 0.5:
                non_grasps += 1

            if not simulator.move_robot((0,0,0)):
                print "Location ", table, " not good"
                collisions += 1
        
        self.assertEqual(collisions,0)
        self.assertEqual(non_grasps,1)


    def test_neural_net(self):
        simulator = GraspingWorld(self.objectSVR, self.tableSVR, 
                             self.reachableSVC)
        
        obj = [1.49350447,  0.22887855,  0.78522903 ] 
        table = [ 0.93704832, -0.18190929, 2.14552283, 0.39354303]

        simulator.create_table(table)
        simulator.create_object(obj)
        print "before moving: ", simulator.can_grasp()

        num_graspings = 0
        good_hiddens = []
        good_movements = []
        maxw = 3
        minw = -3
        for trial in xrange(60):
            simulator = GraspingWorld(self.objectSVR, self.tableSVR, 
                                 self.reachableSVC)
            
            obj = [1.49350447,  0.22887855,  0.78522903 ] 
            table = [ 0.93704832, -0.18190929, 2.14552283, 0.39354303]

            simulator.create_table(table)
            simulator.create_object(obj)

            nhidden = np.random.random_integers(1,10)
            outclass = pybrain.structure.modules.SigmoidLayer
            net = buildNetwork(7, nhidden, 3, 
                    outclass=outclass)
            
            net.params[:] = (maxw-minw)*np.random.random(net.params.shape) + minw
            dx, dy, dth = net.activate(simulator.net_input()) 
            dx = dx 
            dy = 2*dy - 1.0
            dth = pi * dth - pi2

            simulator.move_robot( (dx,dy,dth) )
            if simulator.can_grasp():
                num_graspings += 1
                good_hiddens.append(nhidden)
                good_movements.append((dx, dy, dth))

        print "Num graspings: ", num_graspings 
        print "Good hiddens: ", good_hiddens
        print "Good movements: ", good_movements

    def test_line_intersect(self):
        table = Object(  (3,0,0), (2,2,1) )
        line = ( (0,0), (5,0) )
        self.assertTrue( GraspingWorld.line_intersect_obj(line[0],line[1],table))
        
        table = Object(  (3,0,0), (2,2,1) )
        line = ( (0,0), (2.1,0) )
        self.assertTrue( GraspingWorld.line_intersect_obj(line[0],line[1],table))

        table = Object(  (3,1,0), (2,2,1) )
        line = ( (0,0), (2.1,0) )
        self.assertTrue( GraspingWorld.line_intersect_obj(line[0],line[1],table))

    def test_intersection_and_collision(self):
        table = Object(  (3,1,0), (2,2,1) )
        dims = (table.x_boundaries[0], 
                table.y_boundaries[0],
                table.x_boundaries[1],
                table.y_boundaries[1]
               )
        for endpos in xrange(-10, 10):
            simulator = GraspingWorld(self.objectSVR, self.tableSVR, 
                                 self.reachableSVC)
            simulator.create_table(dims)
            movement = (5, endpos, 0)
            ret_mov = simulator.move_robot(movement)

            robot = Robot(movement)
            ret_coll = simulator.objects_collide(robot, table)

            self.assertEqual(ret_mov, ret_coll)

class StateMachineTests(unittest.TestCase):
    def test_simple(self):
        sm = smach.StateMachine(outcomes=["success", 
                                          "failure", 
                                          "timeout"])
        experiment = smach_svm_grasp.ExperimentSetup()
        experiment.initialize()
        world = GraspingWorld(
                experiment.objectSVR,
                experiment.tableSVR,
                experiment.reachableSVC)
        obj = [1.49350447,  0.22887855,  0.78522903 ] 
        table = [ 0.93704832, -0.18190929, 2.14552283, 0.39354303]

        world.create_table(table)
        world.create_object(obj)

        params = [random.random() 
                for _ in  xrange(smach_svm_grasp.NeuralNetwork2.nparams)]
        with sm:
            smach.StateMachine.add('N1', smach_svm_grasp.NeuralNetwork2(
                                   params,world,experiment),
                                   transitions={'success' : 'Mover',
                                                "timeout": "timeout"},
                                   )
            smach.StateMachine.add('Mover', smach_svm_grasp.RobotMove(
                                   world,experiment),
                                   transitions={'success' : 'ExitSuccess',
                                                "timeout":  "timeout",
                                                "failure" : "N1"},
                                   )
            smach.StateMachine.add('ExitSuccess', smach_svm_grasp.ExitSuccess(
                                   world,experiment),
                                   transitions={'success' : 'success',
                                                "timeout":  "timeout",
                                                "failure" : "N1"},
                                   )

        sm.execute()        
    
    def test_factory(self):
        experiment = smach_svm_grasp.ExperimentSetup()
        experiment.initialize()
        factory = smach_svm_grasp.SVMWorldFactory(experiment)
        
        world = factory.world
        obj = [1.49350447,  0.22887855,  0.78522903 ] 
        table = [ 0.93704832, -0.18190929, 2.14552283, 0.39354303]

        world.create_table(table)
        world.create_object(obj)

        genome = graph_genome.GraphGenome(50, factory.node_degrees,
                                          factory.node_params)
        genome.initialize()
        
        sm = chromosome_smach.convert_chromosome_smach(genome, 
                                                       factory.names_mapping, 
                                                       factory.transitions_mapping, 
                                                       factory.classes_mapping, 
                                                       factory.data_mapping)
        
        sm.execute()

    def test_random_genomes(self):
        steps = 100
        successess = 0
        
        experiment = smach_svm_grasp.ExperimentSetup()
        experiment.initialize()
        factory = smach_svm_grasp.SVMWorldFactory(experiment)
        
        world = factory.world
        obj = [1.49350447,  0.22887855,  0.78522903 ] 
        table = [ 0.93704832, -0.18190929, 2.14552283, 0.39354303]

        for trial in xrange(steps):
            world.reset()
            world.create_table(table)
            world.create_object(obj)
            genome = graph_genome.GraphGenome(50, factory.node_degrees,
                                              factory.node_params)
            genome.initialize()
            sm = chromosome_smach.convert_chromosome_smach(genome, 
                                                       factory.names_mapping, 
                                                       factory.transitions_mapping, 
                                                       factory.classes_mapping, 
                                                       factory.data_mapping)
            if sm.execute() == "success":
                successess += 1

        print "success rate: ", float(successess) / float(steps)

    def test_scoring(self):
        steps = 100
        scores = []
        
        experiment = smach_svm_grasp.ExperimentSetup()
        experiment.initialize()
        factory = smach_svm_grasp.SVMWorldFactory(experiment)
        
        world = factory.world
        obj = [1.49350447,  0.22887855,  0.78522903 ] 
        table = [ 0.93704832, -0.18190929, 2.14552283, 0.39354303]

        for trial in xrange(steps):
            world.reset()
            world.create_table(table)
            world.create_object(obj)
            genome = graph_genome.GraphGenome(50, factory.node_degrees,
                                              factory.node_params)
            genome.initialize()
            sm = chromosome_smach.convert_chromosome_smach(genome, 
                                                       factory.names_mapping, 
                                                       factory.transitions_mapping, 
                                                       factory.classes_mapping, 
                                                       factory.data_mapping)
            sm.execute()
            scores.append(world.grasp_probability())

        print "10 best: ", heapq.nlargest(10,scores)
        print "mean: ", np.mean(scores)
        print "std: ", np.std(scores)

    def test_fitness_function(self):
        experiment = smach_svm_grasp.ExperimentSetup()
        experiment.initialize()
        factory = smach_svm_grasp.SVMWorldFactory(experiment)
        world = factory.world
        
        genome = graph_genome.GraphGenome(50, factory.node_degrees,
                                              factory.node_params)
        genome.initialize()
        sm = chromosome_smach.convert_chromosome_smach(genome, 
                                                       factory.names_mapping, 
                                                       factory.transitions_mapping, 
                                                       factory.classes_mapping, 
                                                       factory.data_mapping)

        fitness = 0.0
        critical_failures = 0
        for inpt, outp in zip(experiment.test_input, experiment.test_output):
            sm.userdata.clear() 
            obj = inpt[:3]
            table = inpt[3:]
            world.reset()
            world.create_object(obj)
            world.create_table(table)
            
            outcome = sm.execute()
            score = world.grasp_probability()
            if outcome == "timeout":
                score = score / 2

            if outp and (outcome != "success"):
                critical_failures += 1
                score = 0
            
            fitness += score

        print "Fitness: ", fitness, " crit failures: ", critical_failures

if __name__ == "__main__":
    unittest.main()


