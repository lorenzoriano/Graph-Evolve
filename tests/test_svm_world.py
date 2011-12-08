import sys
import os
if os.name == "posix":
    sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src')
    sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src/pyevolve')
    sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src/rbfnetwork')
else:
    sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src')
    sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src\\pyevolve')

import unittest
from simulators import multi_svm 
sys.modules["multi_svm"] = multi_svm
import cPickle

from simulators.svm_grasping_world import Object, Robot, GraspingWorld

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

        simulator.create_table(*table_0)
        simulator.create_object(object_0)

        ret = simulator.move_robot(*movement)
        self.assertTrue(ret)

        print "Object next is: ", simulator.obj_vec


    
       
if __name__ == "__main__":
    unittest.main()

