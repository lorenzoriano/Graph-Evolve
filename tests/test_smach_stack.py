import unittest
import smach
import smach_stack

smach.set_loggers(smach.loginfo,
                  smach_stack.null_print,
                  smach_stack.null_print,
                  smach_stack.null_print)

class BasicTests(unittest.TestCase):
    def test_in_collision(self):
        pos1 = (0,0,0)
        dims1 = (1.0, 1.0)
        obj1 = smach_stack.Object(pos1, dims1)
        
        pos2 = (0.1,0,0)
        dims2 = (0.1, 0.5)
        obj2 = smach_stack.Object(pos2, dims2)        
        self.assertTrue(smach_stack.objects_collide(obj1, obj2))
        
        pos2 = (0.1,0.5,0)
        dims2 = (0.1, 0.5)
        obj2 = smach_stack.Object(pos2, dims2)        
        self.assertTrue(smach_stack.objects_collide(obj1, obj2))
        
        pos2 = (0.1,0,0.3)
        dims2 = (0.1, 0.5)
        obj2 = smach_stack.Object(pos2, dims2)        
        self.assertTrue(smach_stack.objects_collide(obj1, obj2))
        
        pos2 = (-0.1,0,0)
        dims2 = (0.1, 0.5)
        obj2 = smach_stack.Object(pos2, dims2)        
        self.assertTrue(smach_stack.objects_collide(obj1, obj2))
        
        pos1 = (0.33524283935165333, -0.059357905570512348, 0.5)
        dims1 =  (0.193690329274, 0.101006366107)
        obj1 = smach_stack.Object(pos1, dims1)
        pos2 = (0.33524283935165333, -0.059357905570512348, 0.5)
        dims2 = (0.108117871605, 0.102062692143)
        obj2 = smach_stack.Object(pos2, dims2)        
        self.assertTrue(smach_stack.objects_collide(obj1, obj2))
    
    
    
    def test_notin_collision(self):
        pos1 = (0,0,0)
        dims1 = (1.0, 1.0)
        obj1 = smach_stack.Object(pos1, dims1)
        
        pos2 = (0.6,0,0)
        dims2 = (0.1, 0.1)
        obj2 = smach_stack.Object(pos2, dims2)        
        self.assertFalse(smach_stack.objects_collide(obj1, obj2))
        
        pos2 = (0.0,0.6,0)
        dims2 = (0.1, 0.1)
        obj2 = smach_stack.Object(pos2, dims2)        
        self.assertFalse(smach_stack.objects_collide(obj1, obj2))
        
        pos2 = (0.0,0,1.1)
        dims2 = (0.1, 0.1)
        obj2 = smach_stack.Object(pos2, dims2)        
        self.assertFalse(smach_stack.objects_collide(obj1, obj2))
        
        pos2 = (-0.6,0,0)
        dims2 = (0.1, 0.1)
        obj2 = smach_stack.Object(pos2, dims2)        
        self.assertFalse(smach_stack.objects_collide(obj1, obj2))
    
    
    def test_initialize(self):
        obj1 = smach_stack.Object()
        smach_stack.initialize(obj1)
         
        self.assertNotEqual(obj1.pos[0], None )
        self.assertTrue(obj1.initialized)
        
        obj2 = smach_stack.clone_object(obj1)
        self.assertNotEqual(obj2.pos[0], None )
        self.assertTrue(obj2.initialized)
    
    

class TestSM(unittest.TestCase):
    def test_solution(self):
        sm = smach.StateMachine(outcomes=['success', "failure", "timeout"])
        default_mapping = {'state_in':'robot_state',
                           'state_out':'robot_state',
                           }
        
        robot_state =  smach_stack.RobotWorldState()
        with sm:
            smach.StateMachine.add("DetectObjects", smach_stack.DetectObjects(),
                                   transitions={"success": "GraspObject",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("GraspObject", smach_stack.GraspObject([0.6],),
                                   transitions={"success": "MoveGripperTo",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("MoveGripperTo", smach_stack.MoveGripperTo([0.6, 0.75, 0.75, 1.0],),
                                   transitions={"success": "OpenGripper",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("OpenGripper", smach_stack.OpenGripper(),
                                   transitions={"success": "CheckSuccess",
                                                "timeout": "timeout"},
                                                remapping = default_mapping)
            smach.StateMachine.add("CheckSuccess", smach_stack.CheckSuccess(),
                                   transitions={"success": "success",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
        
        sm.userdata.robot_state = robot_state
        
        try:
            outcome = sm.execute()
        except smach.InvalidUserCodeError as e:
            print e.message
            raise
        
        if outcome != "success":
            
            print "OBJ1 POS: ", sm.userdata.robot_state.obj1.pos
            print "OBJ1 DIMS: ", sm.userdata.robot_state.obj1.height, sm.userdata.robot_state.obj1.width
            
            print "OBJ2: ", sm.userdata.robot_state.obj2.pos
            print "OBJ1 DIMS: ", sm.userdata.robot_state.obj2.height, sm.userdata.robot_state.obj2.width

                    
        self.assertEqual(outcome, "success")
        robot_state =  sm.userdata.robot_state
        self.assertTrue(robot_state.obj1.initialized)
        self.assertTrue(robot_state.obj2.initialized)
    
    def test_solution2(self):
        sm = smach.StateMachine(outcomes=['success', "failure", "timeout"])
        default_mapping = {'state_in':'robot_state',
                           'state_out':'robot_state',
                           }
        
        robot_state =  smach_stack.RobotWorldState()
        with sm:
            smach.StateMachine.add("DetectObjects", smach_stack.DetectObjects(),
                                   transitions={"success": "GraspObject",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("GraspObject", smach_stack.GraspObject([0.36722707341559913],),
                                   transitions={"success": "MoveGripperTo",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("MoveGripperTo", smach_stack.MoveGripperTo([0.3892801169423678, 
                                                                               0.75262854737284701, 
                                                                               0.74475122670576754, 
                                                                               0.74528599651119609],),
                                   transitions={"success": "OpenGripper",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("OpenGripper", smach_stack.OpenGripper(),
                                   transitions={"success": "CheckSuccess",
                                                "timeout": "timeout"},
                                                remapping = default_mapping)
            smach.StateMachine.add("CheckSuccess", smach_stack.CheckSuccess(),
                                   transitions={"success": "success",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
        
        sm.userdata.robot_state = robot_state
        
        
        try:
            outcome = sm.execute()
        except smach.InvalidUserCodeError as e:
            print e.message
            raise
        
        if outcome != "success":
            
            print "OBJ1 POS: ", sm.userdata.robot_state.obj1.pos
            print "OBJ1 DIMS: ", sm.userdata.robot_state.obj1.height, sm.userdata.robot_state.obj1.width
            
            print "OBJ2: ", sm.userdata.robot_state.obj2.pos
            print "OBJ1 DIMS: ", sm.userdata.robot_state.obj2.height, sm.userdata.robot_state.obj2.width

                    
        self.assertEqual(outcome, "success")
        robot_state = sm.userdata.robot_state
        self.assertTrue(robot_state.obj1.initialized)
        self.assertTrue(robot_state.obj2.initialized)
    
    def test_failure1(self):
        sm = smach.StateMachine(outcomes=['success', "failure", "timeout"])
        default_mapping = {'state_in':'robot_state',
                           'state_out':'robot_state',
                           }
        
        robot_state =  smach_stack.RobotWorldState()
        with sm:
            smach.StateMachine.add("DetectObjects", smach_stack.DetectObjects(),
                                   transitions={"success": "GraspObject",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("GraspObject", smach_stack.GraspObject([0.2],),
                                   transitions={"success": "MoveGripperTo",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("MoveGripperTo", smach_stack.MoveGripperTo([0.6, 0.75, 0.75, 2.0],),
                                   transitions={"success": "OpenGripper",                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("OpenGripper", smach_stack.OpenGripper(),
                                   transitions={"success": "CheckSuccess",
                                                "timeout": "timeout"},
                                                remapping = default_mapping)
            smach.StateMachine.add("CheckSuccess", smach_stack.CheckSuccess(),
                                   transitions={"success": "success",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
        
        sm.userdata.robot_state = robot_state
        
        try:
            outcome = sm.execute()
        except smach.InvalidUserCodeError as e:
            print e.message
            raise
        
        robot_state = sm.userdata.robot_state
        self.assertEqual(outcome, "failure")
        self.assertTrue(robot_state.obj1.initialized)
        self.assertTrue(robot_state.obj2.initialized)
        
    def test_failure2(self):
        sm = smach.StateMachine(outcomes=['success', "failure", "timeout"])
        default_mapping = {'state_in':'robot_state',
                           'state_out':'robot_state',
                           }
        
        robot_state =  smach_stack.RobotWorldState()
        with sm:
            smach.StateMachine.add("DetectObjects", smach_stack.DetectObjects(),
                                   transitions={"success": "GraspObject",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("GraspObject", smach_stack.GraspObject([0.6],),
                                   transitions={"success": "MoveGripperTo",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("MoveGripperTo", smach_stack.MoveGripperTo([0.6, 0.25, 0.75, 2.0],),
                                   transitions={"success": "OpenGripper",                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("OpenGripper", smach_stack.OpenGripper(),
                                   transitions={"success": "CheckSuccess",
                                                "timeout": "timeout"},
                                                remapping = default_mapping)
            smach.StateMachine.add("CheckSuccess", smach_stack.CheckSuccess(),
                                   transitions={"success": "success",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
        
        sm.userdata.robot_state = robot_state
        
        try:
            outcome = sm.execute()
        except smach.InvalidUserCodeError as e:
            print e.message
            raise
        
        robot_state = sm.userdata.robot_state
        self.assertEqual(outcome, "failure")
        self.assertTrue(robot_state.obj1.initialized)
        self.assertTrue(robot_state.obj2.initialized)
    
    def test_failure3(self):
        sm = smach.StateMachine(outcomes=['success', "failure", "timeout"])
        default_mapping = {'state_in':'robot_state',
                           'state_out':'robot_state',
                           }
        
        robot_state =  smach_stack.RobotWorldState()
        with sm:
            smach.StateMachine.add("GraspObject", smach_stack.GraspObject([0.6],),
                                   transitions={"success": "MoveGripperTo",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("MoveGripperTo", smach_stack.MoveGripperTo([0.6, 0.25, 0.75, 2.0],),
                                   transitions={"success": "OpenGripper",                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("OpenGripper", smach_stack.OpenGripper(),
                                   transitions={"success": "CheckSuccess",
                                                "timeout": "timeout"},
                                                remapping = default_mapping)
            smach.StateMachine.add("CheckSuccess", smach_stack.CheckSuccess(),
                                   transitions={"success": "success",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
        
        sm.userdata.robot_state = robot_state
        
        try:
            outcome = sm.execute()
        except smach.InvalidUserCodeError as e:
            print e.message
            raise
        
        robot_state = sm.userdata.robot_state
        self.assertEqual(outcome, "failure")
        self.assertFalse(robot_state.obj1.initialized)
        self.assertFalse(robot_state.obj2.initialized)
    
    def test_failure4(self):
        sm = smach.StateMachine(outcomes=['success', "failure", "timeout"])
        default_mapping = {'state_in':'robot_state',
                           'state_out':'robot_state',
                           }
        
        robot_state =  smach_stack.RobotWorldState()
        with sm:
            smach.StateMachine.add("DetectObjects", smach_stack.DetectObjects(),
                                   transitions={"success": "GraspObject",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("GraspObject", smach_stack.GraspObject([0.6],),
                                   transitions={"success": "MoveGripperTo",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("MoveGripperTo", smach_stack.MoveGripperTo([0.6, 0.25, 0.75, 2.0],),
                                   transitions={"success": "OpenGripper",                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("OpenGripper", smach_stack.OpenGripper(),
                                   transitions={"success": "failure",
                                                "timeout": "timeout"},
                                                remapping = default_mapping)
        
        sm.userdata.robot_state = robot_state
        
        try:
            outcome = sm.execute()
        except smach.InvalidUserCodeError as e:
            print e.message
            raise
        
        robot_state = sm.userdata.robot_state
        self.assertEqual(outcome, "failure")
        self.assertTrue(robot_state.obj1.initialized)
        self.assertTrue(robot_state.obj2.initialized)
    
    def test_timeout(self):
        sm = smach.StateMachine(outcomes=['success', "failure", "timeout"])
        default_mapping = {'state_in':'robot_state',
                           'state_out':'robot_state',
                           }
        
        robot_state =  smach_stack.RobotWorldState()
        with sm:
            smach.StateMachine.add("DetectObjects", smach_stack.DetectObjects(),
                                   transitions={"success": "GraspObject",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("GraspObject", smach_stack.GraspObject([0.6],),
                                   transitions={"success": "MoveGripperTo",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("MoveGripperTo", smach_stack.MoveGripperTo([0.6, 0.25, 0.75, 2.0],),
                                   transitions={"success": "OpenGripper",                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
            smach.StateMachine.add("OpenGripper", smach_stack.OpenGripper(),
                                   transitions={"success": "OpenGripper",
                                                "timeout": "timeout"},
                                                remapping = default_mapping)
            smach.StateMachine.add("CheckSuccess", smach_stack.CheckSuccess(),
                                   transitions={"success": "success",
                                                "failure": "failure",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
        
        sm.userdata.robot_state = robot_state
        
        try:
            outcome = sm.execute()
        except smach.InvalidUserCodeError as e:
            print e.message
            raise
        
        robot_state = sm.userdata.robot_state
        self.assertEqual(outcome, "timeout")
        self.assertTrue(robot_state.obj1.initialized)
        self.assertTrue(robot_state.obj2.initialized)
        
    def test_initialized(self):
        sm = smach.StateMachine(outcomes=['success', "timeout"])
        default_mapping = {'state_in':'robot_state',
                           'state_out':'robot_state',
                           }
        
        robot_state =  smach_stack.RobotWorldState()
        with sm:
            smach.StateMachine.add("DetectObjects", smach_stack.DetectObjects(),
                                   transitions={"success": "success",
                                                "timeout": "timeout"},
                                   remapping = default_mapping)
        
        sm.userdata.robot_state = robot_state
        
        try:
            outcome = sm.execute()
        except smach.InvalidUserCodeError as e:
            print e.message
            raise
        
        robot_state = sm.userdata.robot_state
        self.assertTrue(robot_state.obj1.initialized)
        self.assertTrue(robot_state.obj2.initialized)
        
            