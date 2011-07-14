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
            smach.StateMachine.add("MoveGripperTo", smach_stack.MoveGripperTo([0.6, 0.75, 0.75, 2.0],),
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
        
        self.assertEqual(outcome, "success")
    
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
        
        self.assertEqual(outcome, "failure")
        
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
        
        self.assertEqual(outcome, "failure")