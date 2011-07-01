#!/usr/bin/env python

import smach
import math
import random
import cPickle

class RobotWorldState(object):
    def __init__(self):
        self.gripper_pos = (0,0,0)
        self.object_pos = None
        self.gripper_closed = False
        self.object_in_gripper = False
        self.number_of_transitions = 0
        self.max_number_of_transitions = 10

def state_dist_gripper_object(state, target=None):
    if state.object_pos is None and target is None:
        return 1e1
    gx,gy,gz = state.gripper_pos
    if target is None:
        ox,oy,oz = state.object_pos
    else:
        ox,oy,oz = target
    delta = (ox-gx)**2 + (oy-gy)**2 + (oz-gz)**2
    return math.sqrt(delta)

def state_clone(state):
    newstate = RobotWorldState()
    newstate.gripper_pos = state.gripper_pos
    newstate.object_pos = state.object_pos
    newstate.gripper_closed = state.gripper_closed
    newstate.object_in_gripper = state.object_in_gripper
    newstate.number_of_transitions = state.number_of_transitions
    return newstate

def state_print(state):
    print state.gripper_pos
    print state.object_pos
    print state.gripper_closed
    print state.object_in_gripper
    print state.number_of_transitions

class RecogniseObject(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', "timeout"],
                             input_keys=['state_in'],
                             output_keys=['state_out'])
    def execute(self, userdata):
        robot_state = userdata.state_in
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            return "timeout"
        state_out = state_clone(robot_state)
        userdata.state_out = state_out
        state_out.number_of_transitions +=1
        state_out.object_pos = (0,
                                0,
                                5)
        return "success"
        
class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'failure',"timeout"],
                             input_keys=['state_in'],
                             output_keys=['state_out'])

    def execute(self, userdata):
        robot_state = userdata.state_in
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            return "timeout"       
        state_out = state_clone(robot_state)
        userdata.state_out = state_out
        state_out.number_of_transitions +=1
        if robot_state.object_pos is None:
            return "failure"
        if robot_state.gripper_closed:            
            return "failure"
        if robot_state.object_in_gripper:
            return "failure"
        
        dist = state_dist_gripper_object(robot_state)
        if dist < 0.5:
            state_out.object_in_gripper = True
            state_out.gripper_closed = True
            return "success"        
        else:
            return "failure"

class MoveGripperTo(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'failure',"timeout"],
                             input_keys=['state_in'],
                             output_keys=['state_out'])
        
    def execute(self, userdata):
        robot_state = userdata.state_in
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            return "timeout"        
        state_out = state_clone(robot_state)
        userdata.state_out = state_out
        state_out.number_of_transitions +=1
        if robot_state.object_pos is None:
            return "failure"
        
        if random.random() < 0.7:            
            newpos = (robot_state.object_pos[0] + random.normalvariate(0,0.0),
                      robot_state.object_pos[1] + random.normalvariate(0,0.0),
                      robot_state.object_pos[2] + random.normalvariate(0,0.0),)
            state_out.gripper_pos = newpos
            return "success"
        else:
            return "failure"

class MoveGripperToParam(smach.State):
    def __init__(self, target):
        smach.State.__init__(self,
                             outcomes=['success', 'failure',"timeout"],
                             input_keys=['state_in'],
                             output_keys=['state_out'])
        self.target = [20. * t - 10. for t in target]
        
    def execute(self, userdata):
        robot_state = userdata.state_in
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            return "timeout"        
        state_out = state_clone(robot_state)
        userdata.state_out = state_out
        state_out.number_of_transitions +=1
        if robot_state.object_pos is None:
            return "failure"
        
        if random.random() < 1.0:            
            newpos = (self.target[0] + random.normalvariate(0,0.0),
                      self.target[1] + random.normalvariate(0,0.0),
                      self.target[2] + random.normalvariate(0,0.0),)
            state_out.gripper_pos = newpos
            return "success"
        else:
            return "failure"

class CheckSuccess(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['success', 'failure', "timeout"],
                             input_keys = ['state_in'],
                             output_keys = ['state_out'])
        self.target = (0,0,0)
        
    def execute(self, userdata):
        robot_state = userdata.state_in
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            return "timeout"        
        state_out = state_clone(robot_state)
        userdata.state_out = state_out
        state_out.number_of_transitions += 1
        if robot_state.object_in_gripper:
            if state_dist_gripper_object(robot_state, self.target ) < 0.1:
                return "success"
            else:
                return "failure"
        else:
            return "failure"
    

def null_print(msg):
    pass

def main():

    random.seed()
    smach.set_loggers(smach.loginfo,
                      smach.logwarn,
                      null_print,
                      smach.logerr)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', "failure"])
    robot_state =  RobotWorldState()
    sm.userdata.robot_state = robot_state

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('RecogniseObject', RecogniseObject(),
                               transitions={'success':'MoveToObject',
                                            'timeout':'failure'
                                            },
                               remapping={'state_in':'robot_state',
                                          'state_out':'robot_state',
                                          })
        smach.StateMachine.add('MoveToObjectParam', MoveGripperToParam((0.5,0.5,0.5)),
                               transitions={'success':'CheckSuccess',
                                            'failure':'MoveToObjectParam',
                                            'timeout':'failure'
                                            },
                               remapping={'state_in':'robot_state',
                                          'state_out':'robot_state',
                                          })
        smach.StateMachine.add('MoveToObject', MoveGripperTo(),
                               transitions={'success':'GraspObject',
                                            'failure':'MoveToObject',
                                            'timeout':'failure'
                                            },
                               remapping={'state_in':'robot_state',
                                          'state_out':'robot_state',
                                          })
        smach.StateMachine.add('GraspObject', GraspObject(),
                               transitions={'success':'MoveToObjectParam',
                                            'failure':'MoveToObject',
                                            'timeout':'failure'
                                            },
                               remapping={'state_in':'robot_state',
                                          'state_out':'robot_state',
                                          })
        smach.StateMachine.add('CheckSuccess', CheckSuccess(),
                               transitions={'success':'success',
                                            'failure':'failure',
                                            'timeout':'failure'
                                            },
                               remapping={'state_in':'robot_state',
                                          'state_out':'robot_state',
                                          })
    # Execute SMACH plan
    try:
        outcome = sm.execute()
    except smach.exceptions.InvalidUserCodeError as err:
        print "Error: ", err.message
    else:
        print "Outcome: ", outcome

    state_print(sm.userdata.robot_state)
    print "Distance is: ",  state_dist_gripper_object(sm.userdata.robot_state,
                                                      (0,0,0))
    
if __name__ == '__main__':
    main()