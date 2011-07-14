#!/usr/bin/env python

import smach
import math
import random

target_pos = (0.0,0.0,0.0)
target_rot = (math.pi/2, 0, 0)
success_prob = 0.9

class RobotWorldState(object):
    def __init__(self):
        self.gripper_pos = (0,0,0)
        self.gripper_rot = (0,0,0)
        self.object_pos = None
        self.gripper_closed = False
        self.object_in_gripper = False
        self.number_of_transitions = 0
        self.max_number_of_transitions = 10
        self.has_rotated = False
    
    def __repr__(self):
        msg = "%s\n%s\n%s\n%s\n%s\n%s\n%s" % (
                                               self.gripper_pos,
                                               self.gripper_rot,
                                               self.object_pos,
                                               self.gripper_closed,
                                               self.object_in_gripper,
                                               self.number_of_transitions,
                                               self.has_rotated
                                               )
        return msg
        

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

def state_dist_gripper_rotation(state, target):
    gx,gy,gz = state.gripper_rot
    ox,oy,oz = target
    delta = (ox-gx)**2 + (oy-gy)**2 + (oz-gz)**2
    return math.sqrt(delta)

def state_clone(state):
    newstate = RobotWorldState()
    newstate.gripper_pos = state.gripper_pos
    newstate.gripper_rot = state.gripper_rot
    newstate.object_pos = state.object_pos
    newstate.gripper_closed = state.gripper_closed
    newstate.object_in_gripper = state.object_in_gripper
    newstate.number_of_transitions = state.number_of_transitions
    newstate.max_number_of_transitions = state.max_number_of_transitions
    newstate.has_rotated = state.has_rotated
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
        state_out.number_of_transitions += 1
        if robot_state.object_pos is None:
            return "failure"
        if robot_state.gripper_closed:            
            return "failure"
        if robot_state.object_in_gripper:
            return "failure"
        
        dist = state_dist_gripper_object(robot_state)
        if dist < 0.5 and random.random() < success_prob:
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
        if robot_state.object_in_gripper and robot_state.has_rotated:
            return "failure"
        
        if random.random() < success_prob:            
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
        max_pos = 8.0
        self.target = [2*max_pos * t - max_pos for t in target]
        
    def execute(self, userdata):
        robot_state = userdata.state_in
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            return "timeout"        
        state_out = state_clone(robot_state)
        userdata.state_out = state_out
        state_out.number_of_transitions += 1
        if robot_state.object_in_gripper and robot_state.has_rotated:
            return "failure"
        
        if random.random() < success_prob:            
            newpos = (self.target[0] + random.normalvariate(0,0.0),
                      self.target[1] + random.normalvariate(0,0.0),
                      self.target[2] + random.normalvariate(0,0.0),)
            state_out.gripper_pos = newpos
            
            return "success"
        else:
            return "failure"

class RotateGripperToParam(smach.State):
    def __init__(self, target):
        smach.State.__init__(self,
                             outcomes=['success', 'failure',"timeout"],
                             input_keys=['state_in'],
                             output_keys=['state_out'])
        self.target = [2. * math.pi * t - math.pi for t in target]
        
    def execute(self, userdata):
        robot_state = userdata.state_in
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            return "timeout"        
        state_out = state_clone(robot_state)
        userdata.state_out = state_out
        state_out.number_of_transitions += 1
        
        state_out.has_rotated = True
        newpos = (self.target[0],
                  self.target[1],
                  self.target[2])
        state_out.gripper_rot = newpos
        return "success"

class CheckSuccess(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['success', 'failure', "timeout"],
                             input_keys = ['state_in'],
                             output_keys = ['state_out'])
        self.target = target_pos
        self.target_rot = target_rot
        
    def execute(self, userdata):
        robot_state = userdata.state_in
        if robot_state.number_of_transitions >= robot_state.max_number_of_transitions:
            return "timeout"        
        state_out = state_clone(robot_state)
        userdata.state_out = state_out
        state_out.number_of_transitions += 1        
        if robot_state.object_in_gripper:
            if state_dist_gripper_object(robot_state, self.target ) < 0.01:
                if state_dist_gripper_rotation(robot_state, self.target_rot) < 0.1:
                    return "success"
                else:
                    return "failure"
            else:
                return "failure"
        else:
            return "failure"
    

def null_print(_):
    pass

names_mapping = ["RecogniseObject",
                 "MoveToObject",
                 "GraspObject",
                 "CheckSuccess",
                 "MoveGripperToParam",
                 "RotateGripperToParam"]
classes_mapping = {"RecogniseObject" : RecogniseObject,
                 "MoveToObject": MoveGripperTo,
                 "GraspObject": GraspObject,
                 "CheckSuccess": CheckSuccess,
                 "MoveGripperToParam" : MoveGripperToParam,
                 "RotateGripperToParam" : RotateGripperToParam}
transitions_mapping = {'RecogniseObject':["success",],
                       'MoveToObject':["success", "failure",],
                       'GraspObject':["success", "failure",],
                       "CheckSuccess":[],
                       "MoveGripperToParam":["success", "failure"],
                       "RotateGripperToParam":["success"]}

default_mapping = {'state_in':'robot_state',
                   'state_out':'robot_state',
                   }
data_mapping = {'RecogniseObject': default_mapping,
                'MoveToObject': default_mapping,
                'GraspObject' :default_mapping,
                "CheckSuccess": default_mapping,
                "MoveGripperToParam": default_mapping,
                "RotateGripperToParam": default_mapping}
 
node_degrees = [1,2,2,0,2,1]
nodes_params = [0,0,0,0,3,3]
