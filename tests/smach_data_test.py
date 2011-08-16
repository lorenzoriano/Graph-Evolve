import smach

class Incrementer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success"],
                             input_keys=["incr_input"],
                             output_keys=["incr_output"])
        
    def execute(self, userdata):
        try:
            userdata.incr_output = userdata.incr_input + 1
        except KeyError:
            userdata.incr_output = 0
        return "success"
    

def main():
    sm = smach.StateMachine(outcomes=["success"])
    
    with sm:
        smach.StateMachine.add('A', Incrementer(),
                               transitions={'success' : 'B'},
                               remapping = {'incr_input' : 'init_value',
                                            'incr_output' :  'init_value'})
        smach.StateMachine.add('B', Incrementer(),
                               transitions={'success' : 'success'},
                               remapping = {'incr_input' : 'init_value',
                                            'incr_output' :  'init_value'})
    
    outcome = sm.execute()
    print "Userdata: ", sm.userdata.init_value

if __name__ == "__main__":
    main()