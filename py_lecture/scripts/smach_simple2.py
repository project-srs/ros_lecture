#!/usr/bin/env python

import rospy
import smach
import smach_ros

class State1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','exit'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE1')
        rospy.sleep(2.0)
        if self.counter < 3:
            self.counter += 1
            return 'done'
        else:
            return 'exit'

class State2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE2')
        rospy.sleep(2.0)
        return 'done'

class State3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE2')
        rospy.sleep(2.0)
        return 'done'

# main
def main():
    rospy.init_node('smach_somple2')

    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        smach.StateMachine.add('STATE3', State3(), transitions={'done':'SUB'})

        sm_sub = smach.StateMachine(outcomes=['done'])
        with sm_sub:
            smach.StateMachine.add('STATE1', State1(), transitions={'done':'STATE2', 'exit':'done'})
            smach.StateMachine.add('STATE2', State2(), transitions={'done':'STATE1'})

        smach.StateMachine.add('SUB', sm_sub, transitions={'done':'succeeded'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()