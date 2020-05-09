#!/usr/bin/env python

import rospy
import smach

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

def main():
    rospy.init_node('smach_somple1')

    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        smach.StateMachine.add('STATE1', State1(), transitions={'done':'STATE2', 'exit':'succeeded'})
        smach.StateMachine.add('STATE2', State2(), transitions={'done':'STATE1'})

    outcome = sm_top.execute()

if __name__ == '__main__':
    main()