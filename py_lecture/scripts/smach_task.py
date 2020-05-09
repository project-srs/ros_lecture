#!/usr/bin/env python

import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty

# define state Foo
class MoveBaseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'

def child_term_cb(outcome_map):
    print("child_term_cb")
    if outcome_map['MOVE2'] == 'succeeded':
        return True
    if outcome_map['MONITOR2']:
        return True
    return False

def out_cb(outcome_map):
    print("out_cb")
    if outcome_map['MOVE2'] == 'succeeded':
        return 'done'
    else:
        return 'exit'

def monitor_cb(ud, msg):
    print("monitor_cb")
    return False

def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['done', 'exit'])
    
    # Open the container
    with sm_top:
        goal1=MoveBaseGoal()
        goal1.target_pose.header.frame_id = "dtw_robot1/map"
        goal1.target_pose.pose.position.x = 0.5
        goal1.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add('MOVE1', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal1), transitions={'succeeded':'TASK2', 'preempted':'done', 'aborted':'done'})

        task2_concurrence = smach.Concurrence(outcomes=['done', 'exit'], default_outcome='done', child_termination_cb = child_term_cb, outcome_cb = out_cb)
        with task2_concurrence:
            goal2=MoveBaseGoal()
            goal2.target_pose.header.frame_id = "dtw_robot1/map"
            goal2.target_pose.pose.position.x = -0.5
            goal2.target_pose.pose.orientation.w = 1.0
            smach.Concurrence.add('MOVE2', smach_ros.SimpleActionState('/dtw_robot1/move_base', MoveBaseAction, goal=goal2))
            smach.Concurrence.add('MONITOR2', smach_ros.MonitorState("/sm_stop", Empty, monitor_cb))
        smach.StateMachine.add('TASK2', task2_concurrence, transitions={'done':'done', 'exit':'exit'}) 



    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()