#!/usr/bin/env python

import rospy
import smach
import smach_ros

from task_manager_msgs.msg import *

########
## TM ##
########

# Define State MissionSetup
class MissionSetup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing Mission Setup')
        return 'succeeded'

###########
## SKILL ##
###########

# Define State SkillSetup
class SkillSetup(smach.State):
    # NOTE: output_keys, input_keys can be passed to the derived class constructor
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys = ['foo_input'], output_keys = ['exampleSkillProperty0', 'exampleSkillProperty1'])

    # NOTE: actionGoalConstructor can be overloaded on the derived class
    def actionGoalConstructor(self, userdata):
        userdata.exampleSkillProperty0 = 'abc'
        userdata.exampleSkillProperty1 = 'abc'

    def execute(self, userdata):
        rospy.loginfo('Executing Skill Setup')

        self.actionGoalConstructor(userdata)

        return 'succeeded'

# Define State SkillAnalysis
class SkillAnalysis(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing Skill Analysis')
        return 'succeeded'

# NOTE: Should be as generic as possible
def create_skill_sm(skillName):

    ### Create the sub SMACH state machine for the Skill ###
    sm_skill = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm_skill:

        smach.StateMachine.add(str(skillName) + '_SETUP',
                                SkillSetup(),
                                transitions = {'succeeded': str(skillName) + '_EXECUTION', 'aborted': 'aborted'})

        smach.StateMachine.add(str(skillName) + '_EXECUTION',
                               smach_ros.SimpleActionState('GenericSkill', task_manager_msgs.msg.GenericSkillAction, goal_slots = ['exampleSkillProperty0', 'exampleSkillProperty1'], result_slots = ['percentage', 'skillStatus']),
                               transitions = {'succeeded': str(skillName) + '_ANALYSIS'})

        smach.StateMachine.add(str(skillName) + '_ANALYSIS',
                                SkillAnalysis(),
                                transitions = {'succeeded':'succeeded', 'aborted': 'aborted'})

    return sm_skill


##########
## MAIN ##
##########

def main():
    rospy.init_node('smach_tm_tests')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Open the container
    with sm:

        smach.StateMachine.add('MISSION_SETUP',
                                MissionSetup(),
                                transitions = {'succeeded':'DRIVE_SKILL', 'aborted': 'aborted'})

        smach.StateMachine.add('DRIVE_SKILL', create_skill_sm('DRIVE_SKILL'), transitions = {'succeeded':'PICK_SKILL'})
        smach.StateMachine.add('PICK_SKILL', create_skill_sm('PICK_SKILL'), transitions = {'succeeded':'PLACE_SKILL'})
        smach.StateMachine.add('PLACE_SKILL', create_skill_sm('PLACE_SKILL'))

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('smach_viewer_server', sm, '/MISSION_ID')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
