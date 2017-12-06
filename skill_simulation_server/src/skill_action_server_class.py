#! /usr/bin/env python

import roslib
import rospy
import sys
import actionlib

from task_manager_msgs.msg import *


####################
# GLOBAL VARIABLES #
####################

robotId = "igor"


######################
# CLASSES DEFINITION #
######################

class SkillActionBase(object):

    global robotId

    #TODO inputs: skillname, actionType, rate(fazer overload)
    def __init__(self, skillName, actionType, rate = 30):
        #removes the '/' when the the skillName is input by launch file
        if skillName.startswith('/'):
            self._actionName = skillName[1:]
        else:
            self._actionName = skillName
        self._actionType = actionType #TODO: mudar para _actionType
        self.rate = rate

        # create messages that are used to publish feedback/result
        self._feedback = eval('task_manager_msgs.msg.' + str(self._actionType) + 'Feedback()')
        self._result   = eval('task_manager_msgs.msg.' + str(self._actionType) + 'Result()')
        self._as = actionlib.SimpleActionServer(self._actionName, eval('task_manager_msgs.msg.' + str(self._actionType) + 'Action'), execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):

        rospy.loginfo("[FakeSkillServer] [%s] Starting fake execution", self._actionType)

        # helper variables
        r = rospy.Rate(self.rate)
        percentage = 0

        while percentage != 100:

            # Check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo("[FakeSkillServer] [%s] Preempted!", self._actionName)
                self._as.set_preempted()
                break

            # Updates the feedback percentage
            self._feedback.percentage = percentage

            # Publishes the feedback
            self._as.publish_feedback(self._feedback)

            percentage = percentage + 1
            #rospy.loginfo('%s: Proccessed %s' % self._actionName, str(percentage))

            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

        if percentage == 100:
            rospy.loginfo("[FakeSkillServer] [%s] Succeeded!", self._actionName)

            # Updates the result
            self._result.percentage = percentage

            # Publishes the result
            self._as.set_succeeded(self._result)
