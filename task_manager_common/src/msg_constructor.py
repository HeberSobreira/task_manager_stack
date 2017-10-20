#!/usr/bin/env python

import sys
import rospy

from task_manager_msgs.msg import ActionAcceptedRefused
from task_manager_msgs.msg import TaskStatus

# Class for ROS Message Construction
class MSGConstructor(object):

    """docstring for MSGConstructor."""

    @classmethod
    def ActionAcceptedRefusedConstructor(self, accepted, reasonOfRefusal):
        response = ActionAcceptedRefused()

        response.accepted = accepted
        response.reason_of_refusal = reasonOfRefusal

        return response

    ## TODO: Missing Unit Test
    @classmethod
    def TaskStatusConstructor(self, missionId = 'defaultMissionId', taskId = 'defaultTaskId', statusCode = 0, statusDescription = 'defaultStatusDescription', when = None):
        taskStatus = TaskStatus()

        taskStatus.missionId = str(missionId)
        taskStatus.taskId = str(taskId)
        taskStatus.statusCode = int(statusCode)
        taskStatus.statusDescription = str(statusDescription)
        taskStatus.when = when

        return taskStatus


    ## PoseStampedConstructor
