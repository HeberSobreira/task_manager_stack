#!/usr/bin/env python

import sys
import rospy

from task_manager_msgs.msg import ActionAcceptedRefused
from task_manager_msgs.msg import TaskStatus
# from geometry_msgs.msg import PoseStamped

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
    # def PoseStamped_message_creation(frame_id, px, py, pz, qx, qy, qz, qw):
    #
    # 	# Creates a geometry_msgs/PointStamped Message
    # 	Pose = PoseStamped()
    #
    # 	Pose.header.frame_id = frame_id
    #
    # 	# Origin coordinates
    # 	Pose.pose.position.x = px
    # 	Pose.pose.position.y = py
    # 	Pose.pose.position.z = pz
    #
    # 	Pose.pose.orientation.x = qx
    # 	Pose.pose.orientation.y = qy
    # 	Pose.pose.orientation.z = qz
    # 	Pose.pose.orientation.w = qw
    #
    # 	return Pose
    #
