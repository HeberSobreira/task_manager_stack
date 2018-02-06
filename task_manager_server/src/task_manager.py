#!/usr/bin/env python

import sys
import rospy
import traceback

from task_manager_msgs.srv import AssignMission

from task_manager_class import TaskManager
from ros_interface import ROSInterface

if __name__ == "__main__":

    rospy.init_node('task_manager')

    try:
        ## Parameters
        robotId = ROSInterface.get_ROS_param('~robot_id')
        skills = ROSInterface.get_ROS_param('~skills')
        waitForServerTimeOut = ROSInterface.get_ROS_param('~wait_for_server_timeout')
        waitForActionClientTimeOut = ROSInterface.get_ROS_param('~wait_for_action_client_timeout')
        missionQueueSize = ROSInterface.get_ROS_param('~mission_queue_size')

        ## Services
        assignMissionServiceName = ROSInterface.get_ROS_param('~assign_mission_service_name')
        provideTaskStatusServiceName = ROSInterface.get_ROS_param('~provide_task_status_service_name')
        cancelMissionServiceName = ROSInterface.get_ROS_param('~cancel_mission_service_name')

        ## Topics
        taskStatusTopic = '~TaskStatus'

    except KeyError as e:
        rospy.logerr('[TaskManager] Error: %s', str(e))
        rospy.logdebug(traceback.format_exc())

        quit()


    try:
        tm = TaskManager(robotId = robotId, skills = skills,
                         assignMissionServiceName = assignMissionServiceName,
                         cancelMissionServiceName = cancelMissionServiceName,
                         provideTaskStatusServiceName = provideTaskStatusServiceName,
                         taskStatusTopic = taskStatusTopic,
                         waitForServerTimeOut = waitForServerTimeOut,
                         waitForActionClientTimeOut = waitForActionClientTimeOut,
                         missionQueueSize = missionQueueSize)

        rospy.spin()

    except Exception as e:
        rospy.logerr('[TaskManager] Error: %s', str(e))
        rospy.logdebug(traceback.format_exc())

        quit()
