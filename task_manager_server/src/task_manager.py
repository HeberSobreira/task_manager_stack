#!/usr/bin/env python

import sys
import rospy
import traceback

from task_manager_msgs.srv import AssignMission

from task_manager_class import TaskManager
from ros_interface import ROSInterface

if __name__ == "__main__":

    rospy.init_node('task_manager')

    robotId = ROSInterface.get_ROS_param('~robotId')

    # TODO: This should not be hardcoded... Shold be passeable from the launch file!
    skills = ROSInterface.get_ROS_param(str(robotId) + '/skills')

    # TODO: This should not be hardcoded... Shold be passeable from the launch file!
    assignMissionServiceName = '/stamina_msgs/' + str(robotId) + '/AssignMission'

    # TODO: This should not be hardcoded... Shold be passeable from the launch file!
    provideTaskStatusServiceName = '/stamina_msgs/' + str(robotId) + '/ProvideTaskStatus'

    # TODO: This should not be hardcoded... Shold be passeable from the launch file!
    taskStatusTopic = '/stamina_msgs/TaskStatus'

    # TODO: This should not be hardcoded... Shold be passeable from the launch file!
    waitForServerTimeOut = 10

    # TODO: This should not be hardcoded... Shold be passeable from the launch file!
    waitForActionClientTimeOut = 300

    try:
        tm = TaskManager(robotId = robotId, skills = skills, assignMissionServiceName = assignMissionServiceName, provideTaskStatusServiceName = provideTaskStatusServiceName, taskStatusTopic = taskStatusTopic, waitForServerTimeOut = waitForServerTimeOut, waitForActionClientTimeOut = waitForActionClientTimeOut)

        rospy.spin()

    except Exception as e:
        rospy.logerr('[TaskManager] Error: %s', str(e))
        rospy.logdebug(traceback.format_exc())

        quit()
