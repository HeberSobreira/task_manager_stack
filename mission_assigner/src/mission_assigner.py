#!/usr/bin/env python

import sys
import rospy
import traceback

from mission_assigner_class import MissionAssigner
from ros_interface import ROSInterface

if __name__ == "__main__":

    rospy.init_node('mission_assigner')

    try:
        ## Parameters
        robotId = ROSInterface.get_ROS_param('~robotId')
        missionId = ROSInterface.get_ROS_param('~missionId')
        skills = ROSInterface.get_ROS_param('~skills')
        task = ROSInterface.get_ROS_param('~task')

        ## Services
        assignMissionServiceName = ROSInterface.get_ROS_param('~assign_mission_service_name')

    except KeyError as e:
        rospy.logerr('[TaskManager] Error: %s', str(e))
        rospy.logdebug(traceback.format_exc())

        quit()

    try:
        tasks = task['tasks']
        priority = task['priority']
        ma = MissionAssigner(robotId, missionId, tasks, priority, skills)
        ma.assign_mission(assignMissionServiceName)

    except Exception as e:
        rospy.logerr('[MissionAssigner] Error: %s', str(e))
        rospy.logdebug(traceback.format_exc())

    quit()
