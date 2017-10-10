#!/usr/bin/env python

import sys
import rospy
import traceback

from mission_assigner_class import MissionAssigner
from ros_interface import ROSInterface

if __name__ == "__main__":

    rospy.init_node('mission_assigner')

    robotId = ROSInterface.get_ROS_param('~robotId')
    missionId = ROSInterface.get_ROS_param('~missionId')
    tasks = ROSInterface.get_ROS_param(str(robotId) + '/mission_assigner/tasks')
    skills = ROSInterface.get_ROS_param(str(robotId) + '/skills')

    assignMissionServiceName = "/stamina_msgs/" + str(robotId) + "/AssignMission"

    try:
        ma = MissionAssigner(robotId, missionId, tasks, skills)
        ma.assign_mission(assignMissionServiceName)

    except Exception as e:
        rospy.logerr('[MissionAssigner] Error: %s', str(e))
        rospy.logdebug(traceback.format_exc())

    quit()
