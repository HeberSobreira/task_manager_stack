#! /usr/bin/env python

import roslib
import rospy
import sys
import actionlib

from skill_action_server_class import *


def skill_action_server():

    # sys.argv[0] = /home/tiago/catkin_ws/src/task_manager_stack/skill_simulation_server/src/skill_action_server.py
    # sys.argv[1] = GenericSkill
    # sys.argv[2] = __name:=genericSkill
    # sys.argv[3] = __log:=/home/tiago/.ros/log/fa61ca7a-b583-11e7-848e-aced5c24834e/genericSkill-1.log

    rospy.init_node('skill_simulation_server')
    skillName = rospy.get_name()[1:]; #remove the initial '/'

    try:
        skill_simulation = SkillActionBase(rospy.get_name(), str(sys.argv[1]))
        rospy.loginfo("[%s] Starting Fake Action Server!", skillName)
    except:
        rospy.loginfo("[%s] Fake Skill Not Available!", str(sys.argv[1]))

    rospy.spin()


if __name__ == '__main__':
    skill_action_server()
