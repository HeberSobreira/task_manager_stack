#!/usr/bin/env python

import sys
import rospy

from task_manager_msgs.srv import AssignMission

from skill_class import Skill
from msg_constructor import MSGConstructor

class MissionAssigner(object):

    """docstring for MissionAssigner."""


    def __init__(self, robotId = None, missionId = None, tasks = None, priority = None, skills = None):
        self.robotId = robotId if robotId is not None else 'defaultRobotId'
        self.missionId = missionId if missionId is not None else 'defaultMissionId'
        self.tasks = tasks
        self.priority = priority
        self.skills = skills

    def goal_list_constructor(self, tasks, skills):

        goalList = []

        ################## TODO: Create function in common ##################
        allowedSkills = []
        [allowedSkills.append(s['skillName']) for s in skills]
        #####################################################################

        for task in tasks:

            # If the skill is known...
            if any(skill['skillName'] == task['skillName'] for skill in skills):
                for skill in skills:

                    if skill['skillName'] == task['skillName']:
                        s = Skill(skillName = skill['skillName'], skillType = skill['skillType'], allowedSkillPropertiesKeys = skill['skillProperties'])
                        s.skillPropertiesConstructor(task)
                        goalList.append(s.goal_encoder())
                        break

            # If the skill is not known...
            else:
                raise KeyError('Skill ' + str(task['skillName']) + ' not known! Allowed Skills are: ' + str(allowedSkills))

        return goalList

    def service_response_printer(self, serviceResponse):

        if serviceResponse.accepted in ['True', 'true', 'Yes', 'yes', 'y', '1', 1, True]:
            return '[MissionAssigner] [' + str(self.robotId) + '] [' + str(self.missionId) + '] Mission Accepted!'
        elif serviceResponse.accepted in ['False', 'false', 'No', 'no', 'n', '0', 0, False]:
            return '[MissionAssigner] [' + str(self.robotId) + '] [' + str(self.missionId) + '] Mission Refused! ' + str(serviceResponse.reason_of_refusal)
        else:
            return '[MissionAssigner] [' + str(self.robotId) + '] [' + str(self.missionId) + '] Mission status unknown!'

    def assign_mission(self, assignMissionServiceName):
        rospy.loginfo("[MissionAssigner] [%s] [%s] Constructing goalList...", str(self.robotId), str(self.missionId))

        goalsList = self.goal_list_constructor(self.tasks, self.skills)

        rospy.loginfo("[MissionAssigner] [%s] [%s] Waiting for AssignMission Service at %s", str(self.robotId), str(self.missionId), str(assignMissionServiceName))

        rospy.wait_for_service(assignMissionServiceName)

        rospy.loginfo("[MissionAssigner] [%s] [%s] Service Found! Sending Request...", str(self.robotId), str(self.missionId))

        AssignMissionServiceProxy = rospy.ServiceProxy(assignMissionServiceName, AssignMission)
        AssignMissionServiceResponse = AssignMissionServiceProxy(self.missionId, self.robotId, goalsList, self.priority)

        rospy.loginfo(self.service_response_printer(AssignMissionServiceResponse.actionAcceptedRefused))

        return AssignMissionServiceResponse
