#!/usr/bin/env python

import sys
import unittest
import rostest
import rospy

from task_manager_msgs.msg import ActionAcceptedRefused

from mission_assigner_class import MissionAssigner
from ros_interface import ROSInterface

PKG = 'mission_assigner'
NAME = 'integration_tests'

# Base Class for performin Mission Assigner Integration Tests
class IntegrationTestsBase(unittest.TestCase):

    tasksParamName = "/testRobotId/mission_assigner/tasks"
    skillsParamName = "/testRobotId/skills"
    assignMissionServiceName = '/stamina_msgs/testRobotId/AssignMission'

    @classmethod
    def skill_generator(self, skillName = 'example-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1']):
        return {'skillName': skillName, 'skillType': skillType, 'skillClass': skillClass, 'skillProperties': skillProperties}

    @classmethod
    def task_generator(self, skillName = 'example-skill', nSkillProperties = 2):
        task = {'skillName': skillName}

        for i in range(nSkillProperties):
            skillPropertyKey = 'exampleSkillProperty' + str(i)
            skillPropertyValue = 'exampleSkillValue' + str(i)
            task.update({skillPropertyKey: skillPropertyValue})

        return task

# Integration Tests for the mission_assigner repository
class IntegrationTests(IntegrationTestsBase):

    def test_task_retrieved_from_ros_parameter_server(self):
        tasks = ROSInterface.get_ROS_param(self.tasksParamName)

        self.assertEquals(len(tasks), 11)

    def test_skills_retrieved_from_ros_parameter_server(self):
        skills = ROSInterface.get_ROS_param(self.skillsParamName)

        self.assertEquals(len(skills), 16)

    def test_found_AssignMission_service(self):
        rospy.wait_for_service(self.assignMissionServiceName)

    def test_example_AssignMission_service_request(self):
        rospy.wait_for_service(self.assignMissionServiceName)

        tasks = [self.task_generator()]
        skills = [self.skill_generator()]

        ma = MissionAssigner(robotId = "testRobotId", missionId = "testMissionId", tasks = tasks, skills = skills)
        AssignMissionServiceResponse = ma.assign_mission(self.assignMissionServiceName)

        self.assertEquals(AssignMissionServiceResponse.actionAcceptedRefused.accepted, 'True')

    def test_assign_an_empty_mission(self):
        rospy.wait_for_service(self.assignMissionServiceName)

        tasks = []
        skills = [self.skill_generator()]

        ma = MissionAssigner(robotId = "testRobotId", missionId = "testMissionId", tasks = tasks, skills = skills)
        AssignMissionServiceResponse = ma.assign_mission(self.assignMissionServiceName)

        self.assertEquals(AssignMissionServiceResponse.actionAcceptedRefused.accepted, 'False')

    def test_assign_mission_function(self):
        rospy.wait_for_service(self.assignMissionServiceName)

        tasks = ROSInterface.get_ROS_param(self.tasksParamName)
        skills = ROSInterface.get_ROS_param(self.skillsParamName)

        ma = MissionAssigner(robotId = "testRobotId", missionId = "testMissionId", tasks = tasks, skills = skills)
        AssignMissionServiceResponse = ma.assign_mission(self.assignMissionServiceName)

        self.assertEquals(AssignMissionServiceResponse.actionAcceptedRefused.reason_of_refusal, 'None')
        self.assertEquals(AssignMissionServiceResponse.actionAcceptedRefused.accepted, 'True')


# Test Suite for Mission Assigner
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        integrationTests = loader.loadTestsFromTestCase(IntegrationTests)

        self.addTests(integrationTests)

if __name__ == '__main__':
      rostest.rosrun(PKG, NAME, 'integration_tests.SuiteTest', sys.argv)
