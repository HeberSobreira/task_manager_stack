#!/usr/bin/env python

import sys
import unittest
import rostest
import rospy

from mock import patch

from task_manager_msgs.srv import *
from task_manager_msgs.msg import TaskStatus

from task_manager_class import TaskManager
from ros_interface import ROSInterface
from skill_class import *


PKG = 'task_manager'
NAME = 'integration_tests'

# Base Class for performin Task Manager Integration Tests
class IntegrationTestsBase(unittest.TestCase):

    skillsParamName = "task_manager/skills"

    def testStatusTopicCallback(self):
        pass

    # @classmethod
    # def skill_generator(self, skillName = 'example-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1']):
    #     return {'skillName': skillName, 'skillType': skillType, 'skillClass': skillClass, 'skillProperties': skillProperties}

    # @classmethod
    # def task_generator(self, skillName = 'example-skill', nSkillProperties = 2):
    #     task = {'skillName': skillName}
    #
    #     for i in range(nSkillProperties):
    #         skillPropertyKey = 'exampleSkillProperty' + str(i)
    #         skillPropertyValue = 'exampleSkillValue' + str(i)
    #         task.update({skillPropertyKey: skillPropertyValue})
    #
    #     return task

# Integration Tests for the task_manager repository
class IntegrationTests(IntegrationTestsBase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('task_manager')

    def testStatusTopicCallback(self):
        pass

    def test_skills_retrieved_from_ros_parameter_server(self):
        skills = ROSInterface.get_ROS_param(self.skillsParamName)

        self.assertEquals(len(skills), 16)

    def test_successful_generic_skill_execution(self):
        skills = ROSInterface.get_ROS_param(self.skillsParamName)
        task1 = GenericSkill(skillName = 'example-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['exampleSkillProperty0', 'exampleSkillProperty0'], skillProperties = {'exampleSkillProperty0': 'exampleSkillValue0', 'exampleSkillProperty1': 'exampleSkillValue1'})

        tm = TaskManager(robotId = 'testRobotId', skills = skills)
        tm.ongoingTasks = [task1]

        result = tm.execute_mission('fakeMissionId')

        self.assertEquals(tm.missions[0]['statusCode'], 11)
        self.assertEquals(tm.missions[0]['taskId'], 'example-skill')

    #TODO: ver o que se passa com este teste
    def test_wait_skill_execution_timeout_while_waiting_for_server(self):
        skills = ROSInterface.get_ROS_param(self.skillsParamName)
        task1 = GenericSkill(skillName = 'crob-wait', skillType = 'WaitSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['waitTime'], skillProperties = {'waitTime': 5})

        tm = TaskManager(robotId = 'testRobotId', skills = skills, waitForServerTimeOut = 2)
        tm.ongoingTasks = [task1]

        result = tm.execute_mission('fakeMissionId')

        self.assertEquals(tm.missions[0]['statusCode'], 12)
        self.assertEquals(tm.missions[0]['taskId'], 'crob-wait')
        self.assertEquals(tm.missions[0]['statusDescription'], 'Mission Failed! Task crob-wait Failed: Skill Action Client Error: Timed out (2s) while trying to find Action Server WaitSkill')

    def test_generic_skill_execution_timeout_while_waiting_for_action_response(self):
        skills = ROSInterface.get_ROS_param(self.skillsParamName)
        task1 = GenericSkill(skillName = 'example-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['exampleSkillProperty0', 'exampleSkillProperty0'], skillProperties = {'exampleSkillProperty0': 'exampleSkillValue0', 'exampleSkillProperty1': 'exampleSkillValue1'})

        tm = TaskManager(robotId = 'testRobotId', skills = skills, waitForActionClientTimeOut = 0.5)
        tm.ongoingTasks = [task1]

        result = tm.execute_mission('fakeMissionId')

        self.assertEquals(tm.missions[0]['statusCode'], 12)
        self.assertEquals(tm.missions[0]['taskId'], 'example-skill')
        self.assertEquals(tm.missions[0]['statusDescription'], 'Mission Failed! Task example-skill Failed: Skill Action Client Error: Timed out (0.5s) while waiting for GenericSkill Action Server result')

    def test_multiple_generic_skills(self):
        skills = ROSInterface.get_ROS_param(self.skillsParamName)
        task1 = GenericSkill(skillName = 'example-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['exampleSkillProperty0', 'exampleSkillProperty0'], skillProperties = {'exampleSkillProperty0': 'exampleSkillValue0', 'exampleSkillProperty1': 'exampleSkillValue1'})

        tm = TaskManager(robotId = 'testRobotId', skills = skills)
        tm.ongoingTasks = [task1, task1, task1]

        result = tm.execute_mission('fakeMissionId')

        self.assertEquals(tm.missions[0]['statusCode'], 11)
        self.assertEquals(tm.missions[0]['taskId'], 'example-skill')

    #TODO: ver o que se passa com este teste
    def test_mix_between_correct_and_incorrect_skills(self):
        skills = ROSInterface.get_ROS_param(self.skillsParamName)
        task1 = GenericSkill(skillName = 'example-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['exampleSkillProperty0', 'exampleSkillProperty0'], skillProperties = {'exampleSkillProperty0': 'exampleSkillValue0', 'exampleSkillProperty1': 'exampleSkillValue1'})
        task2 = GenericSkill(skillName = 'crob-wait', skillType = 'WaitSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['waitTime'], skillProperties = {'waitTime': 5})

        tm = TaskManager(robotId = 'testRobotId', skills = skills, waitForServerTimeOut = 2)
        tm.ongoingTasks = [task1, task2, task1]

        result = tm.execute_mission('fakeMissionId')

        self.assertEquals(tm.missions[0]['statusCode'], 12)
        self.assertEquals(tm.missions[0]['taskId'], 'crob-wait')
        self.assertEquals(tm.missions[0]['statusDescription'], 'Mission Failed! Task crob-wait Failed: Skill Action Client Error: Timed out (2s) while trying to find Action Server WaitSkill')

    def test_assignMission_service_parser_existance(self):
        tm = TaskManager(robotId = 'testRobotId', assignMissionServiceName = 'stamina_msgs/testRobotId/AssignMission')

        rospy.wait_for_service('stamina_msgs/testRobotId/AssignMission', timeout=10)

    def test_provideTaskStatus_service_parser_existance(self):
        tm = TaskManager(robotId = 'testRobotId', provideTaskStatusServiceName = 'stamina_msgs/testRobotId/ProvideTaskStatus')

        rospy.wait_for_service('stamina_msgs/testRobotId/ProvideTaskStatus', timeout=10)

    def test_taskStatusTopic_existance(self):
        tm = TaskManager(robotId = 'testRobotId', taskStatusTopic = '/stamina_msgs/TaskStatus')

        rospy.Subscriber('/stamina_msgs/TaskStatus', TaskStatus, self.testStatusTopicCallback)


class DriveEdgesSkillIntegrationTests(IntegrationTestsBase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('task_manager')

    def test_DriveEdgesSkill_contructor(self):
        driveEdgesSkillObject = DriveEdgesSkill(skillName = 'drive-edges',
                                                skillType = 'DriveEdgesSkill',
                                                skillClass = 'DriveEdgesSkill',
                                                allowedSkillPropertiesKeys = ['mode', 'edges'],
                                                skillProperties = {'mode': 'DIFFERENTIAL', 'edges': [5,6]})

        self.assertEquals(driveEdgesSkillObject.skillName, 'drive-edges')
        self.assertEquals(driveEdgesSkillObject.skillType, 'DriveEdgesSkill')
        self.assertEquals(driveEdgesSkillObject.skillClass, 'DriveEdgesSkill')
        self.assertEquals(driveEdgesSkillObject.allowedSkillPropertiesKeys, ['mode', 'edges'])
        self.assertEquals(driveEdgesSkillObject.skillProperties['mode'], 'DIFFERENTIAL')
        self.assertEquals(driveEdgesSkillObject.skillProperties['edges'], [5,6])

    def test_DriveEdgesSkill_actionGoalConstructor(self):
        driveEdgesSkillObject = DriveEdgesSkill(skillName = 'drive-edges',
                                                skillType = 'DriveEdgesSkill',
                                                skillClass = 'DriveEdgesSkill',
                                                allowedSkillPropertiesKeys = ['mode', 'edges'],
                                                skillProperties = {'mode': 'DIFFERENTIAL', 'edges': [5,6]})

        driveEdgesSkillObject.actionGoalConstructor()


# Test Suite for Mission Assigner
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        integrationTests = loader.loadTestsFromTestCase(IntegrationTests)
        driveEdgesSkillIntegrationTests = loader.loadTestsFromTestCase(DriveEdgesSkillIntegrationTests)

        self.addTests(integrationTests)
        self.addTests(driveEdgesSkillIntegrationTests)

if __name__ == '__main__':
      rostest.rosrun(PKG, NAME, 'integration_tests.SuiteTest', sys.argv)
