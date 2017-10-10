#!/usr/bin/env python

import sys
import unittest
import rostest

from mock import patch

from task_manager_msgs.msg import ActionAcceptedRefused

from mission_assigner_class import MissionAssigner
from skill_class import Skill
from msg_constructor import MSGConstructor

PKG = 'mission_assigner'
NAME = 'test_mission_assigner'

# Unit Tests for Mission Assigner
class TestMissionAssigner(unittest.TestCase):

    def test_create_empty_object(self):
        ma = MissionAssigner()

    def test_create_object_with_robotId(self):
        ma = MissionAssigner("testRobotId")
        self.assertEquals(ma.robotId, "testRobotId")

    def test_create_object_with_robotId_and_missionId(self):
        ma = MissionAssigner("testRobotId", "testMissionId")
        self.assertEquals(ma.robotId, "testRobotId")
        self.assertEquals(ma.missionId, "testMissionId")

    def test_validate_default_robotId_and_missionId(self):
        ma = MissionAssigner()
        self.assertIs(type(ma.robotId), str)
        self.assertIs(type(ma.missionId), str)

    def test_service_response_printer_accepted_mission(self):
        ma = MissionAssigner()

        serviceResponse = ActionAcceptedRefused()
        serviceResponse.accepted = 'True'
        serviceResponse.reason_of_refusal = 'false'

        serviceResponsePrint = ma.service_response_printer(serviceResponse)
        expectedPrint = '[MissionAssigner] [' + str(ma.robotId) + '] [' + str(ma.missionId) + '] Mission Accepted!'

        self.assertEquals(serviceResponsePrint, expectedPrint)

    def test_service_response_printer_refused_mission(self):
        ma = MissionAssigner()

        serviceResponse = ActionAcceptedRefused()
        serviceResponse.accepted = 'False'
        serviceResponse.reason_of_refusal = 'Reason!'

        serviceResponsePrint = ma.service_response_printer(serviceResponse)
        expectedPrint = '[MissionAssigner] [' + str(ma.robotId) + '] [' + str(ma.missionId) + '] Mission Refused! Reason!'

        self.assertEquals(serviceResponsePrint, expectedPrint)

    def test_service_response_printer_unknown_mission_status(self):
        ma = MissionAssigner()

        serviceResponse = ActionAcceptedRefused()
        serviceResponse.accepted = None
        serviceResponse.reason_of_refusal = None

        serviceResponsePrint = ma.service_response_printer(serviceResponse)
        expectedPrint = '[MissionAssigner] [' + str(ma.robotId) + '] [' + str(ma.missionId) + '] Mission status unknown!'

        self.assertEquals(serviceResponsePrint, expectedPrint)

# Unit Tests for Goal List Constructor
class TestGoalListConstructor(unittest.TestCase):

    #################
    ## CLASS SETUP ##
    #################

    @classmethod
    def setUpClass(cls):
        cls.ma = MissionAssigner()

    #############
    ## METHODS ##
    #############

    def example_skill(self, skillName = 'example-skill', skillType = 'exampleSkillType', skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1']):
        return {'skillName': skillName, 'skillType': skillType, 'skillProperties': skillProperties}

    def example_task(self, skillName = 'example-skill', nSkillProperties = 2):
        task = {'skillName': skillName}

        for i in range(nSkillProperties):
            skillPropertyKey = 'exampleSkillProperty' + str(i)
            skillPropertyValue = 'exampleSkillValue' + str(i)
            task.update({skillPropertyKey: skillPropertyValue})

        return task

    def assertRaisesTypeError(self, tasks, skills):
        with self.assertRaises(TypeError):
            goalList = self.ma.goal_list_constructor(tasks, skills)

    def assertRaisesKeyError(self, tasks, skills):
        with self.assertRaises(KeyError):
            goalList = self.ma.goal_list_constructor(tasks, skills)

    ###################
    ## UNITARY TESTS ##
    ###################

    def test_tasks_empty_list(self):
        tasks = []
        skills = [self.example_skill()]

        goalList = self.ma.goal_list_constructor(tasks, skills)

        self.assertIsNotNone(goalList)

    def test_skills_empty_list(self):
        tasks = [self.example_task()]
        skills = []

        self.assertRaisesKeyError(tasks, skills)

    def test_tasks_not_a_list(self):
        tasks = "abc"
        skills = [self.example_skill()]

        self.assertRaisesTypeError(tasks, skills)

    def test_skills_not_a_list(self):
        tasks = [self.example_task()]
        skills = "I'm not a list!"

        self.assertRaisesTypeError(tasks, skills)

    def test_tasks_not_a_list_of_dictionaries(self):
        tasks = ["abc", "def"]
        skills = [self.example_skill()]

        self.assertRaisesTypeError(tasks, skills)

    def test_skills_not_a_list_of_dictionaries(self):
        tasks = [self.example_task()]
        skills = ["I'm not a dictionary", "Me neither!"]

        self.assertRaisesTypeError(tasks, skills)

    def test_task_do_not_contain_skillName_key(self):
        tasks = [{"abc": 'def'}]
        skills = [self.example_skill()]

        self.assertRaisesKeyError(tasks, skills)

    def test_skill_do_not_contain_skillName_key(self):
        tasks = [self.example_task()]
        skills = [{'abd': 'def'}]

        self.assertRaisesKeyError(tasks, skills)

    def test_task_contain_a_skill_known(self):
        tasks = [self.example_task('test-skill')]
        skills = [self.example_skill('test-skill')]

        goalList = self.ma.goal_list_constructor(tasks, skills)

        self.assertTrue(len(goalList), 1)

    def test_task_contain_multiple_known_skills(self):
        tasks = [self.example_task('test-skill-1'), self.example_task('test-skill-2')]
        skills = [self.example_skill('test-skill-1'), self.example_skill('test-skill-2'), self.example_skill('test-skill-3')]

        goalList = self.ma.goal_list_constructor(tasks, skills)

        self.assertTrue(len(goalList), 2)

    def test_task_and_skill_from_troublesome_scenario(self):
        tasks = [{'toObjectId': '1234abcd', 'skillName': 'crob-drive', 'toObjectType': 'xpto'}]
        skills = [{'skillName': 'crob-drive', 'skillProperties': ['toObjectId', 'toObjectType'], 'skillType': 'DriveSkill'}]

        goalList = self.ma.goal_list_constructor(tasks, skills)

        self.assertTrue(len(goalList), 1)

    def test_task_contain_a_unknown_skill(self):
        tasks = [self.example_task('test-skill')]
        skills = [self.example_skill('unknown-skill')]

        self.assertRaisesKeyError(tasks, skills)

# Test Suite for Mission Assigner
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        testsMissionAssigner = loader.loadTestsFromTestCase(TestMissionAssigner)
        testGoalListConstructor = loader.loadTestsFromTestCase(TestGoalListConstructor)

        self.addTests(testsMissionAssigner)
        self.addTests(testGoalListConstructor)

if __name__ == '__main__':
      rostest.rosrun(PKG, NAME, 'test_mission_assigner.SuiteTest', sys.argv)
