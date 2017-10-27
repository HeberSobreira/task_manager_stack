#!/usr/bin/env python

import sys
import unittest
import rostest

from mock import patch

from task_manager_msgs.srv import *

from task_manager_class import TaskManager
from skill_class import *

from msg_constructor import MSGConstructor


PKG = 'task_manager_server'
NAME = 'test_task_manager_class'

# Base Class for testing Task Manager Class
class TestTaskManagerClassBase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('task_manager')

    @classmethod
    def skill_generator(self, skillName = 'example-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1']):
        return {'skillName': skillName, 'skillType': skillType, 'skillClass': skillClass, 'skillProperties': skillProperties}

    @classmethod
    def test_goal_generator(self):
        return 'example-skill;exampleSkillProperty0=exampleSkillValue0;exampleSkillProperty1=exampleSkillValue1'

# Unit Tests for Task Manager
class TestTaskManager(TestTaskManagerClassBase):

    def test_create_empty_object(self):
        tm = TaskManager()

    def test_create_object_with_robotId(self):
        tm = TaskManager(robotId = 'testRobotId')

        self.assertEquals(tm.robotId, 'testRobotId')

    def test_create_object_with_skills(self):
        tm = TaskManager(skills = [self.skill_generator()])

        self.assertEquals(tm.skills, [self.skill_generator()])

    def test_create_object_with_waitForServerTimeOut(self):
        tm = TaskManager(waitForServerTimeOut = 10)

        self.assertEquals(tm.waitForServerTimeOut, 10)

    def test_create_object_with_waitForActionClientTimeOut(self):
        tm = TaskManager(waitForActionClientTimeOut = 100)

        self.assertEquals(tm.waitForActionClientTimeOut, 100)

# Unit Tests for Assign Mission Service Handler
class TestAssignMissionServiceHandler(TestTaskManagerClassBase):

    def test_refused_because_different_robotId(self):
        tm = TaskManager(skills = [self.skill_generator()])

        response = tm.assign_mission_service_handler(robotId = 'differentRobotId')
        refusedResponse = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'False', reasonOfRefusal = 'Different robotId!')

        self.assertEquals(response, refusedResponse)
        self.assertEquals(len(tm.ongoingTasks), 0)

    def test_refused_because_empty_goals(self):
        tm = TaskManager(skills = [self.skill_generator()])

        response = tm.assign_mission_service_handler(goals = [])
        refusedResponse = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'False', reasonOfRefusal = 'Empty goals!')

        self.assertEquals(response, refusedResponse)
        self.assertEquals(len(tm.ongoingTasks), 0)

    def test_refused_bad_syntax_goal(self):
        tm = TaskManager(skills = [self.skill_generator()])

        response = tm.assign_mission_service_handler(goals = ['example-skill;exampleSkillProperty0'])
        refusedResponse = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'False', reasonOfRefusal = 'Badly formatted goal (example-skill;exampleSkillProperty0): need more than 1 value to unpack')

        self.assertEquals(response, refusedResponse)
        self.assertEquals(len(tm.ongoingTasks), 0)

    def test_accepted_simple_mission(self):
        tm = TaskManager(skills = [self.skill_generator()])

        response = tm.assign_mission_service_handler(goals = [self.test_goal_generator()])
        acceptedResponse = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'True', reasonOfRefusal = 'None')

        self.assertEquals(response, acceptedResponse)
        self.assertEquals(len(tm.ongoingTasks), 1)

    def test_accepted_complex_mission(self):
        tm = TaskManager(skills = [self.skill_generator(), self.skill_generator(skillName = 'drive', skillType = 'DriveSkill', skillClass = 'DriveSkill', skillProperties = ['toObjectId', 'toObjectType'])])

        response = tm.assign_mission_service_handler(goals = [self.test_goal_generator(), 'drive;toObjectId=testToObjectId;toObjectType=testToObjectType'])
        acceptedResponse = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'True', reasonOfRefusal = 'None')

        self.assertEquals(response, acceptedResponse)
        self.assertEquals(len(tm.ongoingTasks), 2)

    def test_append_multiple_tasks_to_tasks_list(self):
        tm = TaskManager(skills = [self.skill_generator()])

        response = tm.assign_mission_service_handler(goals = [self.test_goal_generator(), self.test_goal_generator(), self.test_goal_generator()])
        acceptedResponse = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'True', reasonOfRefusal = 'None')

        self.assertEquals(response, acceptedResponse)
        self.assertEquals(len(tm.ongoingTasks), 3)

    def test_refuse_mission_then_accept_mission(self):
        tm = TaskManager(skills = [self.skill_generator()])

        response = tm.assign_mission_service_handler(goals = ['example-skill;exampleSkillProperty0'])
        refusedResponse = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'False', reasonOfRefusal = 'Badly formatted goal (example-skill;exampleSkillProperty0): need more than 1 value to unpack')

        self.assertEquals(response, refusedResponse)
        self.assertEquals(len(tm.ongoingTasks), 0)

        response = tm.assign_mission_service_handler(goals = [self.test_goal_generator(), self.test_goal_generator(), self.test_goal_generator()])
        acceptedResponse = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'True', reasonOfRefusal = 'None')

        self.assertEquals(response, acceptedResponse)
        self.assertEquals(len(tm.ongoingTasks), 3)

    def test_refuse_mission_because_robot_is_busy(self):
        tm = TaskManager(skills = [self.skill_generator()])

        response = tm.assign_mission_service_handler(goals = [self.test_goal_generator(), self.test_goal_generator(), self.test_goal_generator()])
        acceptedResponse = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'True', reasonOfRefusal = 'None')

        self.assertEquals(response, acceptedResponse)
        self.assertEquals(len(tm.ongoingTasks), 3)

        response = tm.assign_mission_service_handler(goals = [self.test_goal_generator(), self.test_goal_generator(), self.test_goal_generator()])
        refusedResponse = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'False', reasonOfRefusal = "Robot is currently busy with 3 tasks: ['example-skill', 'example-skill', 'example-skill']")

        self.assertEquals(response, refusedResponse)
        self.assertEquals(len(tm.ongoingTasks), 3)

    @patch('task_manager_class.TaskManager.execute_mission')
    def test_execute_mission_called_for_simple_mission(self, mock):
        tm = TaskManager(skills = [self.skill_generator()])

        response = tm.assign_mission_service_handler(goals = [self.test_goal_generator(), self.test_goal_generator(), self.test_goal_generator()], executeMission = True)
        acceptedResponse = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'True', reasonOfRefusal = 'None')

        self.assertEquals(response, acceptedResponse)
        self.assertEquals(len(tm.ongoingTasks), 3)
        self.assertTrue(mock.called)

# Unit Tests for Execute Mission
class TestExecuteMission(TestTaskManagerClassBase):

    @patch('task_manager_class.TaskManager.execute_mission')
    def test_method_callable(self, mock):
        tm = TaskManager(skills = self.skill_generator())
        tm.ongoingTasks = [Skill()]

        tm.execute_mission()

        self.assertTrue(mock.called)

    @patch('skill_class.Skill.action_client')
    def test_correct_task_action_client_called(self, mock):
        tm = TaskManager(skills = [self.skill_generator()])
        task1 = GenericSkill(skillName = 'example-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['exampleSkillProperty0', 'exampleSkillProperty0'], skillProperties = {'exampleSkillProperty0': 'exampleSkillValue0', 'exampleSkillProperty1': 'exampleSkillValue1'})
        tm.ongoingTasks = [task1]

        tm.execute_mission('fakeMissionId')

        self.assertTrue(mock.called)

    ## TODO, IDEA: Can mock be used to simulate the behavior of the action_client, for testing purposes?

# Unit Tests for Provide Task Status
class TestProvideTaskStatus(TestTaskManagerClassBase):

    pass


# Test Suite for Mission Assigner
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        testsTaskManager = loader.loadTestsFromTestCase(TestTaskManager)
        testsAssignMissionServiceHandler = loader.loadTestsFromTestCase(TestAssignMissionServiceHandler)
        testExecuteMission = loader.loadTestsFromTestCase(TestExecuteMission)
        testProvideTaskStatus = loader.loadTestsFromTestCase(TestProvideTaskStatus)

        self.addTests(testsTaskManager)
        self.addTests(testsAssignMissionServiceHandler)
        self.addTests(testExecuteMission)
        self.addTests(testProvideTaskStatus)

if __name__ == '__main__':
      rostest.rosrun(PKG, NAME, 'test_task_manager_class.SuiteTest', sys.argv)
