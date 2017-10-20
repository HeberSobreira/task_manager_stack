#!/usr/bin/env python

import sys
import unittest
import rostest
import actionlib


from skill_action_server_class import *

PKG = 'skill_simulation_server'
NAME = 'test_skill_action_server_class'


# Base Class for testing Skill Simulation skill_simulation_server
class TestSkillSimulationServerBase(unittest.TestCase):
    pass


class TestSkillActionBase(TestSkillSimulationServerBase):

    def test_create_object_with_name_and_skills(self):
        skill_action_object = SkillActionBase(name = 'testName', skillType = 'DriveSkill')
        self.assertTrue(skill_action_object._action_name == 'testName' and skill_action_object.skillType == 'DriveSkill')


    def test_execute_cb(self):
        skill_action_object = SkillActionBase(name = 'testName', skillType = 'DriveSkill')
        skill_action_object.execute_cb


class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()
        loader = unittest.TestLoader()

        testSkillActionBase = loader.loadTestsFromTestCase(TestSkillActionBase)

        self.addTests(testSkillActionBase)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, 'test_skill_action_server_class.SuiteTest', sys.argv)
