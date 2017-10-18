#!/usr/bin/env python

import sys
import unittest
import rostest

# from mock import patch

# from task_manager_msgs.srv import *
#
# from task_manager_class import TaskManager
# from skill_class import *
#
# from msg_constructor import MSGConstructor

PKG = 'skill_simulation_server'
NAME = 'test_skill_simulation_server'

# Base Class for testing Skill Simulation skill_simulation_server
class TestSkillSimulationServerBase(unittest.TestCase):
    pass

class TestSkillSimulationServer(TestSkillSimulationServerBase):
    def test_skill(self):
        var = 1+1
        self.assertEquals(var, 2)


class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()
        loader = unittest.TestLoader()

        testSkillSimulationServer = loader.loadTestsFromTestCase(TestSkillSimulationServer)

        self.addTests(testSkillSimulationServer)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, 'test_skill_simulation_server.SuiteTest', sys.argv)
