#!/usr/bin/env python

import sys
import unittest
import rostest

from mock import patch

from ros_interface import *


PKG = 'task_manager_common'
NAME = 'test_ros_interface'

# Base Class for testing ROS Interface Class
class TestMSGConstructorBase(unittest.TestCase):
    pass

# Unit Tests for Defining Param Names
class TestParamNames(TestMSGConstructorBase):
    pass

# Test Suite for Mission Assigner
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        testParamNames = loader.loadTestsFromTestCase(TestParamNames)

        self.addTests(testParamNames)

if __name__ == '__main__':
      rostest.rosrun(PKG, NAME, 'test_ros_interface.SuiteTest', sys.argv)
