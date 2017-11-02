#!/usr/bin/env python

import sys
import unittest
import rostest

from path_assigner_class import PathAssigner


PKG = 'path_assigner'
NAME = 'test_path_assigner_class'

# Unit Test for Path Assigner Class
class TestPathAssignerClass(unittest.TestCase):

    def test_create_empty_object(self):
        pa = PathAssigner()

    # TODO: Develop more tests!

# Test Suite for Path Assigner
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        testsPathAssignerClass = loader.loadTestsFromTestCase(TestPathAssignerClass)

        self.addTests(testsPathAssignerClass)

if __name__ == '__main__':
      rostest.rosrun(PKG, NAME, 'test_path_assigner_class.SuiteTest', sys.argv)
