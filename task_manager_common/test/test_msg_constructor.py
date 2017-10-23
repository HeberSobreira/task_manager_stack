#!/usr/bin/env python

import sys
import unittest
import rostest

from mock import patch

from msg_constructor import *

PKG = 'task_manager_common'
NAME = 'test_msg_constructor'

# Base Class for testing Message Constructor Class
class TestMSGConstructorBase(unittest.TestCase):
    pass

# Unit Tests for Action Accepted Refused Constructor
class TestActionAcceptedRefusedConstructor(TestMSGConstructorBase):

    def test_construct_accepted_response(self):
        response = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'True', reasonOfRefusal = 'None')

        self.assertEquals(response.accepted, 'True')
        self.assertEquals(response.reason_of_refusal, 'None')

    def test_construct_refused_response(self):
        response = MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'False', reasonOfRefusal = 'Empty Plan!')

        self.assertEquals(response.accepted, 'False')
        self.assertEquals(response.reason_of_refusal, 'Empty Plan!')

    def test_raise_exception_empty_input(self):
        with self.assertRaises(TypeError):
            response = MSGConstructor.ActionAcceptedRefusedConstructor()


class TestPoseStampedConstructor(TestMSGConstructorBase):

    def test_constructor_inputs(self):
        poseStampedObject = MSGConstructor.PoseStampedConstructor('testFrameID', 1, 2, 3, 4, 5, 6, 7)
        self.assertEqual(poseStampedObject.header.frame_id,'testFrameID')
        self.assertEqual(poseStampedObject.pose.position.x, 1)
        self.assertEqual(poseStampedObject.pose.position.y, 2)
        self.assertEqual(poseStampedObject.pose.position.z, 3)
        self.assertEqual(poseStampedObject.pose.orientation.x, 4)
        self.assertEqual(poseStampedObject.pose.orientation.y, 5)
        self.assertEqual(poseStampedObject.pose.orientation.z, 6)
        self.assertEqual(poseStampedObject.pose.orientation.w, 7)

        #TODO: fazer mais testes para esta class

# Test Suite for Mission Assigner
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        testActionAcceptedRefusedConstructor = loader.loadTestsFromTestCase(TestActionAcceptedRefusedConstructor)
        testPoseStampedConstructor = loader.loadTestsFromTestCase(TestPoseStampedConstructor)

        self.addTests(testActionAcceptedRefusedConstructor)
        self.addTests(testPoseStampedConstructor)

if __name__ == '__main__':
      rostest.rosrun(PKG, NAME, 'test_msg_constructor.SuiteTest', sys.argv)
