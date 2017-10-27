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


class TestTaskStatusConstructor(TestMSGConstructorBase):
    def test_TaskStatusConstructor_empty_input(self):
        taskStatus = MSGConstructor.TaskStatusConstructor()
        self.assertIsInstance(taskStatus, TaskStatus)

    def test_TaskStatusConstructor_empty_input_values(self):
        taskStatus = MSGConstructor.TaskStatusConstructor()
        self.assertEquals(taskStatus.missionId, 'defaultMissionId')
        self.assertEquals(taskStatus.taskId, 'defaultTaskId')
        self.assertEquals(taskStatus.statusCode, 0)
        self.assertEquals(taskStatus.statusDescription, 'defaultStatusDescription')
        self.assertEquals(taskStatus.when, None)

    def test_TaskStatusConstructor_input_values(self):
        taskStatus = MSGConstructor.TaskStatusConstructor('testMissionId','testTaskId', 1, 'testStatusDescription', rospy.Time(10))
        self.assertEquals(taskStatus.missionId, 'testMissionId')
        self.assertEquals(taskStatus.taskId, 'testTaskId')
        self.assertEquals(taskStatus.statusCode, 1)
        self.assertEquals(taskStatus.statusDescription, 'testStatusDescription')
        self.assertEquals(taskStatus.when.secs, 10)

    def test_TaskStatusConstructor_extra_arguments(self):
        with self.assertRaises(TypeError):
            taskStatus = MSGConstructor.TaskStatusConstructor('testMissionId','testTaskId', 1, 'testStatusDescription', rospy.Time(10), 'testArgument')



class TestPoseStampedConstructor(TestMSGConstructorBase):

    def test_PoseStampedConstructor_inputs(self):
        poseStampedObject = MSGConstructor.PoseStampedConstructor('testFrameID', 1, 2, 3, 4, 5, 6, 7)
        self.assertEqual(poseStampedObject.header.frame_id,'testFrameID')
        self.assertEqual(poseStampedObject.pose.position.x, 1)
        self.assertEqual(poseStampedObject.pose.position.y, 2)
        self.assertEqual(poseStampedObject.pose.position.z, 3)
        self.assertEqual(poseStampedObject.pose.orientation.x, 4)
        self.assertEqual(poseStampedObject.pose.orientation.y, 5)
        self.assertEqual(poseStampedObject.pose.orientation.z, 6)
        self.assertEqual(poseStampedObject.pose.orientation.w, 7)

    def test_PoseStampedConstructor_empty_input(self):
        with self.assertRaises(TypeError):
            poseStampedObject = MSGConstructor.PoseStampedConstructor()

    def test_PoseStampedConstructor_missing_arguments(self):
        with self.assertRaises(TypeError):
            poseStampedObject = MSGConstructor.PoseStampedConstructor('testFrameID', 1, 2)

    def test_PoseStampedConstructor_extra_arguments(self):
        with self.assertRaises(TypeError):
            poseStampedObject = MSGConstructor.PoseStampedConstructor('testFrameID', 1, 2, 3, 4, 5, 6, 7, 8)

# Test Suite for Mission Assigner
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        testActionAcceptedRefusedConstructor = loader.loadTestsFromTestCase(TestActionAcceptedRefusedConstructor)
        testPoseStampedConstructor = loader.loadTestsFromTestCase(TestPoseStampedConstructor)
        testTaskStatusConstructor = loader.loadTestsFromTestCase(TestTaskStatusConstructor)

        self.addTests(testActionAcceptedRefusedConstructor)
        self.addTests(testPoseStampedConstructor)
        self.addTests(testTaskStatusConstructor)

if __name__ == '__main__':
      rostest.rosrun(PKG, NAME, 'test_msg_constructor.SuiteTest', sys.argv)
