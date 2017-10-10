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

# Unit Tests for Part To Pick Constructor
class TestPartToPickConstructor(TestMSGConstructorBase):

    @patch('msg_constructor.MSGConstructor.PartToPickConstructor')
    def test_part_to_pick_constructor_selection(self, mock):
        partToPick = MSGConstructor.PartToPickConstructor()
        self.assertTrue(mock.called)

    def test_validate_default_part_to_pick_constructor(self):
        partToPick = MSGConstructor.PartToPickConstructor()

        self.assertEquals(partToPick.cellId, 'defaultCellId')
        self.assertEquals(partToPick.partId, 'defaultPartId')
        self.assertEquals(partToPick.quantity, 1)

    def test_custom_part_to_pick_constructor(self):
        partToPick = MSGConstructor.PartToPickConstructor('exampleCellId', 'examplePartId', 2)

        self.assertEquals(partToPick.cellId, 'exampleCellId')
        self.assertEquals(partToPick.partId, 'examplePartId')
        self.assertEquals(partToPick.quantity, 2)

# Unit Tests for Kitting Order Constructor
class TestKittingOrderConstructor(TestMSGConstructorBase):

    @patch('msg_constructor.MSGConstructor.KittingOrderConstructor')
    def test_kitting_order_constructor(self, mock):
        kittingOrder = MSGConstructor.KittingOrderConstructor()
        self.assertTrue(mock.called)

    def test_validate_default_kitting_order_constructor(self):
        kittingOrder = MSGConstructor.KittingOrderConstructor()

        self.assertEquals(kittingOrder.id, 'defaultId')
        self.assertEquals(kittingOrder.carSeqNumber, 'defaultCarSeqNumber')
        self.assertEquals(kittingOrder.kitId, 'defaultKitId')
        self.assertEquals(len(kittingOrder.partsToPick), 1)

    def test_custom_kitting_order_constructor(self):
        part1 = MSGConstructor.PartToPickConstructor('exampleCellId1', 'examplePartId1', 1)
        part2 = MSGConstructor.PartToPickConstructor('exampleCellId2', 'examplePartId2', 2)
        part3 = MSGConstructor.PartToPickConstructor('exampleCellId3', 'examplePartId3', 3)

        parts = []
        parts.append(part1)
        parts.append(part2)
        parts.append(part3)

        kittingOrder = MSGConstructor.KittingOrderConstructor('exampleId', 'exampleCarSeqNumber', 'exampleKitId', parts)

        self.assertEquals(kittingOrder.id, 'exampleId')
        self.assertEquals(kittingOrder.carSeqNumber, 'exampleCarSeqNumber')
        self.assertEquals(kittingOrder.kitId, 'exampleKitId')
        self.assertEquals(len(kittingOrder.partsToPick), 3)

# Unit Tests for Kitting Order List Constructor
class TestKittingOrderListConstructor(TestMSGConstructorBase):

    @patch('msg_constructor.MSGConstructor.KittingOrderListConstructor')
    def test_kitting_order_list_constructor_selection(self, mock):
        kittingOrderList = MSGConstructor.KittingOrderListConstructor()
        self.assertTrue(mock.called)

    def test_validate_default_kitting_order_list_constructor(self):
        kittingOrderList = MSGConstructor.KittingOrderListConstructor()

        self.assertEquals(len(kittingOrderList), 1)

    def test_custom_kitting_order_list_constructor(self):
        kittingOrder = MSGConstructor.KittingOrderConstructor()

        kittingOrders = []
        kittingOrders.append(kittingOrder)
        kittingOrders.append(kittingOrder)

        kittingOrderList = MSGConstructor.KittingOrderListConstructor(kittingOrders)

        self.assertEquals(len(kittingOrderList), 2)


# Test Suite for Mission Assigner
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        testActionAcceptedRefusedConstructor = loader.loadTestsFromTestCase(TestActionAcceptedRefusedConstructor)
        testPartToPickConstructor = loader.loadTestsFromTestCase(TestPartToPickConstructor)
        testKittingOrderConstructor = loader.loadTestsFromTestCase(TestKittingOrderConstructor)
        testKittingOrderListConstructor = loader.loadTestsFromTestCase(TestKittingOrderListConstructor)

        self.addTests(testActionAcceptedRefusedConstructor)
        self.addTests(testPartToPickConstructor)
        self.addTests(testKittingOrderConstructor)
        self.addTests(testKittingOrderListConstructor)

if __name__ == '__main__':
      rostest.rosrun(PKG, NAME, 'test_msg_constructor.SuiteTest', sys.argv)
