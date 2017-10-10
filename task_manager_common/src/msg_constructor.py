#!/usr/bin/env python

import sys
import rospy

from task_manager_msgs.msg import ActionAcceptedRefused
from task_manager_msgs.msg import PartToPick
from task_manager_msgs.msg import KittingOrder
from task_manager_msgs.msg import TaskStatus


# Class for ROS Message Construction
class MSGConstructor(object):

    """docstring for MSGConstructor."""

    @classmethod
    def ActionAcceptedRefusedConstructor(self, accepted, reasonOfRefusal):
        response = ActionAcceptedRefused()

        response.accepted = accepted
        response.reason_of_refusal = reasonOfRefusal

        return response

    @classmethod
    def PartToPickConstructor(self, cellId = 'defaultCellId', partId = 'defaultPartId', quantity = 1):
        partToPick = PartToPick()

        partToPick.cellId = str(cellId)
        partToPick.partId = str(partId)
        partToPick.quantity = int(quantity)

        return partToPick

    @classmethod
    def KittingOrderConstructor(self, kittingOrderId = 'defaultId', carSeqNumber = 'defaultCarSeqNumber', kitId = 'defaultKitId', partsToPick = None):
        kittingOrder = KittingOrder()

        kittingOrder.id = str(kittingOrderId)
        kittingOrder.carSeqNumber = str(carSeqNumber)
        kittingOrder.kitId = str(kitId)

        if partsToPick is None:
            part = MSGConstructor.PartToPickConstructor()
            kittingOrder.partsToPick.append(part)
        else:
            kittingOrder.partsToPick = partsToPick

        return kittingOrder

    @classmethod
    def KittingOrderListConstructor(self, kittingOrders = None):
        kittingOrderList = []

        if kittingOrders is None:
            kittingOrder = MSGConstructor.KittingOrderConstructor()
            kittingOrderList.append(kittingOrder)
        else:
            kittingOrderList = kittingOrders

        return kittingOrderList

    ## TODO: Missing Unit Test
    @classmethod
    def TaskStatusConstructor(self, missionId = 'defaultMissionId', taskId = 'defaultTaskId', statusCode = 0, statusDescription = 'defaultStatusDescription', when = None):
        taskStatus = TaskStatus()

        taskStatus.missionId = str(missionId)
        taskStatus.taskId = str(taskId)
        taskStatus.statusCode = int(statusCode)
        taskStatus.statusDescription = str(statusDescription)
        taskStatus.when = when

        return taskStatus

        # partToPick = PartToPick()
        #
        # partToPick.cellId = str(cellId)
        # partToPick.partId = str(partId)
        # partToPick.quantity = int(quantity)
        #
        # return partToPick
