#!/usr/bin/env python

import sys
import rospy

class ROSInterface(object):

    """docstring for ROSInterface."""

    @classmethod
    def get_ROS_param(self, paramName):
        try:
            return rospy.get_param(paramName)
        except Exception as e:
            raise KeyError('Unable to access ROS parameter server for ' + str(paramName))
