#!/usr/bin/env python

import sys
import rospy

from path_assigner_class import PathAssigner


if __name__ == "__main__":

    rospy.init_node('path_assigner')

    pa = PathAssigner('abc')


    quit()
