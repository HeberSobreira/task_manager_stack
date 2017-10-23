#!/usr/bin/env python

import sys
import rospy

from ros_interface import ROSInterface
from path_assigner_class import PathAssigner

if __name__ == "__main__":

    rospy.init_node('path_assigner')

    try:
        ## Parameters
        graphEdges = ROSInterface.get_ROS_param('~Edges')
        graphVertices = ROSInterface.get_ROS_param('~Vertices')

        ## Services
        getPathServiceName = ROSInterface.get_ROS_param('~get_path_service_name')


    except KeyError as e:
        rospy.logerr('[PathAssigner] Error: %s', str(e))
        rospy.logdebug(traceback.format_exc())

        quit()

    try:
        pa = PathAssigner(getPathServiceName = getPathServiceName, graphEdges = graphEdges, graphVertices = graphVertices)

        rospy.spin()

    except Exception as e:
        rospy.logerr('[PathAssigner] Error: %s', str(e))
        rospy.logdebug(traceback.format_exc())
