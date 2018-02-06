#!/usr/bin/env python

import sys
import rospy
import math

from task_manager_msgs.srv import *

# TODO: Move to MSGConstructor
from robis_nav_msgs.msg import ParametricPathSet
from robis_nav_msgs.msg import ParametricPath


class PathAssigner(object):

    """docstring for PathAssigner."""

    def __init__(self, getPathServiceName = None, graphEdges = None, graphVertices = None):
        self.graphEdges = graphEdges if graphEdges is not None else []
        self.graphVertices = graphVertices if graphVertices is not None else []

        if getPathServiceName is not None:
            rospy.Service(getPathServiceName, GetPathEdges, self.get_path_request_parser)
            rospy.loginfo('[PathAssigner] Ready to send Paths at ' + str(getPathServiceName))

        # TODO: Raise Exception for empty edges || or should we send an empty path?
        # TODO: Raise Exception for empty vertices || or should we send an empty path?

    ## NOTE: ROS specific
    def get_path_request_parser(self, req):
        return self.get_path_service_handler(edges = list(req.edges))

        ## DEBUG!!!!
        # return self.get_path_service_handler(edges = [6, 7])

    def get_path_service_handler(self, edges = []):
        parametricPathList = []

        rospy.loginfo('[PathAssigner] Path with ' + str(len(edges)) + ' edges requested!')

        for edge in edges:

            for graphEdge in self.graphEdges:
                if edge == graphEdge['Id']:

                    if graphEdge['VelocityForward'] > 0:
                        velocity = graphEdge['VelocityForward']
                    else:
                        velocity = - graphEdge['VelocityBackwards']

                    curveType = graphEdge['CurveType']

                    param1 = graphEdge['ParamF']
                    param2 = graphEdge['ParamB']

                    for graphVertex in self.graphVertices:

                        if graphVertex['Id'] == graphEdge['Origin_ID']:
                            try:
                                frame_id_origin = graphVertex['FrameId']
                            except:
                                frame_id_origin = ''
                            x1 = graphVertex['X']
                            y1 = graphVertex['Y']
                            theta1 = graphVertex['Theta']

                            if velocity < 0:
                                theta1 = theta1 - math.pi

                        elif graphVertex['Id'] == graphEdge['Destination_ID']:
                            try:
                                frame_id_destination = graphVertex['FrameId']
                            except:
                                frame_id_destination = ''
                            x2 = graphVertex['X']
                            y2 = graphVertex['Y']
                            theta2 = graphVertex['Theta']

                            if velocity < 0:
                                theta2 = theta2 - math.pi

                    if frame_id_origin == frame_id_destination:
                        frame_id = frame_id_origin
                    else:
                        rospy.logwarn('[PathAssigner] Origin and destination vertex have different Frame IDs')

                    aux_x1=x1+param1*math.cos(theta1)
                    aux_y1=y1+param1*math.sin(theta1)
                    aux_x2=x2-param2*math.cos(theta2)
                    aux_y2=y2-param2*math.sin(theta2)

                    Cx=3.0*(aux_x1-x1)
                    Bx=3.0*(aux_x2-aux_x1)-Cx
                    Ax=x2-x1-Cx-Bx
                    Cy=3.0*(aux_y1-y1)
                    By=3.0*(aux_y2-aux_y1)-Cy
                    Ay=y2-y1-Cy-By

                    Fx = [x1, Cx, Bx, Ax]
                    Fy = [y1, Cy, By, Ay]

                    ## TODO: To be moved to MSGConstructor
                    # Creates a robis_nav_msgs/ParametricPath Message
                    parametricPathMessage = ParametricPath()

                    parametricPathMessage.Velocity = velocity
                    parametricPathMessage.CurveType = curveType
                    parametricPathMessage.Fx = Fx
                    parametricPathMessage.Fy = Fy
                    parametricPathMessage.FrameId = frame_id

                    parametricPathList.append(parametricPathMessage)



        parametricPathSetMessage = ParametricPathSet()

        parametricPathSetMessage.PathSet = parametricPathList

        rospy.loginfo('[PathAssigner] Path with ' + str(len(parametricPathList)) + ' splines sent!')

        return parametricPathSetMessage
