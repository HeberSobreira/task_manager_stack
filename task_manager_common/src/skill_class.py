#!/usr/bin/env python

import sys
import rospy
import actionlib

from task_manager_msgs.msg import *
from task_manager_msgs.srv import GetPath
from msg_constructor import *

class Skill(object):

    """docstring for Skill."""

    def __init__(self, skillName = None, skillType = None, skillClass = None, allowedSkillPropertiesKeys = None, skillProperties = None):
        self.skillName = skillName
        self.skillType = skillType
        self.skillClass = skillClass
        self.allowedSkillPropertiesKeys = allowedSkillPropertiesKeys
        self.skillProperties = skillProperties

    def skillPropertiesConstructor(self, task):
        self.skillProperties = {}

        for key in task:
            if key == 'skillName':
                pass
            elif key in self.allowedSkillPropertiesKeys:
                self.skillProperties.update({key: task[key]})
            else:
                raise KeyError(str(key) + ' skill property not found in ' + str(self.allowedSkillPropertiesKeys))

    def goal_encoder(self):
        goal = str(self.skillName)

        for key in self.skillProperties:
            goal += ';' + str(key) + '=' + str(self.skillProperties[key])

        return goal

    ## TODO: Missing Unit Test
    def actionNameConstructor(self):
        return self.skillType

    def actionTypeConstructor(self):
        try:
            return eval('task_manager_msgs.msg.' + str(self.skillType) + 'Action')
        except AttributeError as e:
            raise AttributeError('Unable to find Skill msg task_manager_msgs.msg.' + str(self.skillType))

    def actionGoalConstructor(self):
        raise NotImplementedError('Subclass ' + str(self.skillType) + ' must implement abstract method!')

    # ROS specific
    def action_client(self, waitForServerTimeOut = 10, waitForActionClientTimeOut = 300):
        actionName = self.actionNameConstructor() ## TODO: Missing Unit Test
        actionType = self.actionTypeConstructor()
        actionGoal = self.actionGoalConstructor()

        # Creates a ROS Action Client, passing the type of the action to the constructor.
        client = actionlib.SimpleActionClient(actionName, actionType)

        ## IDEA: Maybe execute here some PRE-CONDITIONS defined within the Skill Subclass (Have a method that MUST/COULD be defined)
        ## BETTER IDEA: Encapsulate the 'action_client' method inside a 'skill_client()' method that performs such check

        rospy.logdebug('[' + str(self.skillClass) + '] Waiting for ' + str(actionName) + ' Server...')

        # TODO: Try to find a better Exception Type to handle this
        if not client.wait_for_server(rospy.Duration(waitForServerTimeOut)):
            raise Exception('Skill Action Client Error: Timed out (' + str(waitForServerTimeOut) + 's) while trying to find Action Server ' + str(actionName))

        # Sends the goal to the action server.
        rospy.logdebug('[' + str(self.skillClass) + '] Sending goal to ' + str(actionName) + ' Server...')
        client.send_goal(actionGoal)

        rospy.logdebug('[' + str(self.skillClass) + '] Waiting response from ' + str(actionName) + ' Server...')

        if not client.wait_for_result(rospy.Duration(waitForActionClientTimeOut)):
            raise Exception('Skill Action Client Error: Timed out (' + str(waitForActionClientTimeOut) + 's) while waiting for ' + str(actionName) + ' Action Server result')

        ## IDEA: Maybe execute here some POST-CONDITIONS defined within the Skill Subclass (Have a method that MUST/COULD be defined)
        ## BETTER IDEA: Encapsulate the 'action_client' method inside a 'skill_client()' method that performs such check

        # Return the result of executing the action
        rospy.logdebug('[' + str(self.skillClass) + '] Received result from ' + str(actionName) + ' Server!')
        return client.get_result()

class GenericSkill(Skill):

    """docstring for GenericSkill."""

    def actionGoalConstructor(self):
        arguments = None

        for key, value in self.skillProperties.iteritems():
            if arguments is None:
                if type(value) is not str:
                    arguments = str(key) + '=' + str(value)
                else:
                    arguments = str(key) + "='" + value + "'"
            else:
                if type(value) is not str:
                    arguments += ', ' + str(key) + '=' + str(value)
                else:
                    arguments += ', ' + str(key) + "='" + value + "'"

        return eval('task_manager_msgs.msg.' + str(self.skillType) + 'Goal(' + arguments + ')')

class DriveSkill(Skill):

    """docstring for DriveSkill."""

    pass

class DriveEdgesSkill(Skill):

    """docstring for DriveEdgesSkill."""

    ## TODO: Missing Unit Test
    def actionNameConstructor(self):

        # Should be Object / Class variable
        supportedTractionModes = ['DIFFERENTIAL', 'TRICYCLE', 'OMNI']

        try:
            mode = self.skillProperties['mode']
        except KeyError as e:
            raise KeyError('DriveEdgesSkill missing property: ' + str(e))

        if mode in supportedTractionModes:
            if mode == 'DIFFERENTIAL':
                return 'DriveEdgesSkillDifferential'

            elif mode == 'TRICYCLE':
                return 'DriveEdgesSkillTricycle'

            elif mode == 'OMNI':
                return 'DriveEdgesSkillOmni'

        else:
            raise AttributeError('Unsupported traction mode :' + str(mode) + '. Supported traction modes are: ' + str(supportedTractionModes))

    def actionGoalConstructor(self):

        try:
            mode = self.skillProperties['mode']
            edges = self.skillProperties['edges']
        except KeyError as e:
            raise KeyError('DriveEdgesSkill missing property: ' + str(e))

        rospy.wait_for_service('/task_manager/GetPath', timeout=3) #TODO: Shouldnt be hard coded
        getPath = rospy.ServiceProxy('/task_manager/GetPath', GetPath)
        response = getPath(edges)
        pathSet = response.PathSet

        arguments = 'ControllerMode=mode, PathSet = pathSet'
        return eval('task_manager_msgs.msg.' + str(self.skillType) + 'Goal(' + arguments + ')')

class PoseInteractionSkill(Skill):

    """docstring for PoseInteractionSkill."""

    def actionGoalConstructor(self):


        #TODO:check if the skillProperties match with the allowedSkillPropertiesKeys
        #Does it make sense?
        for key in self.skillProperties:
            if key not in self.allowedSkillPropertiesKeys:
                raise AttributeError('%s is not an allowed property key' %key)

        try:
            frameId = self.skillProperties['frameId']
            px = self.skillProperties['px']
            py = self.skillProperties['py']
            pz = self.skillProperties['pz']
            qx = self.skillProperties['qx']
            qy = self.skillProperties['qy']
            qz = self.skillProperties['qz']
            qw = self.skillProperties['qw']
        except KeyError as e:
            raise KeyError('PoseInteractionSkill missing property: ' + str(e))

        poseStamped = MSGConstructor.PoseStampedConstructor(frameId, px, py, pz, qx, qy, qz, qw)

        arguments = 'ObjectId = frameId, Pose = poseStamped'
        # task_manager_msgs.msg.skillTypeGoal(ObjectId=frameId, Pose = poseStamped)
        return eval('task_manager_msgs.msg.' + str(self.skillType) + 'Goal(' + arguments + ')')


class ObjectInteractionSkill(Skill):

    """docstring for ObjectInteractionSkill."""

    pass
