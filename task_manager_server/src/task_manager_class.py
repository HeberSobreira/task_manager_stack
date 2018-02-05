#!/usr/bin/env python

import sys
import rospy
import threading
import copy

from task_manager_msgs.srv import *
from task_manager_msgs.msg import ActionAcceptedRefused
from task_manager_msgs.msg import TaskStatus

from skill_class import Skill
from skill_factory_class import SkillFactory
from msg_constructor import MSGConstructor

class TaskManager(object):

    """docstring for TaskManager."""

    def __init__(self, robotId = None, skills = None, assignMissionServiceName = None, provideTaskStatusServiceName = None, taskStatusTopic = None, waitForServerTimeOut = None, waitForActionClientTimeOut = None, missionQueueSize = 100): #TODO: what's the best value for the default missionQueueSize?
        self.robotId = robotId if robotId is not None else 'defaultRobotId'
        self.skills = skills if skills is not None else []
        self.ongoingTasks = []
        self.missions = []
        self.taskQueue = []
        self.missionQueueSize = missionQueueSize

        self.waitForServerTimeOut = waitForServerTimeOut if waitForServerTimeOut is not None else 10
        self.waitForActionClientTimeOut = waitForActionClientTimeOut if waitForActionClientTimeOut is not None else 300

        if assignMissionServiceName is not None:
            rospy.Service(assignMissionServiceName, AssignMission, self.assign_mission_request_parser)
            rospy.loginfo('[TaskManager] [' + str(self.robotId) + '] Ready to Assign Missions at ' + str(assignMissionServiceName))

        if provideTaskStatusServiceName is not None:
            rospy.Service(provideTaskStatusServiceName, ProvideTaskStatus, self.provide_task_status_request_parser)
            rospy.loginfo('[TaskManager] [' + str(self.robotId) + '] Ready to Provide Task Status at ' + str(provideTaskStatusServiceName))

        if taskStatusTopic is not None:
            self.taskStatusPub = rospy.Publisher(taskStatusTopic, TaskStatus, queue_size=10)
            rospy.loginfo('[TaskManager] [' + str(self.robotId) + '] Ready to Publish Task Status Messages at ' + str(taskStatusTopic))
        else:
            self.taskStatusPub = None


    ## NOTE: ROS specific
    def assign_mission_request_parser(self, req):
        return self.assign_mission_service_handler(missionId = req.missionId, robotId = req.robotId, goals = req.goals, executeMission = True)

    ## NOTE: ROS specific
    def provide_task_status_request_parser(self, req):
        return self.provide_task_status_handler(missionId = req.missionId)

    ## NOTE: ROS specific
    # TODO: Unit Testing
    def provide_task_status_handler(self, missionId):
        # If mission found...
        for mission in self.missions:
            if mission['missionId'] == missionId:
                taskStatus = MSGConstructor.TaskStatusConstructor(missionId = mission['missionId'], taskId = mission['taskId'], statusCode = mission['statusCode'], statusDescription = mission['statusDescription'], when = mission['when'])
                return taskStatus

        # If mission not found
        taskStatus = MSGConstructor.TaskStatusConstructor(missionId = 'null', taskId = 'null', statusCode = 12, statusDescription = 'null', when = rospy.get_rostime())
        return taskStatus

    ## NOTE: ROS specific
    def assign_mission_service_handler(self, missionId = 'defaultMissionId', robotId = 'defaultRobotId', goals = [], executeMission = False):

        if robotId != self.robotId:
            rospy.logwarn('[TaskManager] [' + str(self.robotId) + '] Mission ' + str(missionId) + ' refused: Different robotId!')
            return MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'False', reasonOfRefusal = 'Different robotId!')

        if self.missionQueueSize > 0:
            if len(self.taskQueue) >= self.missionQueueSize:
                rospy.logwarn('[TaskManager] [' + str(self.robotId) + '] Mission ' + str(missionId) + ' refused: Mission queue is already fulfilled!')
                return MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'False', reasonOfRefusal = 'Mission queue is already fulfilled!')

        for mission in self.missions:
            if missionId == mission['missionId']:
                rospy.logwarn('[TaskManager] [' + str(self.robotId) + '] Mission ' + str(missionId) + ' refused: Mission already stored!')
                return MSGConstructor.ActionAcceptedRefusedConstructor(accepted='False', reasonOfRefusal='Mission already stored!')

        if len(goals) == 0:
            rospy.logwarn('[TaskManager] [' + str(self.robotId) + '] Mission ' + str(missionId) + ' refused: Empty goals!')
            return MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'False', reasonOfRefusal = 'Empty goals!')

        missionSkills = []

        for goal in goals:
            try:
                sf = SkillFactory(goal, self.skills)
                missionSkills += [sf.skill]
            except Exception as e:
                rospy.logwarn('[TaskManager] [' + str(self.robotId) + '] Mission ' + str(missionId) + ' refused: Badly formatted goal (' + str(goal) + '): ' + str(e))
                return MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'False', reasonOfRefusal = 'Badly formatted goal (' + str(goal) + '): ' + str(e))


        self.missions.append({'missionId': missionId, 'taskId': '', 'statusCode':'', 'statusDescription': 'Mission Created'})

        mission = {'missionId': missionId, 'missionSkills': missionSkills, 'executeMission': executeMission}
        self.taskQueue.append(mission)

        if self.ongoingTasks:
            rospy.loginfo('[TaskManager] [' + str(self.robotId) + '] Mission ' + str(missionId) + ' Accepted! Waiting for execution.')
            return MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'True', reasonOfRefusal = 'None')
        else:
            rospy.loginfo('[TaskManager] [' + str(self.robotId) + '] Mission ' + str(missionId) + ' Accepted! Starting Execution.')
            self.queue_execution_handler()
            return MSGConstructor.ActionAcceptedRefusedConstructor(accepted = 'True', reasonOfRefusal = 'None')


    def queue_execution_handler(self):

        if not self.taskQueue:
            rospy.loginfo('[TaskManager] [' + str(self.robotId) + '] There are no more missions to execute.')
            return

        self.ongoingTasks = self.taskQueue[0]['missionSkills']

        if self.taskQueue[0]['executeMission']:
            # TODO: Make sure that this is the appropriate way of executing a thread in Python
            # IDEA: Maybe the Thread object attribute 'Name' can be used to pause / stop an ongoing thread execution
            # TODO: IMPORTANT! CRITICAL! Do not forget to handle mutex / semaphore for accessing self.ongoingTasks!!!
            threading.Thread(target=self.execute_mission, args=(self.taskQueue[0]['missionId'],)).start()
        return

    # TODO: Unit Test
    def update_mission_status(self, missionId, taskId, statusCode, statusDescription):
        if statusCode in [1, 3, 11]:
            rospy.loginfo('[TaskManager] [' + str(self.robotId) + '] ' + str(statusDescription))
        elif statusCode in [4, 12]:
            rospy.logerr('[TaskManager] [' + str(self.robotId) + '] ' + str(statusDescription))

        if self.taskStatusPub is not None:
            taskStatus = MSGConstructor.TaskStatusConstructor(missionId = missionId, taskId = taskId, statusCode = statusCode, statusDescription = statusDescription, when = rospy.get_rostime())
            self.taskStatusPub.publish(taskStatus)

        for mission in self.missions:
            if mission['missionId'] == missionId:
                mission['taskId'] = taskId
                mission['statusCode'] = statusCode
                mission['statusDescription'] = statusDescription
                mission['when'] = rospy.get_rostime()

    ## NOTE: ROS specific
    def execute_mission(self, missionId):
        #self.missions.append({'missionId': missionId, 'taskId': '', 'statusCode':'', 'statusDescription': 'Mission Created'})

        rospy.loginfo('[TaskManager] [' + str(self.robotId) + '] Starting Mission ' + str(missionId) + ' with ' + str(len(self.ongoingTasks)) + ' Tasks... ')

        while self.ongoingTasks:
            task = self.ongoingTasks.pop(0) # TODO: Write a Unit Test for this!

            self.update_mission_status(missionId = missionId, taskId = task.skillName, statusCode = 1, statusDescription = 'Starting Task ' + str(task.skillName))

            try:
                self.update_mission_status(missionId = missionId, taskId = task.skillName, statusCode = 2, statusDescription = 'Executing Task ' + str(task.skillName))

                actionClientResponse = task.action_client(waitForServerTimeOut = self.waitForServerTimeOut, waitForActionClientTimeOut = self.waitForActionClientTimeOut)

                taskCompletion = actionClientResponse.percentage
                taskStatus = actionClientResponse.skillStatus
            except Exception as e:
                taskCompletion = 0
                taskStatus = str(e)

            if int(taskCompletion) == 100:
                self.update_mission_status(missionId = missionId, taskId = task.skillName, statusCode = 3, statusDescription = 'Task ' + str(task.skillName) + ' Succeeded!')

            else:
                # In case of a Mission Failure, the robot should be free to take on new tasks
                self.ongoingTasks = []

                self.update_mission_status(missionId = missionId, taskId = task.skillName, statusCode = 4, statusDescription = 'Task ' + str(task.skillName) + ' Failed: ' + str(taskStatus))
                self.update_mission_status(missionId = missionId, taskId = task.skillName, statusCode = 12, statusDescription = 'Mission Failed! Task ' + str(task.skillName) + ' Failed: ' + str(taskStatus))
                del self.taskQueue[0]
                self.queue_execution_handler()
                return False

        self.update_mission_status(missionId = missionId, taskId = task.skillName, statusCode = 11, statusDescription = 'Mission Success!')
        del self.taskQueue[0]
        return self.queue_execution_handler()
