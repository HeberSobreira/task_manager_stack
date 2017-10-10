#! /usr/bin/env python

import roslib
import rospy
import actionlib
import sys

from task_manager_common.msg import *
from geometry_msgs.msg import PoseStamped

####################
# GLOBAL VARIABLES #
####################

robotId = "igor"

######################
# CLASSES DEFINITION #
######################

class DriveSkillAction(object):
	# create messages that are used to publish feedback/result
	_feedback = task_manager_common.msg.DriveSkillFeedback()
	_result   = task_manager_common.msg.DriveSkillResult()

	global robotId

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, task_manager_common.msg.DriveSkillAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	def execute_cb(self, goal):

		rospy.loginfo("[FakeSkillServer] [DriveSkill] Starting fake execution")

		# helper variables
		r = rospy.Rate(10)
		percentage = 0

		while percentage != 100:

			# Check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				rospy.loginfo("[FakeSkillServer] [%s] Preempted!", self._action_name)
				self._as.set_preempted()
				break

			# Updates the feedback percentage
			self._feedback.percentage = percentage

			# Publishes the feedback
			self._as.publish_feedback(self._feedback)

			percentage = percentage + 1
			#rospy.loginfo('%s: Proccessed %s' % self._action_name, str(percentage))

			# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
			r.sleep()

		if percentage == 100:
			rospy.loginfo("[FakeSkillServer] [%s] Succeeded!", self._action_name)

			# Updates the result
			self._result.percentage = percentage

			# Publishes the result
			self._as.set_succeeded(self._result)

class LocateSkillAction(object):
	# create messages that are used to publish feedback/result
	_feedback = task_manager_common.msg.LocateSkillFeedback()
	_result   = task_manager_common.msg.LocateSkillResult()

	global robotId

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, task_manager_common.msg.LocateSkillAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	def PoseStamped_message_creation(self, frame_id, x, y, z):

		# Creates a geometry_msgs/PointStamped Message
		Pose = PoseStamped()

		Pose.header.frame_id = frame_id

		# Origin coordinates
		Pose.pose.position.x = x
		Pose.pose.position.y = y
		Pose.pose.position.z = z

		Pose.pose.orientation.x = 0
		Pose.pose.orientation.y = 0
		Pose.pose.orientation.z = 0
		Pose.pose.orientation.w = 1

		return Pose

	def execute_cb(self, goal):

		rospy.loginfo("[FakeSkillServer] [LocateSkill] Starting fake execution")

		# helper variables
		r = rospy.Rate(10)
		percentage = 0

		while percentage != 100:

			# Check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				rospy.loginfo("[FakeSkillServer] [%s] Preempted!", self._action_name)
				self._as.set_preempted()
				break

			# Updates the feedback percentage
			self._feedback.percentage = percentage

			# Publishes the feedback
			self._as.publish_feedback(self._feedback)

			percentage = percentage + 20
			#rospy.loginfo('%s: Proccessed %s' % self._action_name, str(percentage))

			# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
			r.sleep()

		if percentage == 100:

			rospy.loginfo("[FakeSkillServer] [%s] Succeeded!", self._action_name)

			# Updates the result
			self._result.percentage = percentage
			self._result.PartPosition = self.PoseStamped_message_creation("map", 0, 0, 0)

			# Publishes the result
			self._as.set_succeeded(self._result)

class PickSkillAction(object):
	# create messages that are used to publish feedback/result
	_feedback = task_manager_common.msg.PickSkillFeedback()
	_result   = task_manager_common.msg.PickSkillResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, task_manager_common.msg.PickSkillAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	def execute_cb(self, goal):

		rospy.loginfo("[FakeSkillServer] [PickSkill] Starting fake execution")

		# helper variables
		r = rospy.Rate(10)
		percentage = 0

		while percentage != 100:

			# Check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				rospy.loginfo('[FakeSkillServer] [%s] Preempted!', self._action_name)
				self._as.set_preempted()
				break

			# Updates the feedback percentage
			self._feedback.percentage = percentage

			# Publishes the feedback
			self._as.publish_feedback(self._feedback)

			percentage = percentage + 1
			#rospy.loginfo('%s: Proccessed %s' % self._action_name, str(percentage))

			# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
			r.sleep()

		if percentage == 100:

			rospy.loginfo('[FakeSkillServer] [%s] Succeeded!', self._action_name)

			# Updates the result
			self._result.percentage = percentage

			# Publishes the result
			self._as.set_succeeded(self._result)

class PlaceSkillAction(object):
	# create messages that are used to publish feedback/result
	_feedback = task_manager_common.msg.PlaceSkillFeedback()
	_result   = task_manager_common.msg.PlaceSkillResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, task_manager_common.msg.PlaceSkillAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	def execute_cb(self, goal):

		rospy.loginfo("[FakeSkillServer] [PlaceSkill] Starting fake execution")

		# helper variables
		r = rospy.Rate(10)
		percentage = 0

		while percentage != 100:

			# Check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				rospy.loginfo('[FakeSkillServer] [%s] Preempted!', self._action_name)
				self._as.set_preempted()
				break

			# Updates the feedback percentage
			self._feedback.percentage = percentage

			# Publishes the feedback
			self._as.publish_feedback(self._feedback)

			percentage = percentage + 1
			#rospy.loginfo('%s: Proccessed %s' % self._action_name, str(percentage))

			# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
			r.sleep()

		if percentage == 100:

			rospy.loginfo('[FakeSkillServer] [%s] Succeeded!', self._action_name)

			# Updates the result
			self._result.percentage = percentage

			# Publishes the result
			self._as.set_succeeded(self._result)

class WaitSkillAction(object):
	# create messages that are used to publish feedback/result
	_feedback = task_manager_common.msg.WaitSkillFeedback()
	_result   = task_manager_common.msg.WaitSkillResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, task_manager_common.msg.WaitSkillAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	def execute_cb(self, goal):
		# helper variables

		rospy.loginfo("[FakeSkillServer] [WaitSkill] Wait Time: %i" % goal.waitTime )

		r = rospy.Rate(1)

		percentage = 0
		timeLeft = goal.waitTime

		while timeLeft != 0:

			# Check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				rospy.loginfo('[FakeSkillServer] [%s] Preempted!', self._action_name)
				self._as.set_preempted()
				break

			# Updates the feedback percentage
			self._feedback.percentage = percentage

			# Publishes the feedback
			self._as.publish_feedback(self._feedback)

			timeLeft = timeLeft - 1
			#rospy.loginfo('%s: Proccessed %s' % self._action_name, str(percentage))

			rospy.loginfo("[FakeSkillServer] [WaitSkill] Time Left: %i" % timeLeft )

			# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
			r.sleep()

		rospy.loginfo('[FakeSkillServer] [%s] Succeeded!', self._action_name)

		# Updates the result
		self._result.percentage = 100

		# Publishes the result
		self._as.set_succeeded(self._result)

class DockSkillAction(object):
	# create messages that are used to publish feedback/result
	_feedback = task_manager_common.msg.DockSkillFeedback()
	_result   = task_manager_common.msg.DockSkillResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, task_manager_common.msg.DockSkillAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	def execute_cb(self, goal):

		rospy.loginfo("[FakeSkillServer] [DockSkill] Starting fake execution")

		# helper variables
		r = rospy.Rate(10)
		percentage = 0

		while percentage != 100:

			# Check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				rospy.loginfo('[FakeSkillServer] [%s] Preempted!', self._action_name)
				self._as.set_preempted()
				break

			# Updates the feedback percentage
			self._feedback.percentage = percentage

			# Publishes the feedback
			self._as.publish_feedback(self._feedback)

			percentage = percentage + 1
			#rospy.loginfo('%s: Proccessed %s' % self._action_name, str(percentage))

			# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
			r.sleep()

		if percentage == 100:

			rospy.loginfo('[FakeSkillServer] [%s] Succeeded!', self._action_name)

			# Updates the result
			self._result.percentage = percentage

			# Publishes the result
			self._as.set_succeeded(self._result)

class ActuateArmSkillAction(object):
	# create messages that are used to publish feedback/result
	_feedback = task_manager_common.msg.ActuateArmSkillFeedback()
	_result   = task_manager_common.msg.ActuateArmSkillResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, task_manager_common.msg.ActuateArmSkillAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	def execute_cb(self, goal):

		rospy.loginfo("[FakeSkillServer] [ActuateArmSkill] Starting fake execution")

		# helper variables
		r = rospy.Rate(10)
		percentage = 0

		while percentage != 100:

			# Check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				rospy.loginfo('[FakeSkillServer] [%s] Preempted!', self._action_name)
				self._as.set_preempted()
				break

			# Updates the feedback percentage
			self._feedback.percentage = percentage

			# Publishes the feedback
			self._as.publish_feedback(self._feedback)

			percentage = percentage + 1
			#rospy.loginfo('%s: Proccessed %s' % self._action_name, str(percentage))

			# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
			r.sleep()

		if percentage == 100:

			rospy.loginfo('[FakeSkillServer] [%s] Succeeded!', self._action_name)

			# Updates the result
			self._result.percentage = percentage

			# Publishes the result
			self._as.set_succeeded(self._result)

class MoveArmSkillAction(object):
	# create messages that are used to publish feedback/result
	_feedback = task_manager_common.msg.MoveArmSkillFeedback()
	_result   = task_manager_common.msg.MoveArmSkillResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, task_manager_common.msg.MoveArmSkillAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	def execute_cb(self, goal):

		rospy.loginfo("[FakeSkillServer] [MoveArmSkill] Starting fake execution")

		# helper variables
		r = rospy.Rate(10)
		percentage = 0

		while percentage != 100:

			# Check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				rospy.loginfo('[FakeSkillServer] [%s] Preempted!', self._action_name)
				self._as.set_preempted()
				break

			# Updates the feedback percentage
			self._feedback.percentage = percentage

			# Publishes the feedback
			self._as.publish_feedback(self._feedback)

			percentage = percentage + 1
			#rospy.loginfo('%s: Proccessed %s' % self._action_name, str(percentage))

			# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
			r.sleep()

		if percentage == 100:

			rospy.loginfo('[FakeSkillServer] [%s] Succeeded!', self._action_name)

			# Updates the result
			self._result.percentage = percentage

			# Publishes the result
			self._as.set_succeeded(self._result)

class GenericSkillAction(object):
	# create messages that are used to publish feedback/result
	_feedback = task_manager_common.msg.GenericSkillFeedback()
	_result   = task_manager_common.msg.GenericSkillResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, task_manager_common.msg.GenericSkillAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	def execute_cb(self, goal):

		rospy.loginfo("[FakeSkillServer] [GenericSkill] Starting fake execution")

		# helper variables
		r = rospy.Rate(100)
		percentage = 0

		while percentage != 100:

			# Check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				rospy.loginfo('[FakeSkillServer] [%s] Preempted!', self._action_name)
				self._as.set_preempted()
				break

			# Updates the feedback percentage
			self._feedback.percentage = percentage

			# Publishes the feedback
			self._as.publish_feedback(self._feedback)

			percentage = percentage + 1
			#rospy.loginfo('%s: Proccessed %s' % self._action_name, str(percentage))

			# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
			r.sleep()

		if percentage == 100:

			rospy.loginfo('[FakeSkillServer] [%s] Succeeded!', self._action_name)

			# Updates the result
			self._result.percentage = percentage

			# Publishes the result
			self._as.set_succeeded(self._result)


########
# MAIN #
########

def skill_action_server():

	global robotId;

	if len(sys.argv) == 1:
		rospy.init_node('skill')
		rospy.loginfo("Starting Fake skill Action Server")

	if len(sys.argv) > 1:
		rospy.init_node(str(sys.argv[1]))

		try:
			robotId = str(sys.argv[2])
		except Exception, e:
			# robotId = "Igor"
			pass

		if str(sys.argv[1]) == "driveSkill":
			rospy.loginfo("[%s] [%s] Starting Fake Action Server!", str(sys.argv[1]), str(robotId))
			DriveSkillAction(rospy.get_name())
		elif str(sys.argv[1]) == "locateSkill":
			rospy.loginfo("[%s] [%s] Starting Fake Action Server!", str(sys.argv[1]), str(robotId))
			LocateSkillAction(rospy.get_name())
		elif str(sys.argv[1]) == "pickSkill":
			rospy.loginfo("[%s] [%s] Starting Fake Action Server!", str(sys.argv[1]), str(robotId))
			PickSkillAction(rospy.get_name())
		elif str(sys.argv[1]) == "placeSkill":
			rospy.loginfo("[%s] [%s] Starting Fake Action Server!", str(sys.argv[1]), str(robotId))
			PlaceSkillAction(rospy.get_name())
		elif str(sys.argv[1]) == "waitSkill":
			rospy.loginfo("[%s] [%s] Starting Fake Action Server!", str(sys.argv[1]), str(robotId))
			WaitSkillAction(rospy.get_name())
		elif str(sys.argv[1]) == "dockSkill":
			rospy.loginfo("[%s] [%s] Starting Fake Action Server!", str(sys.argv[1]), str(robotId))
			DockSkillAction(rospy.get_name())
		elif str(sys.argv[1]) == "actuateArmSkill":
			rospy.loginfo("[%s] [%s] Starting Fake Action Server!", str(sys.argv[1]), str(robotId))
			ActuateArmSkillAction(rospy.get_name())
		elif str(sys.argv[1]) == "moveArmSkill":
			rospy.loginfo("[%s] [%s] Starting Fake Action Server!", str(sys.argv[1]), str(robotId))
			MoveArmSkillAction(rospy.get_name())
		elif str(sys.argv[1]) == "genericSkill":
			rospy.loginfo("[%s] [%s] Starting Fake Action Server!", str(sys.argv[1]), str(robotId))
			GenericSkillAction(rospy.get_name())
		else:
			rospy.loginfo("[%s] [%s] Fake Skill Not Available!", str(sys.argv[1]), str(robotId))

	rospy.spin()


if __name__ == '__main__':
	skill_action_server();
