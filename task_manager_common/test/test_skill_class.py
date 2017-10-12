#!/usr/bin/env python

import sys
import unittest
import rostest

from task_manager_msgs.msg import *

from skill_class import *
# from skill_factory_class import SkillFactory


PKG = 'task_manager_common'
NAME = 'test_skill_class'

# Base Class for testing Skill Class
class TestSkillClassBase(unittest.TestCase):

    @classmethod
    def skill_generator(self, skillName = 'example-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1']):
        return {'skillName': skillName, 'skillType': skillType, 'skillClass': skillClass, 'skillProperties': skillProperties}

    @classmethod
    def task_generator(self, skillName = 'example-skill', nSkillProperties = 2):
        task = {'skillName': skillName}

        for i in range(nSkillProperties):
            skillPropertyKey = 'exampleSkillProperty' + str(i)
            skillPropertyValue = 'exampleSkillValue' + str(i)
            task.update({skillPropertyKey: skillPropertyValue})

        return task

    @classmethod
    def skill_object_constructor_from_dictionary(self, skill):
        return Skill(skillName = skill['skillName'], skillType = skill['skillType'], skillClass = skill['skillClass'], allowedSkillPropertiesKeys = skill['skillProperties'])

# Unit Tests for Skill Class
class TestSkillClass(TestSkillClassBase):

    def test_create_object_with_skillName(self):
        s = Skill(skillName = 'example-skill')
        self.assertEquals(s.skillName, 'example-skill')

    def test_create_object_with_skillType(self):
        s = Skill(skillType = 'GenericSkill')
        self.assertEquals(s.skillType, 'GenericSkill')

    def test_create_object_with_skillClass(self):
        s = Skill(skillClass = 'GenericSkill')
        self.assertEquals(s.skillClass, 'GenericSkill')

    def test_create_object_with_allowedSkillProperties(self):
        s = Skill(allowedSkillPropertiesKeys = ['skillPropertyKey1', 'skillPropertyKey2', 'skillPropertyKey3'])
        self.assertEquals(s.allowedSkillPropertiesKeys, ['skillPropertyKey1', 'skillPropertyKey2', 'skillPropertyKey3'])

    def test_create_object_with_skillProperties(self):
        s = Skill(skillProperties = {'skillPropertyKey1': 'skillPropertyValue1', 'skillPropertyKey2': 'skillPropertyValue2', 'skillPropertyKey3': 'skillPropertyValue3'})
        self.assertEquals(s.skillProperties, {'skillPropertyKey1': 'skillPropertyValue1', 'skillPropertyKey2': 'skillPropertyValue2', 'skillPropertyKey3': 'skillPropertyValue3'})

# Unit Tests for Skill Properties Constructor
class TestSkillPropertiesConstructor(TestSkillClassBase):

    def test_task_contains_correct_skill_properties_keys(self):
        task = self.task_generator()
        skill = self.skill_generator()

        s = self.skill_object_constructor_from_dictionary(skill)
        s.skillPropertiesConstructor(task)

        self.assertEquals(s.skillProperties, {'exampleSkillProperty0': 'exampleSkillValue0', 'exampleSkillProperty1': 'exampleSkillValue1'})

    def test_task_contains_incorrect_skill_properties_keys(self):
        task = {'skillName': 'test-skill', 'correctSkillProperty': 'abc', 'incorrectSkillProperty': 'def'}
        skill = self.skill_generator(skillName = 'test-skill', skillProperties = ['correctSkillProperty'])

        s = self.skill_object_constructor_from_dictionary(skill)

        with self.assertRaises(KeyError):
            s.skillPropertiesConstructor(task)

    def test_skill_properties_empty(self):
        task = self.task_generator()
        skill = self.skill_generator(skillProperties = [])

        s = self.skill_object_constructor_from_dictionary(skill)

        with self.assertRaises(KeyError):
            s.skillPropertiesConstructor(task)

    def test_task_without_properties(self):
        task = self.task_generator(nSkillProperties = 0)
        skill = self.skill_generator()

        s = self.skill_object_constructor_from_dictionary(skill)

        s.skillPropertiesConstructor(task)

        self.assertEquals(s.skillProperties, {})

# Unit Test for Goal Encoder
class TestGoalEncoder(TestSkillClassBase):

    def test_encode_goal_with_one_skill_property(self):
        skill = self.skill_generator(skillProperties = ['exampleSkillProperty0'])
        task = self.task_generator(nSkillProperties = 1)

        s = self.skill_object_constructor_from_dictionary(skill)
        s.skillPropertiesConstructor(task)

        goal = s.goal_encoder()

        self.assertEquals(goal, 'example-skill;exampleSkillProperty0=exampleSkillValue0')

    def test_encode_goal_with_multiple_skill_properties(self):
        skill = self.skill_generator(skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1', 'exampleSkillProperty2'])
        task = self.task_generator(nSkillProperties = 3)

        s = self.skill_object_constructor_from_dictionary(skill)
        s.skillPropertiesConstructor(task)

        goal = s.goal_encoder()

        self.assertTrue('example-skill;' in goal)
        self.assertTrue('exampleSkillProperty0=exampleSkillValue0' in goal)
        self.assertTrue('exampleSkillProperty1=exampleSkillValue1' in goal)
        self.assertTrue('exampleSkillProperty2=exampleSkillValue2' in goal)

    def test_encode_goal_without_skill_properties(self):
        skill = self.skill_generator(skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1', 'exampleSkillProperty2'])
        task = self.task_generator(nSkillProperties = 0)

        s = self.skill_object_constructor_from_dictionary(skill)
        s.skillPropertiesConstructor(task)

        goal = s.goal_encoder()

        self.assertEquals(goal, 'example-skill')

# Unit Test for Action Type Constructor
class TestActionTypeConstructor(TestSkillClassBase):

    def test_create_action_type(self):
        s = Skill(skillName = 'drive-skill', skillType = 'DriveSkill', allowedSkillPropertiesKeys = ['toObjectId', 'toObjectType'], skillProperties = {'toObjectId': 'exampleObjectId', 'toObjectType': 'exampleToObjectId'})

        actionType = s.actionTypeConstructor()

        self.assertEquals(task_manager_msgs.msg.DriveSkillAction, actionType)

    def test_return_error(self):
        s = Skill(skillName = 'drive-skill', skillType = 'DriveSkills', allowedSkillPropertiesKeys = ['toObjectId', 'toObjectType'], skillProperties = {'toObjectId': 'exampleObjectId', 'toObjectType': 'exampleToObjectId'})

        with self.assertRaises(AttributeError):
            actionType = s.actionTypeConstructor()

# Unit Test for Action Goal Constructor
class TestActionGoalConstructor(TestSkillClassBase):

    def test_raise_not_implemented_error(self):
        s = Skill()

        with self.assertRaises(NotImplementedError):
            s.actionGoalConstructor()

# Unit Test for Action Client
class TestActionClient(TestSkillClassBase):

    ## TODO! (If possible... Maybe it can only be done w/ Functional Testing...)
    pass




# Unit Test for Generic Skill
class TestGenericSkill(TestSkillClassBase):

    def test_generic_skill_successful_creation(self):
        gs = GenericSkill(skillName = 'generic-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['exampleSkillProperty0', 'exampleSkillProperty1'], skillProperties = {'exampleSkillProperty0': 'exampleSkillValue0', 'exampleSkillProperty1': 'exampleSkillValue1'})

        actionGoal = gs.actionGoalConstructor()

        self.assertEquals(type(actionGoal), GenericSkillGoal)
        self.assertEquals(actionGoal.exampleSkillProperty0, 'exampleSkillValue0')
        self.assertEquals(actionGoal.exampleSkillProperty1, 'exampleSkillValue1')


    def test_generic_skill_missing_property(self):
        gs = GenericSkill(skillName = 'generic-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['exampleSkillProperty0', 'exampleSkillProperty1'], skillProperties = {'exampleSkillProperty0': 'exampleSkillValue0'})

        actionGoal = gs.actionGoalConstructor()

        self.assertEquals(type(actionGoal), GenericSkillGoal)
        self.assertEquals(actionGoal.exampleSkillProperty0, 'exampleSkillValue0')
        self.assertEquals(actionGoal.exampleSkillProperty1, '')

    def test_generic_skill_extra_property(self):
        gs = GenericSkill(skillName = 'generic-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['exampleSkillProperty0', 'exampleSkillProperty1'], skillProperties = {'exampleSkillProperty0': 'exampleSkillValue0', 'exampleSkillProperty1': 'exampleSkillValue1', 'exampleSkillProperty2': 'exampleSkillValue2'})

        with self.assertRaises(AttributeError):
            actionGoal = gs.actionGoalConstructor()

    def test_generic_skill_wrong_properties(self):
        gs = GenericSkill(skillName = 'generic-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['exampleSkillProperty0', 'exampleSkillProperty1'], skillProperties = {'abc': 'def'})

        with self.assertRaises(AttributeError):
            actionGoal = gs.actionGoalConstructor()

    # TODO: Fix this!
    # def test_generic_skill_wrong_property_type(self):
    #     gs = GenericSkill(skillName = 'generic-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', allowedSkillPropertiesKeys = ['exampleSkillProperty0', 'exampleSkillProperty1'], skillProperties = {'exampleSkillProperty0': 'exampleSkillValue0', 'exampleSkillProperty1': [10, 11, 'abc']})
    #
    #     actionGoal = gs.actionGoalConstructor()
    #
    #     print actionGoal
    #     print type(actionGoal.exampleSkillProperty1)
    #
    #     self.assertEquals(type(actionGoal), GenericSkillGoal)
    #     self.assertEquals(actionGoal.exampleSkillProperty0, 'exampleSkillValue0')
        # self.assertEquals(actionGoal.exampleSkillProperty1, 10)



# Test Suite for Skill Class
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        testSkillClass = loader.loadTestsFromTestCase(TestSkillClass)
        testSkillPropertiesConstructor = loader.loadTestsFromTestCase(TestSkillPropertiesConstructor)
        testGoalEncoder = loader.loadTestsFromTestCase(TestGoalEncoder)
        testActionTypeConstructor = loader.loadTestsFromTestCase(TestActionTypeConstructor)
        testActionGoalConstructor = loader.loadTestsFromTestCase(TestActionGoalConstructor)
        testActionClient = loader.loadTestsFromTestCase(TestActionClient)
        testGenericSkill = loader.loadTestsFromTestCase(TestGenericSkill)

        self.addTests(testSkillClass)
        self.addTests(testSkillPropertiesConstructor)
        self.addTests(testGoalEncoder)
        self.addTests(testActionTypeConstructor)
        self.addTests(testActionGoalConstructor)
        self.addTests(testActionClient)
        self.addTests(testGenericSkill)

if __name__ == '__main__':
      rostest.rosrun(PKG, NAME, 'test_skill_class.SuiteTest', sys.argv)
