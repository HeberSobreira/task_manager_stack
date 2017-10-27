#!/usr/bin/env python

import sys
import unittest
import rostest

from task_manager_msgs.msg import *

from skill_class import *
from skill_factory_class import SkillFactory


PKG = 'task_manager_common'
NAME = 'test_skill_factory_class'

# Base Class for testing Skill Factory Class
class TestSkillFactoryClassBase(unittest.TestCase):

    @classmethod
    def skill_generator(self, skillName = 'example-skill', skillType = 'GenericSkill', skillClass = 'GenericSkill', skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1']):
        return {'skillName': skillName, 'skillType': skillType, 'skillClass': skillClass, 'skillProperties': skillProperties}

# Unit Test for Skill Factory Goal Decoder
class TestSkillFactoryGoalDecoder(TestSkillFactoryClassBase):

    def test_empty_goal_string(self):
        goal = ''
        skills = [self.skill_generator()]

        with self.assertRaises(ValueError):
            sf = SkillFactory(goal, skills)
            s = sf.goal_decoder(goal, skills)

    def test_empty_goal_object(self):
        goal = None
        skills = [self.skill_generator()]

        with self.assertRaises(AttributeError):
            sf = SkillFactory(goal, skills)
            s = sf.goal_decoder(goal, skills)

    def test_wrong_formatted_goal(self):
        goal = 'example-skill;wrongFormatted'
        skills = [self.skill_generator()]

        with self.assertRaises(ValueError):
            sf = SkillFactory(goal, skills)
            s = sf.goal_decoder(goal, skills)

    def test_unknown_skill_name(self):
        goal = 'unknown-skill'
        skills = [self.skill_generator()]

        with self.assertRaises(ValueError):
            sf = SkillFactory(goal, skills)
            s = sf.goal_decoder(goal, skills)

    def test_no_skill_properties(self):
        goal = 'example-skill'
        skills = [self.skill_generator()]

        with self.assertRaises(KeyError):
            sf = SkillFactory(goal, skills)
            s = sf.goal_decoder(goal, skills)

    def test_unknown_skill_properties(self):
        goal = 'example-skill;exampleSkillProperty0=exampleSkillValue0;exampleSkillProperty1=exampleSkillValue1;unknownSkillProperty=unknownSkillPropertyValue'
        skills = [self.skill_generator()]

        with self.assertRaises(KeyError):
            sf = SkillFactory(goal, skills)
            s = sf.goal_decoder(goal, skills)

    def test_decode_simple_goal(self):
        goal = 'example-skill;exampleSkillProperty0=exampleSkillValue0'
        skills = [self.skill_generator(skillProperties = ['exampleSkillProperty0'])]

        sf = SkillFactory(goal, skills)
        decodedGoal = sf.goal_decoder(goal, skills)

        self.assertEquals(decodedGoal['skillName'], 'example-skill')
        self.assertEquals(decodedGoal['skillType'], 'GenericSkill')
        self.assertEquals(decodedGoal['skillClass'], 'GenericSkill')
        self.assertEquals(decodedGoal['allowedSkillPropertiesKeys'], ['exampleSkillProperty0'])
        self.assertEquals(decodedGoal['skillProperties'], {'exampleSkillProperty0': 'exampleSkillValue0'})

    def test_decode_goal_with_multiple_skill_properties(self):
        goal = 'example-skill;exampleSkillProperty0=exampleSkillValue0;exampleSkillProperty1=exampleSkillValue1;exampleSkillProperty2=exampleSkillValue2'
        skills = [self.skill_generator(skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1', 'exampleSkillProperty2'])]

        sf = SkillFactory(goal, skills)
        decodedGoal = sf.goal_decoder(goal, skills)

        self.assertEquals(decodedGoal['skillName'], 'example-skill')
        self.assertEquals(decodedGoal['skillType'], 'GenericSkill')
        self.assertEquals(decodedGoal['skillClass'], 'GenericSkill')
        self.assertEquals(decodedGoal['allowedSkillPropertiesKeys'], ['exampleSkillProperty0', 'exampleSkillProperty1', 'exampleSkillProperty2'])
        self.assertEquals(decodedGoal['skillProperties'], {'exampleSkillProperty0': 'exampleSkillValue0', 'exampleSkillProperty1': 'exampleSkillValue1', 'exampleSkillProperty2': 'exampleSkillValue2'})

    def test_decode_simple_goal_with_list_of_skills(self):
        goal = 'example-skill;exampleSkillProperty0=exampleSkillValue0'
        skills = [self.skill_generator(skillProperties = ['exampleSkillProperty0']), self.skill_generator(skillName = 'example-skill-2')]

        sf = SkillFactory(goal, skills)
        decodedGoal = sf.goal_decoder(goal, skills)

        self.assertEquals(decodedGoal['skillName'], 'example-skill')
        self.assertEquals(decodedGoal['skillType'], 'GenericSkill')
        self.assertEquals(decodedGoal['skillClass'], 'GenericSkill')
        self.assertEquals(decodedGoal['allowedSkillPropertiesKeys'], ['exampleSkillProperty0'])
        self.assertEquals(decodedGoal['skillProperties'], {'exampleSkillProperty0': 'exampleSkillValue0'})

    def test_decode_goal_with_int_property_values(self):
        goal = 'example-skill;exampleSkillProperty0=1;exampleSkillProperty1=10'
        skills = [self.skill_generator(skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1'])]

        sf = SkillFactory(goal, skills)
        decodedGoal = sf.goal_decoder(goal, skills)

        self.assertEquals(decodedGoal['skillProperties'], {'exampleSkillProperty0': 1, 'exampleSkillProperty1': 10})
        self.assertTrue(isinstance(decodedGoal['skillProperties']['exampleSkillProperty0'], int))
        self.assertTrue(isinstance(decodedGoal['skillProperties']['exampleSkillProperty1'], int))

    def test_decode_goal_with_float_property_values(self):
        goal = 'example-skill;exampleSkillProperty0=1.1;exampleSkillProperty1=10.1'
        skills = [self.skill_generator(skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1'])]

        sf = SkillFactory(goal, skills)
        decodedGoal = sf.goal_decoder(goal, skills)

        self.assertEquals(decodedGoal['skillProperties'], {'exampleSkillProperty0': 1.1, 'exampleSkillProperty1': 10.1})
        self.assertTrue(isinstance(decodedGoal['skillProperties']['exampleSkillProperty0'], float))
        self.assertTrue(isinstance(decodedGoal['skillProperties']['exampleSkillProperty1'], float))

    def test_decode_goal_with_list_property_values(self):
        goal = 'example-skill;exampleSkillProperty0=[1, 2, 3];exampleSkillProperty1=[4, 5, 6]'
        skills = [self.skill_generator(skillProperties = ['exampleSkillProperty0', 'exampleSkillProperty1'])]

        sf = SkillFactory(goal, skills)
        decodedGoal = sf.goal_decoder(goal, skills)

        self.assertEquals(decodedGoal['skillProperties'], {'exampleSkillProperty0': [1, 2, 3], 'exampleSkillProperty1': [4, 5, 6]})
        self.assertTrue(isinstance(decodedGoal['skillProperties']['exampleSkillProperty0'], list))
        self.assertTrue(isinstance(decodedGoal['skillProperties']['exampleSkillProperty1'], list))


# Unit Test for Skill Factory Class
class TestSkillFactoryClass(TestSkillFactoryClassBase):

    def test_simple_wait_skill_factory(self):
        goal = 'example-skill;exampleSkillProperty0=exampleSkillValue0;exampleSkillProperty1=exampleSkillValue1'
        skills = [self.skill_generator()]

        sf = SkillFactory(goal, skills)

        self.assertTrue(isinstance(sf.skill, GenericSkill))
        self.assertEquals(sf.skill.skillName, 'example-skill')
        self.assertEquals(sf.skill.skillType, 'GenericSkill')
        self.assertEquals(sf.skill.skillClass, 'GenericSkill')
        self.assertEquals(sf.skill.allowedSkillPropertiesKeys, ['exampleSkillProperty0', 'exampleSkillProperty1'])
        self.assertEquals(sf.skill.skillProperties, {'exampleSkillProperty0': 'exampleSkillValue0', 'exampleSkillProperty1': 'exampleSkillValue1'})

# Test Suite for Skill Factory Class
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        testSkillFactoryGoalDecoder = loader.loadTestsFromTestCase(TestSkillFactoryGoalDecoder)
        testSkillFactoryClass = loader.loadTestsFromTestCase(TestSkillFactoryClass)

        self.addTests(testSkillFactoryGoalDecoder)
        self.addTests(testSkillFactoryClass)

if __name__ == '__main__':
      rostest.rosrun(PKG, NAME, 'test_skill_factory_class.SuiteTest', sys.argv)
