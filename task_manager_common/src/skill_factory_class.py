#!/usr/bin/env python

import sys

from skill_class import *

class SkillFactory(object):

    """docstring for Skill Factory."""

    def __init__(self, goal, skills):
        self.skill = self.create_skill(goal, skills)

    def goal_decoder(self, goal, skills):
        if goal == '':
            raise ValueError('goal cannot be empty')

        goalSplit = goal.split(';')
        skillName = goalSplit.pop(0)

        if not any(skillName == s['skillName'] for s in skills):
            raise ValueError('Unknown Skill named ' + skillName + '. Known Skills are: ' + str([s['skillName'] for s in skills]))

        s = next(s for s in skills if skillName == s['skillName'])
        skillType = s['skillType']
        skillClass = s['skillClass']
        allowedSkillPropertiesKeys = s['skillProperties']

        if len(goalSplit) == 0:
            raise KeyError('No skillProperty in goal')

        skillProperties = {}
        for skillPropertyKeyValuePair in goalSplit:

            skillPropertyKey, skillPropertyValue = skillPropertyKeyValuePair.split('=')

            if skillPropertyKey not in allowedSkillPropertiesKeys:
                raise KeyError('Skill property ' + str(skillPropertyKey) + ' not found in allowed Skill Properties: ' + str(allowedSkillPropertiesKeys))

            skillProperties.update({skillPropertyKey: skillPropertyValue})

        return {'skillName': skillName, 'skillType': skillType, 'skillClass': skillClass, 'allowedSkillPropertiesKeys': allowedSkillPropertiesKeys, 'skillProperties': skillProperties}

    def create_skill(self, goal, skills):
        decodedGoal = self.goal_decoder(goal, skills)

        skillClass = eval(decodedGoal['skillClass'])

        return skillClass(skillName = decodedGoal['skillName'], skillType = decodedGoal['skillType'], skillClass = decodedGoal['skillClass'], allowedSkillPropertiesKeys = decodedGoal['allowedSkillPropertiesKeys'], skillProperties = decodedGoal['skillProperties'])
