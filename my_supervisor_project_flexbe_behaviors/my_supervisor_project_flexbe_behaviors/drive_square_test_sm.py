#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Define Drive Square Test Behavior. Hand-written, following the standard FlexBE generated-SM structure."""

from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from my_supervisor_project_flexbe_states.timed_twist_state import TimedTwistState


class DriveSquareTestSM(Behavior):
    """Drive forward, turn, then stop -- a flat 3-state test behavior on turtlebot3."""

    def __init__(self, node):
        super().__init__()
        self.name = 'Drive Square Test'

        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        TimedTwistState.initialize_ros(node)

    def create(self):
        _state_machine = OperatableStateMachine(outcomes=['finished'])

        with _state_machine:
            OperatableStateMachine.add('Drive_Forward',
                                       TimedTwistState(linear_x=0.15, angular_z=0.0, duration=3),
                                       transitions={'done': 'Turn_Left'},
                                       autonomy={'done': Autonomy.Off})

            OperatableStateMachine.add('Turn_Left',
                                       TimedTwistState(linear_x=0.0, angular_z=0.5, duration=3),
                                       transitions={'done': 'Stop'},
                                       autonomy={'done': Autonomy.Off})

            OperatableStateMachine.add('Stop',
                                       TimedTwistState(linear_x=0.0, angular_z=0.0, duration=1),
                                       transitions={'done': 'finished'},
                                       autonomy={'done': Autonomy.Off})

        return _state_machine
