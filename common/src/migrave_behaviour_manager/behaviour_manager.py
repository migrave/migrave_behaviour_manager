"""Module for functionalities taking care of robot behaviour management.
"""

import numpy as np

from migrave_behaviour_manager.episodic_memory import EpisodicMemory
from migrave_behaviour_manager.procedural_memory import ProceduralMemory

class RobotBehaviourManager(object):
    """Main component for robot behaviour management.
    """
    def __init__(self):
        self.episodic_memory = EpisodicMemory()
        self.procedural_memory = ProceduralMemory()

    def get_action(self, state):
        # we check if we have a known action for the current state
        known_action = self.episodic_memory.get_action_for_state(state)
        if known_action:
            return known_action

        # check which actions are possible from the procedural memory
        possible_actions, action_preferences = self.procedural_memory.get_possible_actions(state)
        if not possible_actions:
            return 'do_nothing'

        # if action preferences are available, select the action with
        # the highest preference; select a random action otherwise
        selected_action = None
        if action_preferences:
            sorted_preference_indices = np.argsort(action_preferences)
            selected_action = possible_actions[sorted_preference_indices[-1]]
        else:
            selected_action = np.random.choice(possible_actions)
        return selected_action
