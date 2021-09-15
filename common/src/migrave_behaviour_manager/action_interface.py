from typing import Dict

import yaml
import rospy

class ActionInterface(object):
    'The purpose of this class is to fetch actions defined in the config/actions directory'

    def __init__(self, config_path: str):
        self.actions = []
        self.path = config_path
        self.refresh()

    def refresh(self):
        self.actions = self.fetch_actions()

    def fetch_actions(self):
        with open(self.path, "r") as config:
            try:
                actions = yaml.safe_load(config)

            except yaml.YAMLError as exc:
                rospy.logerr(exc)

        return actions

    def get_actions(self):
        return self.actions

    def get_action(self, action_name: str) -> Dict[str, str]:
        actions_list = [action for action in self.actions if action_name == action['id']]
        if not actions_list:
            return None
        if len(actions_list) > 1:
            raise ValueError
        return actions_list[0]


# if __name__ == '__main__':
#     actionint = ActionInterface()
#     print(actionint.get_actions())
#     print(actionint.get_action('self_introduce'))
