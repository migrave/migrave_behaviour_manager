import os
import yaml
import rospy

class ActionInterface():
    'The purpose of this class is to fetch actions defined in the config/actions directory'
    
    def __init__(self):
        self.actions = []

        # TO-DO make reconfigurable
        self.path = '../../../config/actions.yaml'

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


    def get_action(self, action_name):
        actions_list = [action for action in self.actions if action_name == action['name']]
        
        if not actions_list:
            return None
        elif len(actions_list) > 1:
            raise ValueError
        else:
            return actions_list[0]

    
# if __name__ == '__main__':
#     actionint = ActionInterface()
#     print(actionint.get_actions())
#     print(actionint.get_action('self_introduce'))
