import os
import yaml
import rospy

class ActionFactory():
    'The purpose of this class is to fetch actions defined in the config/actions directory'
    
    def __init__(self):
        self.actions = []

        # TO-DO make reconfigurable
        self.path = '../../config/actions'

    def fetch_actions(self):
        action_configs = os.listdir(self.path)
        
        for action_config in action_configs:
            with open(os.path.join(self.path, action_config), "r") as config:
                try:
                    action = yaml.safe_load(config)
                    self.actions.append(action)
                
                except yaml.YAMLError as exc:
                    rospy.logerr(exc)
        return self.actions