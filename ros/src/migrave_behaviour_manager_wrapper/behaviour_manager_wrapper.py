import rospy

from migrave_behaviour_manager.action_interface import ActionInterface
from migrave_behaviour_manager.behaviour_manager import RobotBehaviourManager
from migrave_ros_msgs.msg import RobotAction, AffectiveState, GamePerformance

class BehaviourManagerWrapper(object):
    def __init__(self):
        action_config_path = rospy.get_param('~action_config_path', '')
        game_performance_topic = rospy.get_param('~game_performance_topic', 'game_performance')
        action_topic = rospy.get_param('~action_topic', 'robot_action')
        affective_state_topic = rospy.get_param('~affective_state_topic', 'affective_state')

        self.current_affective_state = None
        self.current_game_performance = None
        self.robot_action_msg = RobotAction()
        self.action_interface = ActionInterface(action_config_path)
        self.behaviour_manager = RobotBehaviourManager()

        self.action_pub = rospy.Publisher(action_topic, RobotAction, queue_size=1)
        self.state_sub = rospy.Subscriber(affective_state_topic,
                                          AffectiveState,
                                          self.affective_state_cb)
        self.game_performance_sub = rospy.Subscriber(game_performance_topic,
                                                     GamePerformance,
                                                     self.game_performance_cb)

    def act(self) -> None:
        """Retrieves an appropriate action for the robot and
        publishes it to an action executor.
        """
        action_name = self.behaviour_manager.get_action(None)
        action_params = self.action_interface.get_action(action_name)

        self.robot_action_msg.action_id = action_params['id']
        self.robot_action_msg.action_name = action_params['name']
        self.robot_action_msg.sentence = action_params['sentence']
        self.robot_action_msg.gesture_type = action_params['gesture']
        self.robot_action_msg.face_expression = action_params['face_expression']
        self.robot_action_msg.stamp = rospy.Time.now()

        self.action_pub.publish(self.robot_action_msg)

    def affective_state_cb(self, affective_state_msg: AffectiveState) -> None:
        self.current_affective_state = affective_state_msg

    def game_performance_cb(self, game_performance_msg: GamePerformance) -> None:
        self.current_game_performance = game_performance_msg
