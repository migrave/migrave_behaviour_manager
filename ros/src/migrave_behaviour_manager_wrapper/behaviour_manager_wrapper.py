import rospy

from migrave_behaviour_manager.action_interface import ActionInterface
from migrave_behaviour_manager.behaviour_manager import RobotBehaviourManager
from migrave_ros_msgs.msg import RobotAction, AffectiveState, \
                                 GamePerformance, TherapistFeedback

class BehaviourManagerWrapper(object):
    """ROS wrapper for a behaviour manager that chooses actions for a robot to perform
    based on an estimated affective state, ongoing game performance, and therapist
    feedback for overriding actions.

    The wrapper exposes the following parameters:
    therapist_feedback_topic: str -- topic on which therapist feedback is published
    game_performance_topic: str -- topic on which ongoing game performance is published
    affective_state_topic: str -- topic on which the affective state of a person is published
    suggested_action_topic: str -- topic on which suggested robot actions are published
    robot_action_topic: str -- topic on which actual robot actions to be executed are published

    """
    component_name = 'behaviour_manager_wrapper'

    game_performance_topic = None
    affective_state_topic = None
    therapist_feedback_topic = None
    suggested_action_topic = None
    robot_action_topic = None

    suggested_action_pub = None
    robot_action_pub = None
    therapist_feedback_sub = None
    affective_state_sub = None
    game_performance_sub = None

    def __init__(self):
        action_config_path = rospy.get_param('~action_config_path', '')
        self.therapist_feedback_topic = rospy.get_param('~therapist_feedback_topic', '/migrave_therapist_feedback')
        self.game_performance_topic = rospy.get_param('~game_performance_topic', '/migrave_game_performance')
        self.affective_state_topic = rospy.get_param('~affective_state_topic',
                                                     '/migrave_perception/person_state_estimator/affective_state')
        self.suggested_action_topic = rospy.get_param('~suggested_action_topic', 'suggested_robot_action')
        self.robot_action_topic = rospy.get_param('~robot_action_topic', 'robot_action')

        self.current_affective_state = None
        self.current_game_performance = None
        self.therapist_feedback = None
        self.robot_action_msg = RobotAction()
        self.action_interface = ActionInterface(action_config_path)
        self.behaviour_manager = RobotBehaviourManager()

        self.setup_ros()

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

        self.robot_action_pub.publish(self.robot_action_msg)

    def affective_state_cb(self, affective_state_msg: AffectiveState) -> None:
        self.current_affective_state = affective_state_msg

    def game_performance_cb(self, game_performance_msg: GamePerformance) -> None:
        self.current_game_performance = game_performance_msg

    def therapist_feedback_cb(self, therapist_feedback_msg: TherapistFeedback) -> None:
        self.therapist_feedback = therapist_feedback_msg

    def setup_ros(self):
        rospy.loginfo('[{0}] Initialising publisher on topic {1}'.format(self.component_name,
                                                                         self.suggested_action_topic))
        self.suggested_action_pub = rospy.Publisher(self.suggested_action_topic,
                                                    RobotAction,
                                                    queue_size=1)
        rospy.loginfo('[{0}] Initialised {1} publisher'.format(self.component_name,
                                                               self.suggested_action_topic))

        rospy.loginfo('[{0}] Initialising publisher on topic {1}'.format(self.component_name,
                                                                         self.robot_action_topic))
        self.robot_action_pub = rospy.Publisher(self.robot_action_topic,
                                                RobotAction,
                                                queue_size=1)
        rospy.loginfo('[{0}] Initialised {1} publisher'.format(self.component_name,
                                                               self.robot_action_topic))

        rospy.loginfo('[{0}] Initialising subscriber on topic {1}'.format(self.component_name,
                                                                          self.therapist_feedback_topic))
        self.therapist_feedback_sub = rospy.Subscriber(self.therapist_feedback_topic,
                                                       TherapistFeedback,
                                                       self.therapist_feedback_cb)
        rospy.loginfo('[{0}] Initialised {1} subscriber'.format(self.component_name,
                                                                self.therapist_feedback_topic))

        rospy.loginfo('[{0}] Initialising subscriber on topic {1}'.format(self.component_name,
                                                                          self.affective_state_topic))
        self.affective_state_sub = rospy.Subscriber(self.affective_state_topic,
                                                    AffectiveState,
                                                    self.affective_state_cb)
        rospy.loginfo('[{0}] Initialised {1} subscriber'.format(self.component_name,
                                                                self.affective_state_topic))

        rospy.loginfo('[{0}] Initialising subscriber on topic {1}'.format(self.component_name,
                                                                          self.game_performance_topic))
        self.game_performance_sub = rospy.Subscriber(self.game_performance_topic,
                                                     GamePerformance,
                                                     self.game_performance_cb)
        rospy.loginfo('[{0}] Initialised {1} subscriber'.format(self.component_name,
                                                                self.game_performance_topic))
