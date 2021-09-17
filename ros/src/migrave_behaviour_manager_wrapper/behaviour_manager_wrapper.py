from typing import Dict

import rospy
from std_msgs.msg import String

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
    event_in_topic = None
    person_state_estimator_event_topic = None
    face_feature_detector_event_topic = None

    suggested_action_pub = None
    robot_action_pub = None
    person_state_estimator_event_pub = None
    face_feature_detector_event_pub = None
    robot_action_pub = None
    therapist_feedback_sub = None
    affective_state_sub = None
    game_performance_sub = None
    event_in_sub = None

    running = False
    action_proposed = False

    def __init__(self):
        action_config_path = rospy.get_param('~action_config_path', '')
        self.therapist_feedback_topic = rospy.get_param('~therapist_feedback_topic', '/migrave_therapist_feedback')
        self.game_performance_topic = rospy.get_param('~game_performance_topic', '/migrave_game_performance')
        self.affective_state_topic = rospy.get_param('~affective_state_topic',
                                                     '/migrave_perception/person_state_estimator/affective_state')
        self.suggested_action_topic = rospy.get_param('~suggested_action_topic', 'suggested_robot_action')
        self.robot_action_topic = rospy.get_param('~robot_action_topic', 'robot_action')
        self.person_state_estimator_event_topic = rospy.get_param('~person_state_estimator_event_topic',
                                                                  '/migrave_perception/person_state_estimator/event_in')
        self.face_feature_detector_event_topic = rospy.get_param('~face_feature_detector_event_topic',
                                                                 '/migrave_perception/face_feature_detector/event_in')
        self.event_in_topic = rospy.get_param('~event_in_topic', 'event_in')
        self.ask_for_therapist_feedback_before_acting = rospy.get_param('~ask_for_therapist_feedback_before_acting', True)

        self.current_affective_state = None
        self.current_game_performance = None
        self.therapist_feedback = None
        self.robot_action_msg = RobotAction()
        self.action_interface = ActionInterface(action_config_path)
        self.behaviour_manager = RobotBehaviourManager()

        self.setup_ros()

    def propose_action(self) -> None:
        """Retrieves an appropriate action for the robot and suggests it to a therapist.
        """
        action_id = self.behaviour_manager.get_action(None)
        action_params = self.action_interface.get_action(action_id)
        self.update_action_msg_from_params(action_params)

        rospy.loginfo('[%s] Proposing action %s',
                      self.component_name, self.robot_action_msg.action_name)
        self.suggested_action_pub.publish(self.robot_action_msg)

    def act(self) -> None:
        """Publishes an action to a robot action executor. Behaves differently depending on
        whether a therapist should be asked for feedback before acting or not:
        * if feedback is not used, selects an action first
        * if feedback is used, sends the proposed action to the robot if the therapist
          agrees with the proposal; otherwise, sends the action selected by the therapist
        """
        if self.ask_for_therapist_feedback_before_acting:
            # we propose an action if we haven't done that yet and return
            # so that we don't block until the therapist has sent feedback
            if not self.action_proposed:
                self.propose_action()
                self.action_proposed = True
                return

            # we return if no therapist feedback has been received yet
            # so that we don't block the execution
            if self.therapist_feedback is None:
                return

            # we process the therapist feedback and overwrite the robot's
            # proposed action with the action selected by the therapist
            # if the therapist hasn't approved the proposed action
            if not self.therapist_feedback.proposed_action_approved and \
               self.therapist_feedback.chosen_action_id:
                action_params = self.action_interface.get_action(self.therapist_feedback.chosen_action_id)
                self.update_action_msg_from_params(action_params)

                rospy.logwarn('[%s] Overriding proposed action with %s',
                              self.component_name, action_params['name'])
            self.therapist_feedback = None
            self.action_proposed = False
        else:
            action_id = self.behaviour_manager.get_action(None)
            action_params = self.action_interface.get_action(action_id)
            self.update_action_msg_from_params(action_params)

        rospy.loginfo('[%s] Sending action %s',
                      self.component_name, self.robot_action_msg.action_name)
        self.robot_action_pub.publish(self.robot_action_msg)

        self.running = False
        self.trigger_dependencies('e_stop')

    def affective_state_cb(self, affective_state_msg: AffectiveState) -> None:
        self.current_affective_state = affective_state_msg

    def game_performance_cb(self, game_performance_msg: GamePerformance) -> None:
        self.current_game_performance = game_performance_msg

    def therapist_feedback_cb(self, therapist_feedback_msg: TherapistFeedback) -> None:
        self.therapist_feedback = therapist_feedback_msg

    def event_in_cb(self, event_msg: String) -> None:
        self.running = event_msg.data == 'e_start'
        self.trigger_dependencies('e_start')

    def trigger_dependencies(self, event_str: str) -> None:
        rospy.loginfo('[%s] Sending event %s to dependencies', self.component_name, event_str)
        self.person_state_estimator_event_pub.publish(String(data=event_str))
        self.face_feature_detector_event_pub.publish(String(data=event_str))

    def update_action_msg_from_params(self, action_params: Dict[str, str]) -> None:
        """Updates self.robot_action_msg using the parameters in action_params.

        Keyword arguments:
        action_params: Dict[str, str] -- action parameters as defined in the action config file

        """
        self.robot_action_msg.action_id = action_params['id']
        self.robot_action_msg.action_name = action_params['name']
        self.robot_action_msg.sentence = action_params['sentence']
        self.robot_action_msg.gesture_type = action_params['gesture']
        self.robot_action_msg.face_expression = action_params['face_expression']
        self.robot_action_msg.stamp = rospy.Time.now()

    def setup_ros(self):
        rospy.loginfo('[%s] Initialising publisher on topic %s', self.component_name, self.suggested_action_topic)
        self.suggested_action_pub = rospy.Publisher(self.suggested_action_topic,
                                                    RobotAction,
                                                    queue_size=1)
        rospy.loginfo('[%s] Initialised %s publisher', self.component_name, self.suggested_action_topic)

        rospy.loginfo('[%s] Initialising publisher on topic %s', self.component_name, self.robot_action_topic)
        self.robot_action_pub = rospy.Publisher(self.robot_action_topic,
                                                RobotAction,
                                                queue_size=1)
        rospy.loginfo('[%s] Initialised %s publisher', self.component_name, self.robot_action_topic)

        rospy.loginfo('[%s] Initialising publisher on topic %s', self.component_name, self.person_state_estimator_event_topic)
        self.person_state_estimator_event_pub = rospy.Publisher(self.person_state_estimator_event_topic,
                                                                String,
                                                                queue_size=1)
        rospy.loginfo('[%s] Initialised %s publisher', self.component_name, self.person_state_estimator_event_topic)

        rospy.loginfo('[%s] Initialising publisher on topic %s', self.component_name, self.face_feature_detector_event_topic)
        self.face_feature_detector_event_pub = rospy.Publisher(self.face_feature_detector_event_topic,
                                                               String,
                                                               queue_size=1)
        rospy.loginfo('[%s] Initialised %s publisher', self.component_name, self.face_feature_detector_event_topic)

        rospy.loginfo('[%s] Initialising subscriber on topic %s', self.component_name, self.therapist_feedback_topic)
        self.therapist_feedback_sub = rospy.Subscriber(self.therapist_feedback_topic,
                                                       TherapistFeedback,
                                                       self.therapist_feedback_cb)
        rospy.loginfo('[%s] Initialised %s subscriber', self.component_name, self.therapist_feedback_topic)

        rospy.loginfo('[%s] Initialising subscriber on topic %s', self.component_name, self.affective_state_topic)
        self.affective_state_sub = rospy.Subscriber(self.affective_state_topic,
                                                    AffectiveState,
                                                    self.affective_state_cb)
        rospy.loginfo('[%s] Initialised %s subscriber', self.component_name, self.affective_state_topic)

        rospy.loginfo('[%s] Initialising subscriber on topic %s', self.component_name, self.game_performance_topic)
        self.game_performance_sub = rospy.Subscriber(self.game_performance_topic,
                                                     GamePerformance,
                                                     self.game_performance_cb)
        rospy.loginfo('[%s] Initialised %s subscriber', self.component_name, self.game_performance_topic)

        rospy.loginfo('[%s] Initialising subscriber on topic %s', self.component_name, self.event_in_topic)
        self.event_in_sub = rospy.Subscriber(self.event_in_topic,
                                             String,
                                             self.event_in_cb)
        rospy.loginfo('[%s] Initialised %s subscriber', self.component_name, self.event_in_topic)
