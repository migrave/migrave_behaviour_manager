from typing import Dict
import os
import numpy as np
import rospy
import actionlib
from std_msgs.msg import String, Bool

from qt_robot_interface.srv import behavior_talk_text, emotion_show, audio_play
from qt_gesture_controller.srv import gesture_play as qt_gesture_play

from mas_tools.file_utils import load_yaml_file
from migrave_ros_msgs.msg import GamePerformance, StampedString, GetAverageEngagementAction, GetAverageEngagementGoal

class GameBase(object):
    """! Base class for implementing games.
    """

    ## ID of the performed game
    game_id = None

    ## list of game activity names
    tasks = None

    ## dictionary of activity names and their IDs
    game_activity_ids = None

    ## dictionary of activity names and their difficulty levels
    difficulty_levels = None

    ## currently performed task
    task = "waiting"

    ## index of the currently performed task
    task_idx = -1

    ## current game status
    game_status = "waiting"

    ## current task status
    task_status = "waiting"

    ## result of the latest answer
    result = "waiting"

    ## number of rounds that have been played of the current activity
    round_count = 0

    ## correct answer counter
    correct_answer_count = 0

    ## correct answer counter for the ordering activity
    partially_correct_answer_count = 0

    ## number of times an incorrect answer has been given in the current activity
    wrong_answer_count = 0

    ## list of IDs of received answer messages (to prevent processing messages multiple times)
    received_answer_msg_ids = None

    ## list of IDs of received status messages (to prevent processing messages multiple times)
    received_status_msg_ids = None

    game_performance = None

    msg_acknowledged = False

    gesture_speed = 1.0

    avg_engagement_window_s = 2.0

    def __init__(self, game_config_dir_path: str,
                 game_id: str,
                 game_status_topic: str,
                 game_answer_topic: str,
                 game_performance_topic: str,
                 msg_acknowledgement_topic: str = '/migrave/msg_acknowledgement',
                 avg_engagement_action: str = '/migrave_perception/get_avg_engagement',
                 waiting_times_before_robot_prompt_s = {'level1': 5., 'level2': 12.5, 'level3': 25.}):
        self.game_id = game_id
        self.game_status_topic = game_status_topic
        self.game_answer_topic = game_answer_topic
        self.game_performance_topic = game_performance_topic
        self.msg_acknowledgement_topic = msg_acknowledgement_topic
        self.avg_engagement_action = avg_engagement_action
        self.game_performance = GamePerformance()
        self.msg_acknowledged = False
        self.received_answer_msg_ids = []
        self.received_status_msg_ids = []
        self.game_config = load_yaml_file(os.path.join(game_config_dir_path, game_id + '.yaml'))

        self.game_id = self.game_config["game_id"]
        self.tasks = self.game_config["general_game_params"]["tasks"]
        self.game_activity_ids = self.game_config["general_game_params"]["game_activity_ids"]
        self.difficulty_levels = self.game_config["general_game_params"]["difficulty_levels"]
        self.end_sentence = self.game_config["general_game_params"]["end_sentence"]
        # self.celebration_sound_name = self.game_config["media_params"]["celebration_sound_name"]

        self.game_activity_start_time = None
        self.waiting_times_before_robot_prompt_s = waiting_times_before_robot_prompt_s

        # level 1: basic prompt to press something on the tablet
        # level 2: prompt due to disengagement
        # level 3: final prompt before a therapist intervenes
        # gone: the player is not visible to the robot anymore
        self.coping_reactions_performed = {'level1': False,
                                           'level2': False,
                                           'level3': False,
                                           'gone': False}

        self.game_state = 'idle'

        self.setup_ros()

    def monitor_game(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.task_status == 'waiting_for_child_input':
                elapsed_time = rospy.Time.now().to_sec() - self.game_activity_start_time
                coping_reactions_remaining = sum([1 if not x else 0 for (_,x) in self.coping_reactions_performed.items()]) > 0
                if coping_reactions_remaining:
                    # A level 1 reaction is simply prompting the player to press the tablet
                    if not self.coping_reactions_performed['level1'] and elapsed_time > self.waiting_times_before_robot_prompt_s['level1']:
                        self.say_text("Tippe auf das Tablet.")
                        self.coping_reactions_performed['level1'] = True
                    # A level 3 reaction is again prompting the player to press the tablet;
                    # this is the last reaction level before a therapist intervenes
                    elif not self.coping_reactions_performed['level3'] and elapsed_time > self.waiting_times_before_robot_prompt_s['level3']:
                        self.say_text("Tippe auf das Tablet und du wirst einen Stern bekommen.")
                        self.coping_reactions_performed['level3'] = True

                    # For a level 2 reaction, we are monitoring the player's engagement, such that
                    # there are two cases to distinguish: (i) the player is visible to the robot,
                    # but is disengaged, in which case we prompt with motivating feedback, and
                    # (ii) the player is not visible to the robot anymore (has gone out of sight),
                    # in which case we prompt them to come back. Either of these is only performed once.
                    # We only monitor the engagement until a level 3 reaction is performed because
                    # a therapist will intervene after that
                    if self.waiting_times_before_robot_prompt_s['level3'] > elapsed_time > self.waiting_times_before_robot_prompt_s['level2']:
                        avg_engagement_goal = GetAverageEngagementGoal()
                        avg_engagement_goal.end_time = rospy.Time.now().to_sec()
                        avg_engagement_goal.start_time = rospy.Time.now().to_sec() - self.avg_engagement_window_s
                        self.avg_engagement_client.send_goal(avg_engagement_goal)
                        self.avg_engagement_client.wait_for_result(rospy.Duration(2))
                        avg_engagement_result = self.avg_engagement_client.get_result()
                        if avg_engagement_result is not None:
                            if not avg_engagement_result.avg_engagement:
                                if not self.coping_reactions_performed['gone']:
                                    self.say_text("Hey, wo bist du? Komm zurueck!")
                                    self.coping_reactions_performed['gone'] = True
                            else:
                                mean_engagement = np.mean(avg_engagement_result.avg_engagement)
                                if mean_engagement < 0:
                                    if not self.coping_reactions_performed['level2']:
                                        self.say_text("Hey, wo bist du? Tippe auf das Tablet!")
                                        self.coping_reactions_performed['level2'] = True
                                else:
                                    self.coping_reactions_performed['level2'] = False

    def game_start(self):
        # publish game performance
        self.game_performance.answer_correctness = -1
        self.game_performance.game_activity.game_id = self.game_id
        self.game_performance.game_activity.game_activity_id = "game_init"
        self.game_performance.stamp = rospy.Time.now()
        self.game_performance_pub.publish(self.game_performance)
        
        rospy.set_param("/migrave/game_performance/game_id", self.game_id)

        # reset counters
        self.task_idx = 0
        self.round_count = 0
        self.wrong_answer_count
        self.correct_answer_count = 0

    def task_start(self):
        rospy.loginfo("Starting new task")
        self.wrong_answer_count = 0

        # reset the counters when starting a new task;
        # publish the game performance when resuming
        if "resume" not in self.game_status:
            self.round_count = 0
            self.correct_answer_count = 0
        else:
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = self.game_status
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)
        
        if "resume" in self.game_status:
            task_name = self.game_status[0:self.game_status.find('_resume')]
            self.task_idx = self.tasks.index(task_name)
        elif "next_task" in self.game_status:
            self.task_idx += 1
            if self.task_idx != len(self.tasks):
                self.say_text("Lass uns weiter machen!")
                rospy.sleep(2)

        # we complete the game if the previous activity was the last one
        if self.task_idx == len(self.tasks):
            rospy.loginfo("Game complete; no new task to start")
            self.say_text("Geschafft, du hast super mitgemacht!")
            self.show_emotion("kiss")
            self.say_text(self.end_sentence)
            self.task_status = "done"
            self.task_status_pub.publish("done")
        else:
            self.task = self.tasks[self.task_idx]
            rospy.loginfo(f"Running task: {self.task}")
            rospy.loginfo("Publishing task status: running")
            self.task_status = "running"
            self.task_status_pub.publish('running')

    def evaluate_answer(self, feedback_emotions: Dict[str, str],
                        feedback_texts: Dict[str, str],
                        feedback_sounds: Dict[str, str] = None) -> None:
        result = self.result
        self.game_performance.stamp = rospy.Time.now()
        self.game_performance.game_activity.game_id = self.game_id
        self.game_performance.game_activity.game_activity_id = self.game_activity_ids[self.task]
        self.game_performance.game_activity.difficulty_level = self.difficulty_levels[self.task]
        self.game_performance.answer_correctness = 1 if "right" in result else 0
        self.game_performance_pub.publish(self.game_performance)
        rospy.loginfo("Publish game performance")
        # emotion = feedback_emotions[result]
        # self.show_emotion(emotion)
        text = feedback_texts[result]
        self.say_text(text)

        if result == "right":
            self.partially_correct_answer_count = 0 
            self.round_count += 1
            self.correct_answer_count += 1
            self.wrong_answer_count = 0
            rospy.loginfo(f"Count: {self.round_count}; Correct: {self.correct_answer_count}")
            if feedback_sounds != None:
                self.audio_play(str(feedback_sounds[result]) + ".mp3")
            self.say_text("Dafür bekommst du einen Stern!")
            self.audio_play("rfh-koeln/MIGRAVE/Reward2")
            if self.task.find('order_steps') != -1:
                image = f"{self.correct_answer_count}_2Token"
                rospy.loginfo(f"Publish image: {self.correct_answer_count}_2Token")
            else: 
                image = f"{self.correct_answer_count}Token"
                rospy.loginfo(f"Publish image: {self.correct_answer_count}Token")
            rospy.loginfo(image)
            self.tablet_image_pub.publish(image)

            rospy.sleep(3)
            if self.round_count == 2 and self.task.find('order_steps') != -1:
                rospy.loginfo("Ending current task")
                self.finish_one_task()
            elif self.round_count == 5:
                rospy.loginfo("Ending current task")
                self.finish_one_task()
            else:
                rospy.loginfo("Continuing current task")
                # self.show_emotion("kiss")
                self.say_text("Noch einmal!")
                self.start_new_round_and_grade()
                self.reset_coping_reactions()
        elif result == "wrong":
            self.wrong_answer_count += 1
            rospy.loginfo(f"Wrong answer count: {self.wrong_answer_count}")
            self.retry_after_wrong()
            self.reset_coping_reactions()
        elif result == "partially_correct":
            self.wrong_answer_count = 0
            self.partially_correct_answer_count += 1
            rospy.loginfo("Continuing current ordering round")
            # self.show_emotion("kiss")
            self.start_new_round_and_grade()
            self.reset_coping_reactions()

    def retry_after_wrong(self):
        raise NotImplementedError("retry_after_wrong needs to be overridden")

    def start_new_round_and_grade(self):
        raise NotImplementedError("start_new_round_and_grade needs to be overridden")

    def game_answer_cb(self, msg: StampedString):
        rospy.loginfo("[game_answer_cb] Publishing acknowledgement")
        self.msg_acknowledgement_pub.publish(True)

        self.task_status = 'processing_answer'

        # we sleep for a while to allow subscribers on
        # the game side to receive the acknowledgement
        rospy.sleep(3.)

        # we reset the acknowledgement due to the message latching
        rospy.loginfo("[game_answer_cb] Resetting acknowledgement")
        self.msg_acknowledgement_pub.publish(False)
        if msg.id in self.received_answer_msg_ids:
            return

        self.received_answer_msg_ids.append(msg.id)
        self.result = msg.data
        rospy.loginfo(f"Game result: {self.result}")
        self.evaluate_answer()

    def game_status_cb(self, msg: String):
        rospy.loginfo("[game_status_cb] Publishing acknowledgement")
        self.msg_acknowledgement_pub.publish(True)

        # we sleep for a while to allow subscribers on
        # the game side to receive the acknowledgement
        rospy.sleep(3.)

        # we reset the acknowledgement due to the message latching
        rospy.loginfo("[game_status_cb] Resetting acknowledgement")
        self.msg_acknowledgement_pub.publish(False)
        if msg.id in self.received_status_msg_ids:
            return

        self.received_status_msg_ids.append(msg.id)
        self.game_status = msg.data

        # start the game
        if "start" in self.game_status:
            rospy.loginfo(f"Game {self.game_id} starts")
            self.game_start()
            self.task_start()
        # end the game
        elif "end" in self.game_status:
            rospy.loginfo(f"Game {self.game_id} ends")
            # publish game performance when ending
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = "game_end"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)
        else:
            self.task_start()

    def game_performance_cb(self, msg: GamePerformance):
        self.game_performance = msg
        rospy.set_param("/migrave/game_performance/participant_id", msg.person.id)

    def msg_acknowledgement_cb(self, msg: Bool):
        self.msg_acknowledged = True

    def audio_play(self, audio: str):
        qt_audio_play = rospy.ServiceProxy("/qt_robot/audio/play", audio_play)
        rospy.wait_for_service("/qt_robot/audio/play")
        try:
            # publish robot action info (start playing audio)
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = "robot_audio_start"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

            # call the audio play service
            qt_audio_play(audio, "")

            # publish robot action info (finish audio)
            self.game_performance.game_activity.game_activity_id = "robot_audio_end"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def gesture_play(self, gesture: str):
        qt_gesture_play_proxy = rospy.ServiceProxy("/qt_robot/gesture/play",
                                                   qt_gesture_play)
        # block/wait for ros service
        rospy.wait_for_service("/qt_robot/gesture/play")
        try:
            # publish robot action info (start moving)
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = "robot_gesture_start"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

            # call the gesture play service
            qt_gesture_play_proxy(gesture, self.gesture_speed)

            # publish robot action info (finish moving)
            self.game_performance.game_activity.game_activity_id = "robot_gesture_end"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def say_text(self, text: str):
        qt_talk_text = rospy.ServiceProxy("/qt_robot/behavior/talkText",
                                          behavior_talk_text)
        # block/wait for ros service
        rospy.wait_for_service("/qt_robot/behavior/talkText")
        try:
            # publish robot action info (start talking)
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = "robot_talk_start"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

            # call the talk text service
            qt_talk_text(text)

            # publish robot action info (finish talking)
            self.game_performance.game_activity.game_activity_id = "robot_talk_end"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def show_emotion(self, emotion: str):
        qt_emotion_show = rospy.ServiceProxy("/qt_robot/emotion/show", emotion_show)
        rospy.wait_for_service("/qt_robot/emotion/show")
        try:
            # publish robot action info (start showing emotion)
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = "robot_emotion_start"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

            qt_emotion_show("QT/" + emotion)

            # publish robot action info (finish showing emotion)
            self.game_performance.game_activity.game_activity_id = "robot_emotion_end"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def finish_one_task(self):
        self.round_count = 0
        self.correct_answer_count = 0
        self.wrong_answer_count = 0

        self.say_text("Super, du hast alle Sterne gesammelt!")
        self.show_emotion("kiss")
        self.gesture_play("QT/happy")
        self.say_text("Gut gemacht!")
        rospy.loginfo("Publishing task status: finish")
        self.task_status_pub.publish("finish")

        rospy.loginfo("Publishing image: fireworks")
        self.say_text("Schau mal auf das Tablet. Da ist ein Feuerwerk für dich!")
        self.tablet_image_pub.publish("fireworks")
        rospy.sleep(1)

        # rospy.loginfo("Publishing sounds: Fireworks")
        # rospy.sleep(2)
        # self.audio_play(self.celebration_sound_name)

    def setup_ros(self):
        task_status_topic = f"/migrave_game_{self.game_id}/task_status"
        rospy.loginfo('[%s] Initialising publisher on topic %s', self.game_id, task_status_topic)
        self.task_status_pub = rospy.Publisher(task_status_topic, String, queue_size=10)
        rospy.loginfo('[%s] Initialised %s publisher', self.game_id, task_status_topic)

        tablet_image_topic = f"/migrave_game_{self.game_id}/tablet_image"
        rospy.loginfo('[%s] Initialising publisher on topic %s', self.game_id, tablet_image_topic)
        self.tablet_image_pub = rospy.Publisher(tablet_image_topic, String, queue_size=10)
        rospy.loginfo('[%s] Initialised %s publisher', self.game_id, tablet_image_topic)

        educator_choice_topic = f"/migrave_game_{self.game_id}/show_educator_choice"
        rospy.loginfo('[%s] Initialising publisher on topic %s', self.game_id, educator_choice_topic)
        self.show_educator_choice_pub = rospy.Publisher(educator_choice_topic, Bool, queue_size=10)
        rospy.loginfo('[%s] Initialised %s publisher', self.game_id, educator_choice_topic)

        rospy.loginfo('[%s] Initialising publisher on topic %s', self.game_id, self.game_performance_topic)
        self.game_performance_pub = rospy.Publisher(self.game_performance_topic,
                                                    GamePerformance, queue_size=1)
        rospy.loginfo('[%s] Initialised %s publisher', self.game_id, self.game_performance_topic)

        rospy.loginfo('[%s] Initialising publisher on topic %s', self.game_id, self.msg_acknowledgement_topic)
        self.msg_acknowledgement_pub = rospy.Publisher(self.msg_acknowledgement_topic,
                                                       Bool, queue_size=1, latch=True)
        rospy.loginfo('[%s] Initialised %s publisher', self.game_id, self.msg_acknowledgement_topic)

        rospy.loginfo('[%s] Initialising subscriber on topic %s', self.game_id, self.game_performance_topic)
        self.game_performance_sub = rospy.Subscriber(self.game_performance_topic,
                                                     GamePerformance,
                                                     self.game_performance_cb)
        rospy.loginfo('[%s] Initialised %s subscriber', self.game_id, self.game_performance_topic)

        rospy.loginfo('[%s] Initialising subscriber on topic %s', self.game_id, self.game_status_topic)
        self.game_status_sub = rospy.Subscriber(self.game_status_topic, StampedString,
                                                self.game_status_cb)
        rospy.loginfo('[%s] Initialised %s subscriber', self.game_id, self.game_status_topic)

        rospy.loginfo('[%s] Initialising subscriber on topic %s', self.game_id, self.game_answer_topic)
        self.game_answer_sub = rospy.Subscriber(self.game_answer_topic, StampedString,
                                                self.game_answer_cb)
        rospy.loginfo('[%s] Initialised %s subscriber', self.game_id, self.game_answer_topic)

        rospy.loginfo('[%s] Initialising subscriber on topic %s', self.game_id, self.msg_acknowledgement_topic)
        self.msg_acknowledgement_sub = rospy.Subscriber(self.msg_acknowledgement_topic, Bool,
                                                        self.msg_acknowledgement_cb)
        rospy.loginfo('[%s] Initialised %s subscriber', self.game_id, self.msg_acknowledgement_topic)

        self.avg_engagement_client = actionlib.SimpleActionClient(self.avg_engagement_action, GetAverageEngagementAction)
        rospy.loginfo('[%s] Waiting for action server %s', self.game_id, self.avg_engagement_action)
        self.avg_engagement_client.wait_for_server(rospy.Duration(5))
        rospy.loginfo('[%s] Server %s initialised or timeout expired', self.game_id, self.avg_engagement_action)

    def reset_coping_reactions(self):
        self.game_activity_start_time = rospy.Time.now().to_sec()
        for x in self.coping_reactions_performed:
            self.coping_reactions_performed[x] = False
        self.task_status = 'waiting_for_child_input'

    def has_performed_coping(self):
        """Returns True if any coping reaction has been performed (as indicated by the
        entries in self.coping_reactions_performed); returns False otherwise.
        """
        return sum([1 if x else 0 for (_,x) in self.coping_reactions_performed.items()]) > 0