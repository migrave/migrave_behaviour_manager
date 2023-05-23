#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import random
import rospy

from migrave_game_library.game_base import GameBase
from migrave_ros_msgs.msg import UIActivityParameters

class MigraveGameHandWash(GameBase):
    def __init__(self, game_config_dir_path: str,
                 game_id: str,
                 game_status_topic: str = "/migrave_game_hand_wash/status",
                 game_answer_topic: str = "/migrave_game_hand_wash/answer",
                 game_performance_topic: str = "/migrave/game_performance"):
        super(MigraveGameHandWash, self).__init__(game_config_dir_path,
                                                game_id,
                                                game_status_topic,
                                                game_answer_topic,
                                                game_performance_topic)

        self.target_objects = self.game_config["game_specific_params"]["target_objects"]
        self.distractor_objects = self.game_config["game_specific_params"]["distractor_objects"]
        self.target_activities = self.game_config["game_specific_params"]["target_activities"]
        self.ordered_activities = self.game_config["game_specific_params"]["target_activities"]
        self.when_to_wash = self.game_config["game_specific_params"]["wash"]
        self.when_to_not_wash = self.game_config["game_specific_params"]["not_wash"]
        self.first = self.game_config["game_specific_params"]["first"]
        self.after = self.game_config["game_specific_params"]["after"]
        self.initial_phrase = ["Schau auf das Tablet!", "Guck auf das Tablet!", "Schau mal auf das Tablet!", "Guck mal auf das Tablet!",  "Sieh mal auf das Tablet!"]
        self.questions_when_to_wash = self.game_config["media_params"]["questions_when_to_wash"]
        self.questions_what_comes_first = self.game_config["media_params"]["questions_what_comes_first"]
        
        self.activity_parameters = UIActivityParameters()
        self.activity_parameters_pub = rospy.Publisher("/migrave_game_hand_wash/activity_parameters",
                                                       UIActivityParameters, queue_size=1)
        self.object = "Waiting"
        self.object_image = "Kein"
        length = len(self.ordered_activities)
        self.ordering_sequence = [1, length - 2]
        self.ordering_game_idx = 0
        self.ordering_activity = ""
        self.en_to_de_map = {"soap": "die seife", "water": "das wasser", "hand_towel": "das handtuch", 
                             "open_water_tap": "Wasserhahn aufmachen", "take_soap": "Seife nehmen", 
                             "rub_hands": "Hände einseifen", "rinse_hands": "Seife abspülen", "black_square":"",
                             "close_water_tap": "Wasserhahn zumachen", "dry_hands": "Hände abtrocknen",
                             "eat": "Vor dem Essen ", "use_toilet": "Nach der Toilette", "blow_nose": "Nach dem Naseputzen", 
                             "dirty_hands": "Wenn die Hände schmutzig sind", "play_outside": "Nach dem Spielen draußen"}
        self.monitor_game()

    def game_start(self):
        super().game_start()
        rospy.loginfo("Hand wash game starts")
        self.say_text("Heute lernst du wie du deine Hände waschen kannst. Fangen wir an!")
        self.show_emotion("happy")
        self.say_text("Hände auf den Tisch. Schau mich an.")
        self.show_emotion("happy")

    def task_start(self):
        super().task_start()
        if self.task_status == "done":
            return

        if self.task in ["object_vs_objects", "object_vs_objects_resume"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round()

        elif self.task in ["first_activity","first_activity_resume", "when_to_wash","when_to_wash_resume"]:
            rospy.loginfo(f"Starting select first task '{self.task}'")
            self.start_new_selection_round()

        elif self.task in ["order_steps","order_steps_resume"]:
            rospy.loginfo(f"Starting ordering task '{self.task}'")
            self.start_new_ordering_round()
        self.reset_coping_reactions()

    def start_new_round_and_grade(self):

        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo("[start_new_round_and_grade] Publishing task status 'running'")
            self.task_status_pub.publish("running")
            rospy.sleep(2)

        if self.task in ["object_vs_objects", "object_vs_objects_resume"]:
            self.start_new_differentiation_round()
        elif self.task in ["first_activity","first_activity_resume", "when_to_wash","when_to_wash_resume"]:
            self.start_new_selection_round()
        elif self.task in ["order_steps", "order_steps_resume"]:
            self.start_new_ordering_round()

    def start_new_differentiation_round(self):
        possible_objects = list(self.target_objects)
        self.object = random.choice(possible_objects)
        self.object_image = f"{self.object}"
        
        look_at_tablet = random.choice(self.initial_phrase)
        self.say_text(f"{look_at_tablet} Was brauchst du zum Hände waschen?")

        distractors = random.sample(self.distractor_objects, 2)
        self.activity_parameters.correct_image = [self.object_image]
        self.activity_parameters.correct_image_highlighted = [f"{self.object_image}-highlighted"]

        list_of_images = [self.object_image, distractors[0], distractors[1]]
        self.activity_parameters.images = random.sample(list_of_images, len(list_of_images))

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_differentiation_round] Publishing task parameters " +\
                          f"-- object: {self.object}, image: {self.object_image}, " +\
                          f"all images: {list_of_images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)

    def start_new_selection_round(self):
        look_at_tablet = random.choice(self.initial_phrase)
        if "first_activity" in self.task:
            self.object = self.first[self.round_count]
            self.object_image = f"{self.object}"
            distractors = self.after[self.round_count]
            distractor_images = f"{distractors}"
            audio_text = look_at_tablet + "Was kommt zuerst?"

        elif "when_to_wash" in self.task:
            self.object = self.when_to_wash[self.round_count]
            self.object_image = f"{self.object}"
            distractors = self.when_to_not_wash[self.round_count]
            distractor_images = f"{distractors}"
            audio_text = look_at_tablet + "Wann solltest du deine Hände waschen?" + self.questions_when_to_wash[self.round_count]
        
        self.say_text(audio_text)
        self.say_text("Tippe auf das richtige Bild!")

        self.activity_parameters.correct_image = [self.object_image]
        self.activity_parameters.correct_image_highlighted = [f"{self.object_image}-highlighted"]

        list_of_images = [self.object_image, distractors]
        self.activity_parameters.images = random.sample(list_of_images, len(list_of_images))

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_selection_round] Publishing task parameters " +\
                          f"-- activity: {self.object}, image: {self.object_image}, " +\
                          f"distractor images: {distractor_images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
       
    def start_new_ordering_round(self):
        possible_activities = list(self.ordered_activities)
        game_idx = self.ordering_sequence[self.ordering_game_idx]
        self.object = [possible_activities[game_idx - 1], possible_activities[game_idx], possible_activities[game_idx + 1]]
        self.activity_parameters.correct_image = [self.object[self.partially_correct_answer_count]]
        self.activity_parameters.correct_image_highlighted = [f"{self.object[self.partially_correct_answer_count]}-highlighted"]
        for black_image in range(self.partially_correct_answer_count):
            self.object[black_image] = 'black_square'
        self.numbers = [str(game_idx), str(game_idx + 1), str(game_idx + 2)]
        list_of_images = random.sample(self.object, len(self.object)) + self.numbers
        self.activity_parameters.images = list_of_images

        if self.partially_correct_answer_count == 0:
            look_at_tablet = random.choice(self.initial_phrase)
            self.say_text(f"{look_at_tablet} Bringe die Bilder in die richtigee Reihenfolge, Was kommt zuerst?")
        else:
            self.say_text("Was ist der nächste Schritt?")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_ordering_round] Publishing task parameters " +\
                          f"-- object: {self.object}, current index: {self.partially_correct_answer_count}, " +\
                          f"all images: {list_of_images}, " +\
                          f"correct image: {self.activity_parameters.correct_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)

        self.object_image = self.object[self.partially_correct_answer_count]
        self.ordering_activity = self.en_to_de_map[self.object[self.partially_correct_answer_count]]

        if self.partially_correct_answer_count == 2:
            self.ordering_game_idx += 1

    def evaluate_answer(self):
        if self.wrong_answer_count > 0:
            self.possitive_feedback = random.choice(["Gut gemacht", "Gut", "Prima"])
        else:
            self.possitive_feedback = random.choice(["Wunderbar", "Klasse", "Spitzenmäßig", "Sehr gut", "Toll", "Super"])

        feedback_emotions = {
            "right": "kiss",
            "right_1": "kiss",
            "right_2": "kiss",
            "partially_correct": "kiss",
            "wrong": "",
            "wrong_1": "",
            "wrong_2": ""
        }
        right_texts = {
            "order_steps": fr" \emph\ {self.ordering_activity}! \emph\ {self.possitive_feedback}!",
            "when_to_wash": fr"\emph\ {self.en_to_de_map[self.object_image]}! \emph\ {self.possitive_feedback}!", 
            "object_vs_objects": fr"\emph\ {self.en_to_de_map[self.object_image]}! \emph\ {self.possitive_feedback}!",
            "first_activity": fr"\emph\ {self.questions_what_comes_first[self.round_count]}! \emph\ {self.possitive_feedback}!"
        }

        feedback_texts = {
            "right": right_texts[self.task],
            "wrong": "Lass es uns nochmal probieren!",
            "partially_correct": self.ordering_activity
        }
        super().evaluate_answer(feedback_emotions, feedback_texts)

    def retry_after_wrong(self):
        rospy.loginfo("[retry_after_wrong] Publishing task status 'running'")
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            self.task_status_pub.publish("running")
            rospy.sleep(2)

        if self.wrong_answer_count == 1:
            correct_image_idx = self.activity_parameters.images.index(self.activity_parameters.correct_image[0])
            self.activity_parameters.images[correct_image_idx] = self.activity_parameters.correct_image_highlighted[0]
            self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted

        elif self.wrong_answer_count == 2:
            if self.task.find('order_steps') != -1:
                correct_image_idx = self.activity_parameters.images.index(self.activity_parameters.correct_image[0])
                for black_image in range(3):
                    if black_image != correct_image_idx:
                        self.activity_parameters.images[black_image] = 'black_square'          
                self.activity_parameters.images[correct_image_idx] = self.activity_parameters.correct_image_highlighted[0]
                self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted
            else:
                self.activity_parameters.images = [self.activity_parameters.correct_image_highlighted[0]]
                self.activity_parameters.correct_image = [self.activity_parameters.correct_image_highlighted[0]]
        
        look_at_tablet = random.choice(self.initial_phrase)
        if self.task in ["order_steps","order_steps_resume"]:
            self.say_text(f"{look_at_tablet} Sortieren der Bilder in der richtigeen Reihenfolge!")
        else:
            self.say_text(f"{look_at_tablet} Tippe auf das richtigee Bild!")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_round] Publishing task parameters " +\
                          f"-- correct: {self.object}, image: {self.object_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
