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
        self.ordered_activities = self.game_config["game_specific_params"]["ordered_activities"]
        self.when_to_wash = self.game_config["game_specific_params"]["wash"]
        self.when_to_not_wash = self.game_config["game_specific_params"]["not_wash"]

        self.activity_parameters = UIActivityParameters()
        self.activity_parameters_pub = rospy.Publisher("/migrave_game_hand_wash/activity_parameters",
                                                       UIActivityParameters, queue_size=1)
        self.object = "Waiting"
        self.object_image = "Kein"
        length = len(self.ordered_activities)
        self.ordering_sequence = [1, length//2, length - 2]
        self.ordering_game_idx = 0
        self.en_to_de_map = {"soap": "die seife", "water": "das wasser", "hand_towel": "das handtuch", 
        "open_water_tap": "öffne den Wasserhahn", "wet_hands": "Befeuchte deine Hände", 
        "take_soap": "Nimm Seife", "rub_hands": "Reibe deine Hände", "rinse_hands": "Wasche deine Hände",
        "close_water_tap": "Schließ den Wasserhahn", "dry_hands": "Trockne deine Hände", "ordering": "Richtig! Wunderbar!",
        "eat": "Vor dem Essen", "use_toilet": "Nach der Toilette", "blow_nose": "Nach dem Nase putzen",
        "dirty_hands": "Wenn sie schmutzig sind", "play_outside": "Nach dem Spielen draußen"}
        
    def game_start(self):
        super().game_start()

        rospy.loginfo("Hand wash game starts")
        self.say_text("Heute lernst du wie du deine Hände waschen kannst. Fangen wir an!")
        self.show_emotion("showing_smile")
        self.say_text("Hände auf den Tisch. Schau mich an.")
        self.show_emotion("showing_smile")

    def task_start(self):
        super().task_start()
        if self.task_status == "done":
            return

        if self.task in ["object_vs_objects", "object_vs_objects_resume"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round()

        elif self.task in ["first_activity","first_activity_resume"]:
            rospy.loginfo(f"Starting select first task '{self.task}'")
            self.start_new_selection_round("first")

        elif self.task in ["order_steps","order_steps_resume"]:
            rospy.loginfo(f"Starting ordering task '{self.task}'")
            # we reset the list of previously used generalisation objects
            # before (re)starting a generalisation task
            self.used_ordering_activities = []
            self.start_new_ordering_round()

        elif self.task in ["when_to_wash","when_to_wash_resume"]:
            rospy.loginfo(f"Starting generalisation task '{self.task}'")
            self.start_new_selection_round("correct")

    def start_new_round_and_grade(self):
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo("[start_new_round_and_grade] Publishing task status 'running'")
            self.task_status_pub.publish("running")
            rospy.sleep(2)

        if self.task in ["object_vs_objects", "object_vs_objects_resume"]:
            self.start_new_differentiation_round()
        elif self.task in ["first_activity","first_activity_resume"]:
            self.start_new_selection_round("first")
        elif self.task in ["order_steps","order_steps_resume"]:
            self.start_new_ordering_round()
        elif self.task in ["when_to_wash","when_to_wash_resume"]:
            self.start_new_selection_round("correct")

    def start_new_differentiation_round(self):
        
        possible_objects = list(self.target_objects)
        self.object = random.choice(possible_objects)
        self.object_image = f"{self.object}"

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
        rospy.sleep(2)
        self.say_text("Schau auf das Tablet! Was brauchst du zum Hände waschen?")

    def start_new_selection_round(self, game_type):
     
        if game_type == "first":
            possible_activities = random.sample(self.target_activities, 2)
            if self.target_activities.index(possible_activities[0]) < self.target_activities.index(possible_activities[1]):
                self.object = possible_activities[0]
                distractors = possible_activities[1]
                distractor_images = f"{possible_activities[1]}"
            else:
                self.object = possible_activities[1]
                distractors = possible_activities[0]
                distractor_images = f"{possible_activities[0]}"
            
            audio_text = "Schau auf das Tablet! Was kommt zuerst? Tippe auf das richtige Bild!"
            self.object_image = f"{self.object}"

        elif game_type == "correct":
            self.object = random.sample(self.when_to_wash, 1)[0]
            self.object_image = f"{self.object}"
            distractors = random.sample(self.when_to_not_wash, 1)[0]
            distractor_images = f"{distractors}"
            audio_text = "Schau auf das Tablet! Wann solltest du deine Hände waschen?"

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
        rospy.sleep(2)
        self.say_text(audio_text)

    def start_new_ordering_round(self):

        possible_activities = list(self.ordered_activities)
        game_idx = self.ordering_sequence[self.ordering_game_idx]
        self.object = [possible_activities[game_idx - 1], possible_activities[game_idx], possible_activities[game_idx + 1]]
        self.activity_parameters.correct_image = []
        self.activity_parameters.correct_image_highlighted = []
        for i in self.object:
            self.activity_parameters.correct_image.append(f"{i}")
            self.activity_parameters.correct_image_highlighted.append(f"{i}-enumerated")

        self.numbers = [str(game_idx), str(game_idx+1), str(game_idx + 2)]
        list_of_images = random.sample(self.object, len(self.object)) + self.numbers
        self.object_image = self.object
        self.activity_parameters.images = list_of_images

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_ordering_round] Publishing task parameters " +\
                          f"-- object: {self.object}, image: {self.object_image}, " +\
                          f"all images: {list_of_images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)
        self.say_text("Schau auf das Tablet! Bringe die Bilder in die richtige Reihenfolge")
        self.object = 'ordering'
        self.ordering_game_idx = self.ordering_game_idx + 1

    def evaluate_answer(self):

        if self.wrong_answer_count > 0:
            self.possitive_feedback = random.choice(["gut gemacht", "gut"])
        else:
            self.possitive_feedback = random.choice(["Wunderbar", "Klasse", "Spitzenmäßig", "Sehr gut"])

        feedback_emotions = {
            "right": "showing_smile",
            "right_1": "showing_smile",
            "right_2": "showing_smile",
            "wrong": "",
            "wrong_1": "",
            "wrong_2": ""
        }
        right_texts = {
            "order_steps": fr"\emph\ {self.en_to_de_map[self.object]}! \emph\ {self.possitive_feedback}!",
            "when_to_wash": fr"\emph\ Richtig! \emph\ {self.en_to_de_map[self.object]}! \emph\ {self.possitive_feedback}!", 
            "object_vs_objects": fr"\emph\ Richtig! \emph\ {self.en_to_de_map[self.object]}! \emph\ {self.possitive_feedback}!",
            "first_activity": fr"\emph\ Richtig! \emph\ {self.en_to_de_map[self.object]}! \emph\ {self.possitive_feedback}!"
        }

        feedback_texts = {
            "right": right_texts[self.task],
            "wrong": "Lass es uns nochmal probieren!"
        }
        super().evaluate_answer(feedback_emotions, feedback_texts)

    def retry_after_wrong(self):
        rospy.loginfo("[retry_after_wrong] Publishing task status 'running'")
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            self.task_status_pub.publish("running")
            rospy.sleep(2)

        if self.wrong_answer_count == 1:
            if len(self.activity_parameters.correct_image) == 1:
                correct_image_idx = self.activity_parameters.images.index(self.activity_parameters.correct_image[0])
                self.activity_parameters.images[correct_image_idx] = self.activity_parameters.correct_image_highlighted[0]
                self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted
            else: 
                self.activity_parameters.images = self.activity_parameters.correct_image_highlighted + self.numbers
                self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted

        elif self.wrong_answer_count == 2:
            if len(self.activity_parameters.correct_image) == 1:
                self.activity_parameters.images = [self.activity_parameters.correct_image_highlighted[0]]
                self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted[0]
            else: 
                self.activity_parameters.images = self.activity_parameters.correct_image_highlighted + self.numbers
                self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- correct: {self.object}, image: {self.object_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)
        self.say_text(f"Schau auf das Tablet! Tippe auf das richtige Bild!")
