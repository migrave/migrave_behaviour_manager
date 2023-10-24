#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import random
import rospy

from migrave_game_library.game_base import GameBase
from migrave_ros_msgs.msg import UIActivityParameters

class MigraveGameToothBrush(GameBase):
    def __init__(self, game_config_dir_path: str,
                 game_id: str,
                 game_status_topic: str = "/migrave_game_tooth_brush/status",
                 game_answer_topic: str = "/migrave_game_tooth_brush/answer",
                 game_performance_topic: str = "/migrave/game_performance"):
        super(MigraveGameToothBrush, self).__init__(game_config_dir_path,
                                                game_id,
                                                game_status_topic,
                                                game_answer_topic,
                                                game_performance_topic)

        self.target_objects = self.game_config["game_specific_params"]["target_objects"]
        self.distractor_objects = self.game_config["game_specific_params"]["distractor_objects"]
        self.target_activities = self.game_config["game_specific_params"]["target_activities"]
        self.ordered_activities = self.game_config["game_specific_params"]["target_activities"]
        self.first = self.game_config["game_specific_params"]["first"]
        self.after = self.game_config["game_specific_params"]["after"]
        self.answer_what_comes_first = self.game_config["media_params"]["answer_what_comes_first"]
        self.initial_phrase = ["Schau auf das Tablet!", "Guck auf das Tablet!", "Schau mal auf das Tablet!", "Guck mal auf das Tablet!",  "Sieh mal auf das Tablet!"]
        self.activity_parameters = UIActivityParameters()
        self.activity_parameters_pub = rospy.Publisher("/migrave_game_tooth_brush/activity_parameters",
                                                       UIActivityParameters, queue_size=1)
        self.object = "Waiting"
        self.object_image = "Kein"
        length = len(self.ordered_activities)
        self.ordering_sequence = [1, length - 2]
        self.ordering_game_idx = 0
        self.ordering_activity = ""
        self.en_to_de_map = {"water": "das wasser", "towel": "das handtuch", "toothbrush": "die Zahnbürste", "black_square":"", 
        "toothpaste": "die Zahnpasta", "take_water": "Nimm Wasser", "wet_toothbrush": "Befeuchte deine Zahnbürste", 
        "open_toothpaste": "öffne die Zahnpasta", "take_toothpaste": "Nimm Zahnpasta", "brushing": "putz deine Zähne", 
        "clean_toothbrush": "reinige deine Zahnbürste", "rinse_mouth": "reinige deinen Mund", "wipe_mouth": "wisch dir deinen Mund ab"}
        self.monitor_game()

    def game_start(self):
        super().game_start()
        # rospy.loginfo("Tooth brush game starts")
        # self.say_text("Heute lernst du wie du deine Zähne putzen kannst. Fangen wir an!")
        # self.show_emotion("happy")
        # self.say_text("Hände auf den Tisch. Schau mich an.")
        # self.show_emotion("happy")

    def task_start(self):
        super().task_start()
        if self.task_status == "done":
            return

        rospy.loginfo("Tooth brush game starts")
        self.say_text("Heute lernst du wie du deine Zähne putzen kannst. Fangen wir an!")
        self.show_emotion("happy")
        self.say_text("Hände auf den Tisch. Schau mich an.")
        self.show_emotion("happy")

        if self.task in ["object_vs_objects", "object_vs_objects_resume"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round()

        elif self.task in ["first_activity","first_activity_resume"]:
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
        elif self.task in ["first_activity","first_activity_resume"]:
            self.start_new_selection_round()
        elif self.task in ["order_steps","order_steps_resume"]:
            self.start_new_ordering_round()
    
    def start_new_differentiation_round(self):
        possible_objects = list(self.target_objects)
        self.object = random.choice(possible_objects)
        self.object_image = f"{self.object}"

        distractors = random.sample(self.distractor_objects, 2)
        self.activity_parameters.correct_image = [self.object_image]
        self.activity_parameters.correct_image_highlighted = [f"{self.object_image}-highlighted"]

        list_of_images = [self.object_image, distractors[0], distractors[1]]
        self.activity_parameters.images = random.sample(list_of_images, len(list_of_images))

        if self.round_count == 0:
            look_at_tablet = random.choice(self.initial_phrase)
            self.say_text(f"{look_at_tablet} Was brauchst du zum Zähneputzen?")
        else:
            self.say_text(f"Was brauchst du zum Zähneputzen?")

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
    
        self.object = self.first[self.round_count]
        self.object_image = f"{self.object}"
        distractors = self.after[self.round_count]
        distractor_images = f"{distractors}"

        if self.round_count == 0:
            audio_text = look_at_tablet + "Was kommt zuerst? Tippe auf das richtige Bild!"
        else:
            audio_text = "Was kommt zuerst? Tippe auf das richtige Bild!"
        self.say_text(audio_text)
        
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
        self.object_image = self.object[self.ordering_game_idx]
        self.activity_parameters.images = list_of_images

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_ordering_round] Publishing task parameters " +\
                          f"-- object: {self.object}, current index: {self.partially_correct_answer_count}, " +\
                          f"all images: {list_of_images}, " +\
                          f"correct image: {self.activity_parameters.correct_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)
        
        look_at_tablet = random.choice(self.initial_phrase)
        self.ordering_activity = self.en_to_de_map[self.object[self.partially_correct_answer_count]]

        sentence_to_say = ""
        if self.partially_correct_answer_count == 0:
            sentence_to_say = "Bringe die Bilder in die richtigee Reihenfolge! Was kommt zuerst?"
        elif self.partially_correct_answer_count == 1:
            sentence_to_say = "Was kommt jetzt?"
        elif self.partially_correct_answer_count == 2:
            self.ordering_game_idx += 1
            sentence_to_say = "Was kommt jetzt?"

        if self.round_count == 0:
            self.say_text(f"{look_at_tablet} {sentence_to_say}")
        else:
            self.say_text(f"{sentence_to_say}")

    def evaluate_answer(self):
        if self.wrong_answer_count > 0 or self.has_performed_coping():
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
            "object_vs_objects": fr"\emph\ {self.en_to_de_map[self.object_image]}! \emph\ {self.possitive_feedback}!",
            "first_activity": fr"\emph\ {self.answer_what_comes_first[self.round_count]}! \emph\ {self.possitive_feedback}!"
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

        if self.task in ["order_steps","order_steps_resume"]:
            self.say_text(f"Bring die Bilder in die richtigee Reihenfolge!")
        else:
            self.say_text(f"Tippe auf das richtige Bild!")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- correct: {self.object}, image: {self.object_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
