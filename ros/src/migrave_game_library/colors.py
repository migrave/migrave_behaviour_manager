#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import random
import rospy

from migrave_game_library.game_base import GameBase
from migrave_ros_msgs.msg import UIActivityParameters

class MigraveGameColors(GameBase):
    def __init__(self, game_config_dir_path: str,
                 game_id: str,
                 game_status_topic: str = "/migrave_game_colors/status",
                 game_answer_topic: str = "/migrave_game_colors/answer",
                 game_performance_topic: str = "/migrave/game_performance"):
        super(MigraveGameColors, self).__init__(game_config_dir_path,
                                                game_id,
                                                game_status_topic,
                                                game_answer_topic,
                                                game_performance_topic)

        self.target_colors = self.game_config["game_specific_params"]["target_colors"]
        self.distractor_colors = self.game_config["game_specific_params"]["distractor_colors"]
        self.generalisation_objects = self.game_config["game_specific_params"]["generalisation_objects"]

        # during a generalisation task, objects should not be repeated in multiple rounds;
        # we thus keep a list of selected objects so that they can be avoided in subsequent rounds
        self.used_generalisation_objects = []

        self.activity_parameters = UIActivityParameters()
        self.activity_parameters_pub = rospy.Publisher("/migrave_game_colors/activity_parameters",
                                                       UIActivityParameters, queue_size=1)
        self.color = "Waiting"
        self.color_image = "Kein"
        self.initial_phrase = ["Schau auf das Tablet!", "Guck auf das Tablet!", "Schau mal auf das Tablet!", "Guck mal auf das Tablet!",  "Sieh mal auf das Tablet!"]

        self.en_to_de_color_map = {"red": "rot", "green": "grün",
                                   "blue": "blau", "yellow": "gelb"}

    def game_start(self):
        super().game_start()

        rospy.loginfo("Color game starts")
        self.say_text("Heute lernen wir Farben. Fangen wir an!")
        self.show_emotion("happy")
        self.say_text("Hände auf den Tisch. Schau mich an.")
        self.show_emotion("happy")
        self.say_text("Ich nenne dir eine Farbe und du tippst auf das passende Bild.")

    def task_start(self):
        super().task_start()
        if self.task_status == "done":
            return

        if self.task in ["red", "green", "blue", "yellow",
                         "red_resume", "green_resume", "blue_resume", "yellow_resume"]:
            rospy.loginfo(f"Starting simple task '{self.task}'")
            self.start_new_simple_round()
        elif self.task in ["red_vs_other", "green_vs_other", "blue_vs_other", "yellow_vs_other",
                           "red_or_yellow_vs_other", "blue_or_green_vs_other",
                           "red_vs_other_resume", "green_vs_other_resume",
                           "blue_vs_other_resume", "yellow_vs_other_resume"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round()
        elif self.task in ["three_squares", "three_cars", "three_objects"]:
            rospy.loginfo(f"Starting generalisation task '{self.task}'")

            # we reset the list of previously used generalisation objects
            # before (re)starting a generalisation task
            self.used_generalisation_objects = []
            self.start_new_generalisation_round()

    def start_new_round_and_grade(self):
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo("[start_new_round_and_grade] Publishing task status 'running'")
            self.task_status_pub.publish("running")
            rospy.sleep(2)

        if self.task in ["red", "green", "blue", "yellow",
                         "red_resume", "green_resume", "blue_resume", "yellow_resume"]:
            self.start_new_simple_round()
        elif self.task in ["red_vs_other", "green_vs_other", "blue_vs_other", "yellow_vs_other",
                           "red_or_yellow_vs_other", "blue_or_green_vs_other"]:
            self.start_new_differentiation_round()
        elif self.task in ["three_squares", "three_cars", "three_objects"]:
            self.start_new_generalisation_round()

    def start_new_simple_round(self):
        self.color = self.task
        self.color_image = f"{self.task}-square"

        self.activity_parameters.images = [self.color_image]
        self.activity_parameters.correct_image = [self.color_image]
        
        look_at_tablet = random.choice(self.initial_phrase)
        self.say_text(f"{look_at_tablet} Tippe auf {self.en_to_de_color_map[self.color]}!")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_simple_round] Publishing task parameters " +\
                          f"-- color: {self.color}, image: {self.color_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)

    def start_new_differentiation_round(self):
        # differentiation tasks are expected to have names of the form
        # [color]_vs_other or [color1]_or_[color2]_vs_other
        if "or" in self.task:
            tested_colors = self.task[0:self.task.find("vs")-1].split("_or_")
            self.color = random.choice(tested_colors)
        else:
            self.color = self.task.split('_')[0]
        self.color_image = f"{self.color}-square"

        distractor_color = random.choice(self.distractor_colors)
        distractor_image = f"{distractor_color}-square"

        self.activity_parameters.correct_image = [self.color_image]
        self.activity_parameters.correct_image_highlighted = [f"{self.color_image}-highlighted"]

        # we randomise the position of the correct image
        if random.random() < 0.5:
            self.activity_parameters.images = [self.color_image, distractor_image]
        else:
            self.activity_parameters.images = [distractor_image, self.color_image]
        
        look_at_tablet = random.choice(self.initial_phrase)
        self.say_text(f"{look_at_tablet} Tippe auf {self.en_to_de_color_map[self.color]}!")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_differentiation_round] Publishing task parameters " +\
                          f"-- color: {self.color}, image: {self.color_image}, " +\
                          f"distractor image: {distractor_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)

    def start_new_generalisation_round(self):
        possible_colors = list(self.target_colors)
        for _ in range(5):
            random.shuffle(possible_colors)
            rospy.sleep(0.05)

        # we only take the first three elements of the list
        possible_colors = possible_colors[0:3]

        self.color = random.choice(possible_colors)
        if "squares" in self.task:
            self.color_image = f"{self.color}-square"
            self.activity_parameters.correct_image = [self.color_image]
            self.activity_parameters.correct_image_highlighted = [f"{self.color_image}-highlighted"]
            self.activity_parameters.images = [f"{x}-square" for x in possible_colors]
        elif "cars" in self.task:
            self.color_image = f"{self.color}-car"
            self.activity_parameters.correct_image = [self.color_image]
            self.activity_parameters.correct_image_highlighted = [f"{self.color_image}-highlighted"]
            self.activity_parameters.images = [f"{x}-car" for x in possible_colors]
        elif "objects" in self.task:
            # we select generalisation images by ensuring that previously used
            # images are not repeated in subsequent rounds
            self.activity_parameters.images = []
            for current_color in possible_colors:
                image_for_color = None
                image_selected = False
                while not image_selected:
                    image_for_color = random.choice(self.generalisation_objects[current_color])
                    image_selected = image_for_color not in self.used_generalisation_objects
                self.activity_parameters.images.append(image_for_color)
                self.used_generalisation_objects.append(image_for_color)

            # we extract the image corresponding to the correct color
            self.color_image = self.activity_parameters.images[possible_colors.index(self.color)]
            self.activity_parameters.correct_image = [self.color_image]
            self.activity_parameters.correct_image_highlighted = [f"{self.color_image}-highlighted"]
        
        look_at_tablet = random.choice(self.initial_phrase)
        self.say_text(f"{look_at_tablet} Tippe auf {self.en_to_de_color_map[self.color]}!")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- color: {self.color}, image: {self.color_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)
        
    def evaluate_answer(self):
        self.possitive_feedback = []
        if self.wrong_answer_count > 0:
            self.possitive_feedback = random.choice(["gut gemacht", "gut"])
        else:
            self.possitive_feedback = random.choice(["Wunderbar", "Klasse", "Spitzenmäßig", "Sehr gut"])

        feedback_emotions = {
            "right": "kiss",
            "right_1": "kiss",
            "right_2": "kiss",
            "wrong": "",
            "wrong_1": "",
            "wrong_2": ""
        }
        right_texts = {
            "red": fr"\emph\ Richtig! \emph\ Rot! \emph\ {self.possitive_feedback}!",
            "red_vs_other": fr"\emph\ Richtig! \emph\ Rot! \emph\ {self.possitive_feedback}!",
            "green": fr" \emph\ Richtig! \emph\ Grün! \emph\ {self.possitive_feedback}!",
            "green_vs_other": fr" \emph\ Richtig! \emph\ Grün! \emph\ {self.possitive_feedback}!",
            "blue": fr" \emph\ Richtig! \emph\ Blau! \emph\ {self.possitive_feedback}!",
            "blue_vs_other": fr" \emph\ Richtig! \emph\ Blau! \emph\ {self.possitive_feedback}!",
            "yellow": fr"\emph\ Richtig! \emph\ Gelb! \emph\ {self.possitive_feedback}!",
            "yellow_vs_other": fr"\emph\ Richtig! \emph\ Gelb! \emph\ {self.possitive_feedback}!",
            "red_or_yellow_vs_other": fr"\emph\ Richtig! \emph\ {self.en_to_de_color_map[self.color]}! \emph\ {self.possitive_feedback}!",
            "blue_or_green_vs_other": fr"\emph\ Richtig! {self.en_to_de_color_map[self.color]}! \emph\ {self.possitive_feedback}!",
            "three_squares": fr"\emph\ Richtig! \emph\ {self.en_to_de_color_map[self.color]}! \emph\ {self.possitive_feedback}!",
            "three_cars": fr"\emph\ Richtig! \emph\ {self.en_to_de_color_map[self.color]}!  \emph\ {self.possitive_feedback}!",
            "three_objects": fr"\emph\ Richtig! \emph\ {self.en_to_de_color_map[self.color]}! \emph\ {self.possitive_feedback}!"
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
            correct_image_idx = self.activity_parameters.images.index(self.activity_parameters.correct_image[0])
            self.activity_parameters.images[correct_image_idx] = self.activity_parameters.correct_image_highlighted[0]
            self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted
        elif self.wrong_answer_count == 2:
            self.activity_parameters.images = self.activity_parameters.correct_image_highlighted
            self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted

        look_at_tablet = random.choice(self.initial_phrase)
        self.say_text(f"{look_at_tablet} Tippe auf {self.en_to_de_color_map[self.color]}!")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- color: {self.color}, image: {self.color_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)