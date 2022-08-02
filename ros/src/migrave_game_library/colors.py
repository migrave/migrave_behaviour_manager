#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import random
import rospy

from migrave_game_library.game_base import GameBase
from migrave_games.msg import TaskParameters

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

        self.distractor_images = self.game_config["game_specific_params"]["distractor_images"]

        self.task_parameters = TaskParameters()
        self.task_parameters_pub = rospy.Publisher("/migrave_game_colors/task_parameters",
                                                   TaskParameters, queue_size=1)
        self.color = "Waiting"
        self.color_image = "Kein"

        self.en_to_de_color_map = {"red": "rot", "green": "grün",
                                   "blue": "blau", "yellow": "gelb"}

    def game_start(self):
        super().game_start()

        rospy.loginfo("Color game starts")
        self.say_text("Heute lernen wir Farben. Fangen wir an!")
        self.show_emotion("showing_smile")
        self.say_text("Hände auf den Tisch. Schau mich an.")
        self.show_emotion("showing_smile")
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
                           "red_vs_other_resume", "green_vs_other_resume",
                           "blue_vs_other_resume", "yellow_vs_other_resume"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round()

    def start_new_round_and_grade(self):
        rospy.loginfo("[start_new_round_and_grade] Publishing task status 'running'")
        self.task_status_pub.publish("running")
        rospy.sleep(2)

        if self.task in ["red", "green", "blue", "yellow",
                         "red_resume", "green_resume", "blue_resume", "yellow_resume"]:
            self.start_new_simple_round()
        elif self.task in ["red_vs_other", "green_vs_other", "blue_vs_other", "yellow_vs_other",
                           "red_vs_other_resume", "green_vs_other_resume",
                           "blue_vs_other_resume", "yellow_vs_other_resume"]:
            self.start_new_differentiation_round()

    def start_new_simple_round(self):
        self.color = self.task
        self.color_image = f"{self.task}-square"

        self.say_text("Schau auf das Tablet!")

        self.task_parameters.emotion = self.color
        self.task_parameters.image_1 = self.color_image

        rospy.loginfo(f"[start_new_simple_round] Publishing task parameters " +\
                      f"-- color: {self.color}, image: {self.color_image}")
        self.task_parameters_pub.publish(self.task_parameters)

        self.say_text(f"Tippe auf {self.en_to_de_color_map[self.color]}!")

    def start_new_differentiation_round(self):
        pass

    def evaluate_answer(self):
        feedback_emotions = {
            "right": "showing_smile",
            "right_1": "showing_smile",
            "right_2": "showing_smile",
            "wrong": "",
            "wrong_1": "",
            "wrong_2": ""
        }
        right_texts = {
            "red": "Rot! Richtig! Wunderbar!",
            "red_vs_other": "Rot! Richtig! Wunderbar!",
            "green": "Grün! Richtig! Wunderbar!",
            "green_vs_other": "Grün! Richtig! Wunderbar!",
            "blue": "Blau! Richtig! Wunderbar!",
            "blue_vs_other": "Blau! Richtig! Wunderbar!",
            "yellow": "Gelb! Richtig! Wunderbar!",
            "yellow_vs_other": "Gelb! Richtig! Wunderbar!"
        }
        feedback_texts = {
            "right": right_texts[self.task],
            "right_1": right_texts[self.task],
            "right_2": right_texts[self.task],
            "wrong": "Lass es uns nochmal probieren!",
            "wrong_1": "Lass es uns nochmal probieren!",
            "wrong_2": "Lass es uns nochmal probieren!"
        }

        super().evaluate_answer(feedback_emotions, feedback_texts)

    def retry_after_wrong(self):
        rospy.loginfo("[retry_after_wrong] Publishing task status 'running'")
        self.task_status_pub.publish("running")
        rospy.sleep(2)
