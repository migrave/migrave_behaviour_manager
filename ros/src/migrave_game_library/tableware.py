#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import random
import rospy

from migrave_game_library.game_base import GameBase
from migrave_ros_msgs.msg import UIActivityParameters

class MigraveGameTableware(GameBase):
    def __init__(self, game_config_dir_path: str,
                 game_id: str,
                 game_status_topic: str = "/migrave_game_tableware/status",
                 game_answer_topic: str = "/migrave_game_tableware/answer",
                 game_performance_topic: str = "/migrave/game_performance"):
        super(MigraveGameTableware, self).__init__(game_config_dir_path,
                                                game_id,
                                                game_status_topic,
                                                game_answer_topic,
                                                game_performance_topic)

        self.target_tableware = self.game_config["game_specific_params"]["target_tableware"]
        self.distractor_tableware = self.game_config["game_specific_params"]["distractor_tableware"]
        self.distractor_objects = self.game_config["game_specific_params"]["distractor_objects"]
        self.distractor_child = self.game_config["game_specific_params"]["distractor_child"]
        self.generalisation_objects = self.game_config["game_specific_params"]["generalisation_objects"]
        # during a generalisation task, objects should not be repeated in multiple rounds;
        # we thus keep a list of selected objects so that they can be avoided in subsequent rounds
        
        self.used_generalisation_objects = []
        self.activity_parameters = UIActivityParameters()
        self.activity_parameters_pub = rospy.Publisher("/migrave_game_tableware/activity_parameters",
                                                       UIActivityParameters, queue_size=1)
        self.crockery = "Waiting"
        self.crockery_image = "Kein"

        self.en_to_de_crockery_map = {"fork": "Gabel", "teacup": "Tasse",
                                   "spoon": "Löffel", "plate": "Teller"}
        self.en_article_crockery_map = {"fork": "die", "teacup": "die",
                                   "spoon": "den", "plate": "den"}

    def game_start(self):
        super().game_start()

        rospy.loginfo("Tableware game starts")
        self.say_text("Heute lernen wir etwas über Geschirr. Fangen wir an!")
        self.show_emotion("showing_smile")
        self.say_text("Hände auf den Tisch. Schau mich an.")
        self.show_emotion("showing_smile")
        self.say_text("Ich nenne dir ein Gerät und du tippst auf das passende Bild.")

    def task_start(self):
        super().task_start()
        if self.task_status == "done":
            return

        if self.task in ["fork", "teacup", "spoon", "plate",
                         "fork_resume", "teacup_resume", "spoon_resume", "plate_resume"]:
            rospy.loginfo(f"Starting simple task '{self.task}'")
            self.start_new_simple_round()
        
        elif self.task in ["fork_vs_objects", "teacup_vs_objects", "plate_vs_objects", "spoon_vs_objects"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round('objects')
        
        elif self.task in ["fork_vs_tableware", "teacup_vs_tableware", "spoon_vs_tableware", "plate_vs_tableware"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round('tableware')
        
        elif self.task in ["fork_vs_random", "teacup_vs_random", "spoon_vs_random", "plate_vs_random"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round('random')
      
        elif self.task in ["fork_kid_vs_random", "teacup_kid_vs_random", "spoon_kid_vs_random", "plate_kid_vs_random",
                           "fork_kid_vs_random_resume", "teacup_kid_vs_random_resume", "spoon_kid_vs_random_resume", "plate_kid_vs_random_resume"]:
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

        if self.task in ["fork", "teacup", "spoon", "plate",
                         "fork_resume", "teacup_resume", "spoon_resume", "plate_resume"]:
            self.start_new_simple_round()
      
        elif self.task in ["fork_vs_objects", "teacup_vs_objects", "plate_vs_objects", "spoon_vs_objects"]:
            self.start_new_differentiation_round('objects')
      
        elif self.task in ["fork_vs_tableware", "teacup_vs_tableware", "spoon_vs_tableware", "plate_vs_tableware"]:
            self.start_new_differentiation_round('tableware')
      
        elif self.task in ["fork_vs_random", "teacup_vs_random", "spoon_vs_random", "plate_vs_random"]:
            self.start_new_differentiation_round('random')    
      
        elif self.task in ["fork_kid_vs_random", "teacup_kid_vs_random", "spoon_kid_vs_random", "plate_kid_vs_random", 
                           "fork_kid_vs_random_resume", "teacup_kid_vs_random_resume", "spoon_kid_vs_random_resume", "plate_kid_vs_random_resume"]:
            self.start_new_generalisation_round()

    def start_new_simple_round(self):
        self.crockery = self.task
        self.crockery_image = f"{self.task}"

        self.activity_parameters.images = [self.crockery_image]
        self.activity_parameters.correct_image = self.crockery_image

        self.say_text("Schau auf das Tablet!")
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_simple_round] Publishing task parameters " +\
                          f"-- crockery: {self.crockery}, image: {self.crockery_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        self.say_text(f"Tippe auf {self.en_article_crockery_map[self.crockery]} {self.en_to_de_crockery_map[self.crockery]}!")

    def start_new_differentiation_round(self, type_of_differentiation):
        # differentiation tasks are expected to have names of the form
        # [tableware]_vs_other or [tableware1]_or_[tableware2]_vs_other
        if "or" in self.task:
            tested_tableware = self.task[0:self.task.find("vs")-1].split("_or_")
            self.crockery = random.choice(tested_tableware)
        else:
            self.crockery = self.task.split('_')[0]
        self.crockery_image = f"{self.crockery}"

        if type_of_differentiation == 'objects':
            distractors = random.sample(self.distractor_objects, 2)
        if type_of_differentiation == 'tableware':
            distractors = random.sample(self.distractor_tableware, 2)
        if type_of_differentiation == 'random':
            distractors = random.sample(self.distractor_tableware + self.distractor_objects, 2)

        distractor_image = f"{distractors}"
        self.activity_parameters.correct_image = self.crockery_image
        self.activity_parameters.correct_image_highlighted = f"{self.crockery_image}-highlighted"

        # we randomise the position of the correct image
        list_of_images = [self.crockery_image, distractors[0], distractors[1]]
        self.activity_parameters.images = random.sample(list_of_images, len(list_of_images))

        self.say_text("Schau auf das Tablet!")
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_differentiation_round] Publishing task parameters " +\
                          f"-- crockery: {self.crockery}, image: {self.crockery_image}, " +\
                          f"distractor image: {distractor_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        self.say_text(f"Tippe auf {self.en_article_crockery_map[self.crockery]} {self.en_to_de_crockery_map[self.crockery]}!")

    def start_new_generalisation_round(self):
        # possible_tableware = list(self.generalisation_objects)
        # print("possible_tableware: ", possible_tableware)
        # for _ in range(5):
        #     random.shuffle(possible_tableware)
        #     rospy.sleep(0.05)

        # # we only take the first three elements of the list
        # possible_tableware = possible_tableware[0:3]

        # self.crockery = random.choice(possible_tableware)
        self.crockery = self.task[0:self.game_status.find('_kid_vs_random')]
        self.crockery_image = f"child-{self.crockery}"
        self.activity_parameters.correct_image = self.crockery_image
        self.activity_parameters.correct_image_highlighted = f"{self.crockery_image}-highlighted"
            
        # we select generalisation images by ensuring that previously used
        # images are not repeated in subsequent rounds
        distractors = random.sample(self.distractor_child, 2)
        list_of_images = [self.crockery_image, distractors[0], distractors[1]]            
        self.activity_parameters.images = random.sample(list_of_images, len(list_of_images))
        
        # self.activity_parameters.images = []
        # for current_tableware in possible_tableware:
        #     image_for_tableware = None
        #     image_selected = False
        #     while not image_selected:
        #         image_for_tableware = random.choice(self.generalisation_objects[current_tableware])
        #         image_selected = image_for_tableware not in self.used_generalisation_objects
        #     self.activity_parameters.images.append(image_for_tableware)
        #     self.used_generalisation_objects.append(image_for_tableware)

        # # we extract the image corresponding to the correct tableware
        # self.tableware_image = self.activity_parameters.images[possible_tableware.index(self.tableware)]
        # self.activity_parameters.correct_image = self.tableware_image
        # self.activity_parameters.correct_image_highlighted = f"{self.tableware_image}-highlighted"

        self.say_text("Schau auf das Tablet!")
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- crockery: {self.crockery}, image: {self.crockery_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        self.say_text(f"Tippe auf {self.en_article_crockery_map[self.crockery]} {self.en_to_de_crockery_map[self.crockery]}!")

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
            "fork": r"\emph\ die Gabel! \emph\ Richtig! \emph\ Wunderbar!",
            "fork_vs_objects": r"\emph\ die Gabel! \emph\ Richtig! \emph\ Wunderbar!",
            "fork_vs_tableware": r"\emph\ die Gabel! \emph\ Richtig! \emph\ Wunderbar!",
            "fork_vs_random": r"\emph\ die Gabel! \emph\ Richtig! \emph\ Wunderbar!",
            "fork_kid_vs_random": r"\emph\ die Gabel! \emph\ Richtig! \emph\ Wunderbar!",
            "teacup": r"\emph\ die Tasse! \emph\ Richtig! \emph\ Wunderbar!",
            "teacup_vs_objects": r"\emph\ die Tasse! \emph\ Richtig! \emph\ Wunderbar!",
            "teacup_vs_tableware": r"\emph\ die Tasse! \emph\ Richtig! \emph\ Wunderbar!",
            "teacup_vs_random": r"\emph\ die Tasse! \emph\ Richtig! \emph\ Wunderbar!",
            "teacup_kid_vs_random": r"\emph\ die Tasse! \emph\ Richtig! \emph\ Wunderbar!",
            "spoon": r"\emph\ der Löffel! \emph\ Richtig! \emph\ Wunderbar!",
            "spoon_vs_objects": r"\emph\ der Löffel! \emph\ Richtig! \emph\ Wunderbar!",
            "spoon_vs_tableware": r"\emph\ der Löffel! \emph\ Richtig! \emph\ Wunderbar!",
            "spoon_vs_random": r"\emph\ der Löffel! \emph\ Richtig! \emph\ Wunderbar!",
            "spoon_kid_vs_random": r"\emph\ der Löffel! \emph\ Richtig! \emph\ Wunderbar!",
            "plate": r"\emph\ der Teller! \emph\ Richtig! \emph\ Wunderbar!",
            "plate_vs_objects": r"\emph\ der Teller! \emph\ Richtig! \emph\ Wunderbar!",
            "plate_vs_tableware": r"\emph\ der Teller! \emph\ Richtig! \emph\ Wunderbar!",
            "plate_vs_random": r"\emph\ der Teller! \emph\ Richtig! \emph\ Wunderbar!",
            "plate_kid_vs_random": r"\emph\ der Teller! \emph\ Richtig! \emph\ Wunderbar!"
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
            correct_image_idx = self.activity_parameters.images.index(self.activity_parameters.correct_image)
            self.activity_parameters.images[correct_image_idx] = self.activity_parameters.correct_image_highlighted
            self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted
        elif self.wrong_answer_count == 2:
            self.activity_parameters.images = [self.activity_parameters.correct_image_highlighted]
            self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted

        self.say_text("Schau auf das Tablet!")
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- crockery: {self.crockery}, image: {self.crockery_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        self.say_text(f"Tippe auf {self.en_article_crockery_map[self.crockery]} {self.en_to_de_crockery_map[self.crockery]}!")
