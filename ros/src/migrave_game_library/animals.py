#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import random
import rospy

from migrave_game_library.game_base import GameBase
from migrave_ros_msgs.msg import UIActivityParameters

class MigraveGameAnimals(GameBase):
    def __init__(self, game_config_dir_path: str,
                 game_id: str,
                 game_status_topic: str = "/migrave_game_animals/status",
                 game_answer_topic: str = "/migrave_game_animals/answer",
                 game_performance_topic: str = "/migrave/game_performance"):
        super(MigraveGameAnimals, self).__init__(game_config_dir_path,
                                                game_id,
                                                game_status_topic,
                                                game_answer_topic,
                                                game_performance_topic)

        self.target_animals = self.game_config["game_specific_params"]["target_animals"]
        self.distractor_objects = self.game_config["game_specific_params"]["distractor_objects"]
        self.distractor_animals = self.game_config["game_specific_params"]["distractor_animals"]
        self.generalisation_objects = self.game_config["game_specific_params"]["generalisation_objects"]
        # during a generalisation task, objects should not be repeated in multiple rounds;
        # we thus keep a list of selected objects so that they can be avoided in subsequent rounds
        
        self.used_generalisation_objects = []
        self.activity_parameters = UIActivityParameters()
        self.activity_parameters_pub = rospy.Publisher("/migrave_game_animals/activity_parameters",
                                                       UIActivityParameters, queue_size=1)
        self.animal = "Waiting"
        self.animal_image = "Kein"
        
        self.initial_phrase = ["Schau auf das Tablet!", "Guck auf das Tablet!", "Schau mal auf das Tablet!", "Guck mal auf das Tablet!",  "Sieh mal auf das Tablet!"]
        
        self.en_to_de_animal_map = {"cat": "Katze", "dog": "Hund",
                                   "cow": "Kuh", "horse": "Pferd"}
        
        self.ak_article_animal_map = {"cat": "die", "dog": "den",
                                   "cow": "die", "horse": "das"}
        
        self.no_article_animal_map = {"cat": "die", "dog": "der",
                                   "cow": "die", "horse": "das"}

    def game_start(self):
        super().game_start()

        rospy.loginfo("Animal game starts")
        self.say_text("Heute werden wir etwas über Tiere lernen. Fangen wir an!")
        self.show_emotion("showing_smile")
        self.say_text("Hände auf den Tisch. Schau mich an.")
        self.show_emotion("showing_smile")
        self.say_text("Ich nenne dir ein Tier und du tippst auf das passende Bild.")
        
    def task_start(self):
        super().task_start()
        if self.task_status == "done":
            return

        if self.task in ["cat", "dog", "cow", "horse",
                         "cat_resume", "dog_resume", "cow_resume", "horse_resume"]:
            rospy.loginfo(f"Starting simple task '{self.task}'")
            self.start_new_simple_round()

        elif self.task in ["cat_vs_objects", "dog_vs_objects", "cow_vs_objects", "horse_vs_objects"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round('objects')

        elif self.task in ["cat_vs_animals", "dog_vs_animals","cow_vs_animals", "cow_vs_animals", "horse_vs_animals"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round('animals')

        elif self.task in ["cartoon_vs_objects", "black-white_vs_objects", "draw_vs_objects", "animal_vs_animals",
                           "cartoon_vs_objects_resume", "black-white_vs_objects_resume", "draw_vs_objects_resume", "animal_vs_animals_resume"]:       
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

        if self.task in ["cat", "dog", "cow", "horse",
                         "cat_resume", "dog_resume", "cow_resume", "horse_resume"]:
            self.start_new_simple_round()

        elif self.task in ["cat_vs_objects", "dog_vs_objects", "cow_vs_objects", "horse_vs_objects"]:
            self.start_new_differentiation_round('objects')

        elif self.task in ["cat_vs_animals", "dog_vs_animals", "cow_vs_animals", "horse_vs_animals"]:
            self.start_new_differentiation_round('animals')

        elif self.task in ["cartoon_vs_objects", "black-white_vs_objects", "draw_vs_objects", "animal_vs_animals",
                           "cartoon_vs_objects_resume", "black-white_vs_objects_resume", "draw_vs_objects_resume", "animal_vs_animals_resume"]:
            self.start_new_generalisation_round()

    def start_new_simple_round(self):
        self.animal = self.task
        self.animal_image = f"{self.task}" 

        self.activity_parameters.images = [self.animal_image]
        self.activity_parameters.correct_image = [self.animal_image]

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_simple_round] Publishing task parameters " +\
                          f"-- animal: {self.animal}, image: {self.animal_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)

        look_at_tablet = random.choice(self.initial_phrase)
        self.say_text(f"{look_at_tablet} Tippe auf {self.ak_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!")

    def start_new_differentiation_round(self, type_of_differentiation):
        if "or" in self.task:
            tested_animals = self.task[0:self.task.find("vs")-1].split("_or_")
            self.animal = random.choice(tested_animals)
        else:
            self.animal = self.task.split('_')[0]
        self.animal_image = f"{self.animal}"

        if type_of_differentiation == 'objects':
            distractors = random.sample(self.distractor_objects, 2)
        if type_of_differentiation == 'animals':
            distractors = random.sample(self.distractor_animals, 2)

        distractor_image = f"{distractors}"
        self.activity_parameters.correct_image = [self.animal_image]
        self.activity_parameters.correct_image_highlighted = f"{self.animal_image}-highlighted"
        
        # we randomise the position of the correct image
        list_of_images = [self.animal_image, distractors[0], distractors[1]]
        self.activity_parameters.images = random.sample(list_of_images, len(list_of_images))

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_differentiation_round] Publishing task parameters " +\
                          f"-- animal: {self.animal}, image: {self.animal_image}, " +\
                          f"distractor image: {distractor_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)

        look_at_tablet = random.choice(self.initial_phrase)
        self.say_text(f"{look_at_tablet} Tippe auf {self.ak_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!")

    def start_new_generalisation_round(self):
        possible_animals = list(self.target_animals)
        for _ in range(5):
            random.shuffle(possible_animals)
            rospy.sleep(0.05)

        # we only take the first three elements of the list
        possible_animals = possible_animals[0:3]

        self.animal = random.choice(possible_animals)
        if "cartoon" in self.task:
            self.animal_image = f"{self.animal}-cartoon"
            self.activity_parameters.correct_image = [self.animal_image]
            self.activity_parameters.correct_image_highlighted = [f"{self.animal_image}-highlighted"]
        elif "black-white" in self.task:
            self.animal_image = f"{self.animal}-black-white"
            self.activity_parameters.correct_image = [self.animal_image]
            self.activity_parameters.correct_image_highlighted = [f"{self.animal_image}-highlighted"]
        elif "draw" in self.task:
            self.animal_image = f"{self.animal}-draw"
            self.activity_parameters.correct_image = [self.animal_image]
            self.activity_parameters.correct_image_highlighted = [f"{self.animal_image}-highlighted"]
        
        #geralisation with random images (no animals)
        distractors = random.sample(self.distractor_objects, 2)
        list_of_images = [self.animal_image, distractors[0], distractors[1]]            
        self.activity_parameters.images = random.sample(list_of_images, len(list_of_images))

        if "animals" in self.task:
            # we select generalisation images by ensuring that previously used
            # images are not repeated in subsequent rounds
            self.activity_parameters.images = []
            for current_animal in possible_animals:
                image_for_animal = None
                image_selected = False
                while not image_selected:
                    image_for_animal = random.choice(self.generalisation_objects[current_animal])
                    image_selected = image_for_animal not in self.used_generalisation_objects
                self.activity_parameters.images.append(image_for_animal)
                self.used_generalisation_objects.append(image_for_animal)

            # we extract the image corresponding to the correct animal
            self.animal_image = self.activity_parameters.images[possible_animals.index(self.animal)]
            self.activity_parameters.correct_image = [self.animal_image]
            self.activity_parameters.correct_image_highlighted = [f"{self.animal_image}-highlighted"]
        
        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- animal: {self.animal}, image: {self.animal_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)
        
        look_at_tablet = random.choice(self.initial_phrase)
        self.say_text(f"{look_at_tablet} Tippe auf {self.ak_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!")

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
            "cat": fr"\emph\ Richtig! \emph\ die Katze! \emph\ {self.possitive_feedback}!",
            "cat_vs_objects": fr"\emph\ Richtig! \emph\ die Katze! \emph\ {self.possitive_feedback}!",
            "cat_vs_animals": fr"\emph\ Richtig! \emph\ die Katze! \emph\ {self.possitive_feedback}!",
            "cat_vs_others": fr"\emph\ Richtig! \emph\ die Katze! \emph\ {self.possitive_feedback}!",

            "dog": fr"\emph\ Richtig!\emph\ der Hund! \emph\ {self.possitive_feedback}!",
            "dog_vs_objects": fr"\emph\ Richtig! \emph\ der Hund! \emph\ {self.possitive_feedback}!",
            "dog_vs_animals": fr"\emph\ Richtig! \emph\ der Hund! \emph\ {self.possitive_feedback}!",
            "dog_vs_others": fr"\emph\ Richtig! \emph\ der Hund! \emph\ {self.possitive_feedback}!",

            "cow": fr"\emph\ Richtig! \emph\ die Kuh! \emph\ {self.possitive_feedback}!",
            "cow_vs_objects": fr"\emph\ Richtig! \emph\ die Kuh! \emph\ {self.possitive_feedback}!",
            "cow_vs_animals": fr"\emph\ Richtig! \emph\ die Kuh! \emph\ {self.possitive_feedback}!",
            "cow_vs_others": fr"\emph\ Richtig! \emph\ die Kuh! \emph\ {self.possitive_feedback}!",

            "horse": fr"\emph\ Richtig! \emph\ das Pferd! \emph\ {self.possitive_feedback}!",
            "horse_vs_objects": fr"\emph\ Richtig! \emph\ das Pferd! \emph\ {self.possitive_feedback}!",
            "horse_vs_animals": fr"\emph\ Richtig! \emph\ das Pferd! \emph\ {self.possitive_feedback}!",
            "horse_vs_others": fr"\emph\ Richtig! \emph\ das Pferd! \emph\ {self.possitive_feedback}!",

            "cartoon_vs_objects": fr"\emph\ Richtig! \emph\ {self.no_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}! \emph\ {self.possitive_feedback}!",
            "black-white_vs_objects": fr"\emph\ Richtig! \emph\ {self.no_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}! \emph\ {self.possitive_feedback}!",
            "draw_vs_objects": fr"\emph\ Richtig! \emph\ {self.no_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}! \emph\ {self.possitive_feedback}!",
            "animal_vs_animals": fr"\emph\ Richtig! \emph\ {self.no_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}! \emph\ {self.possitive_feedback}!"
        }

        right_audios = {
            "cat": "aleksandar.mitrevski/animals/Katze",
            "cat_vs_objects": "aleksandar.mitrevski/animals/Katze",
            "cat_vs_animals": "aleksandar.mitrevski/animals/Katze",
            "cat_vs_others": "aleksandar.mitrevski/animals/Katze",
            
            "dog": "aleksandar.mitrevski/animals/Hund",
            "dog_vs_objects": "aleksandar.mitrevski/animals/Hund",
            "dog_vs_animals": "aleksandar.mitrevski/animals/Hund",
            "dog_vs_others": "aleksandar.mitrevski/animals/Hund",
            
            "cow": "aleksandar.mitrevski/animals/Kuh",
            "cow_vs_objects": "aleksandar.mitrevski/animals/Kuh",
            "cow_vs_animals": "aleksandar.mitrevski/animals/Kuh",
            "cow_vs_others": "aleksandar.mitrevski/animals/Kuh",
            
            "horse": "aleksandar.mitrevski/animals/Pferd",
            "horse_vs_objects": "aleksandar.mitrevski/animals/Pferd",
            "horse_vs_animals": "aleksandar.mitrevski/animals/Pferd",
            "horse_vs_others": "aleksandar.mitrevski/animals/Pferd",

            "cartoon_vs_objects": "aleksandar.mitrevski/animals/"+ str(self.en_to_de_animal_map[self.animal]),
            "black-white_vs_objects": "aleksandar.mitrevski/animals/"+ str(self.en_to_de_animal_map[self.animal]),
            "draw_vs_objects": "aleksandar.mitrevski/animals/"+ str(self.en_to_de_animal_map[self.animal]),
            "animal_vs_animals": "aleksandar.mitrevski/animals/"+ str(self.en_to_de_animal_map[self.animal])
        }
        feedback_texts = {
            "right": right_texts[self.task],
            "wrong": "Lass es uns nochmal probieren!"
        }
        feedback_sounds = {
            "right": right_audios[self.task]
        }

        super().evaluate_answer(feedback_emotions, feedback_texts, feedback_sounds)

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
            self.activity_parameters.images = [self.activity_parameters.correct_image_highlighted]
            self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted

        look_at_tablet = random.choice(self.initial_phrase)
        self.say_text(f"{look_at_tablet}")

        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- animal: {self.animal}, image: {self.animal_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        self.say_text(f"Tippe auf {self.ak_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!")
