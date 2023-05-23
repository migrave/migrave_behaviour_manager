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
        self.options_animals = self.game_config["game_specific_params"]["options"]
        self.distractor_objects = self.game_config["game_specific_params"]["distractor_objects"]
        self.distractor_animals = self.game_config["game_specific_params"]["distractor_animals"]
        self.generalisation_objects = self.game_config["game_specific_params"]["generalisation_objects"]
        # during a generalisation task, objects should not be repeated in multiple rounds;
        # we thus keep a list of selected objects so that they can be avoided in subsequent rounds
        
        self.activity_parameters = UIActivityParameters()
        self.activity_parameters_pub = rospy.Publisher("/migrave_game_animals/activity_parameters",
                                                       UIActivityParameters, queue_size=1)
        self.animal = "Waiting"
        self.animal_image = "Kein"
        self.initial_phrase = ["Schau auf das Tablet!", "Guck auf das Tablet!", "Schau mal auf das Tablet!", "Guck mal auf das Tablet!",  "Sieh mal auf das Tablet!"]
        self.en_to_de_animal_map = {"cat": "Katze", "dog": "Hund", "cow": "Kuh", "pig": "Schwein", "mouse": "Maus"}
        self.ak_article_animal_map = {"cat": "die", "dog": "den", "cow": "die", "pig": "das", "mouse": "die"}
        self.no_article_animal_map = {"cat": "die", "dog": "der", "cow": "die", "pig": "das", "mouse": "die"}
        self.monitor_game()

    def game_start(self):
        super().game_start()

        rospy.loginfo("Animal game starts")
        self.say_text("Heute lernen wir Tiere kennen. Fangen wir an!")
        self.show_emotion("showing_smile")
        self.say_text("Hände auf den Tisch. Schau mich an.")
        self.show_emotion("showing_smile")
        self.say_text("Ich nenne dir ein Tier und du tippst auf das passende Bild.")
        
    def task_start(self):
        super().task_start()
        if self.task_status == "done":
            return

        if self.task in ["cat", "dog", "cow", "pig", "mouse"]:
            rospy.loginfo(f"Starting simple task '{self.task}'")
            self.start_new_simple_round()

        elif self.task in ["cat_vs_objects", "dog_vs_objects", "cow_vs_objects", "pig_vs_objects", "mouse_vs_objects"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round('objects')

        elif self.task in ["cat_vs_animals", "dog_vs_animals","cow_vs_animals", "cow_vs_animals", "pig_vs_animals", "mouse_vs_animals"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round('animals')

        elif self.task in ["animal_vs_objects", "animal_vs_others", "animal_vs_animals", "animal_vs_objects_resume", 
                           "animal_vs_others_resume", "animal_vs_animals_resume"]:     
            rospy.loginfo(f"Starting generalisation task '{self.task}'")
            self.start_new_generalisation_round()
        self.reset_coping_reactions()

    def start_new_round_and_grade(self):
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo("[start_new_round_and_grade] Publishing task status 'running'")
            self.task_status_pub.publish("running")
            rospy.sleep(2)
            
        if self.task in ["cat", "dog", "cow", "pig", "mouse"]:
            self.start_new_simple_round()

        elif self.task in ["cat_vs_objects", "dog_vs_objects", "cow_vs_objects", "pig_vs_objects", "mouse_vs_objects"]:
            self.start_new_differentiation_round('objects')

        elif self.task in ["cat_vs_animals", "dog_vs_animals", "cow_vs_animals", "pig_vs_animals", "mouse_vs_animals"]:
            self.start_new_differentiation_round('animals')

        elif self.task in ["animal_vs_objects", "animal_vs_others", "animal_vs_animals", "animal_vs_objects_resume", 
                           "animal_vs_others_resume", "animal_vs_animals_resume"]:
            self.start_new_generalisation_round()

    def start_new_simple_round(self):

        self.animal = self.task
        self.animal_image = f"{self.target_animals[self.task][0]}" 

        self.activity_parameters.images = [self.animal_image]
        self.activity_parameters.correct_image = [self.animal_image]

        if self.round_count == 0:
            look_at_tablet = random.choice(self.initial_phrase)
            self.say_text(f"{look_at_tablet} Tippe auf {self.ak_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!")
        else:
            self.say_text(f"Tippe auf {self.ak_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_simple_round] Publishing task parameters " +\
                          f"-- animal: {self.animal}, image: {self.animal_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)

    def start_new_differentiation_round(self, type_of_differentiation):
        self.animal = self.task.split("_")[0]
        if type_of_differentiation == 'objects':
            self.animal_image = random.choice(self.target_animals[self.animal])
            distractors = random.sample(self.distractor_objects, 2)
        elif type_of_differentiation == 'animals':
            self.animal_image = random.choice(self.target_animals[self.animal] + self.generalisation_objects[self.animal])
            distractors = random.sample(self.distractor_animals, 2)

        distractor_image = f"{distractors}"
        self.activity_parameters.correct_image = [self.animal_image]
        self.activity_parameters.correct_image_highlighted = [f"{self.animal_image}-highlighted"]
        
        # we randomise the position of the correct image
        list_of_images = [self.animal_image, distractors[0], distractors[1]]
        self.activity_parameters.images = random.sample(list_of_images, len(list_of_images))

        if self.round_count == 0:
            look_at_tablet = random.choice(self.initial_phrase)
            self.say_text(f"{look_at_tablet} Tippe auf {self.ak_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!")
        else:
            self.say_text(f"Tippe auf {self.ak_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_differentiation_round] Publishing task parameters " +\
                          f"-- animal: {self.animal}, image: {self.animal_image}, " +\
                          f"distractor image: {distractor_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)

    def start_new_generalisation_round(self):
        self.animal = random.choice(self.options_animals)
        if "objects" in self.task: 
            self.animal_image = self.target_animals[self.animal][0]
            distractors = random.sample(self.distractor_objects, 2)
        elif "others" in self.task:
            self.animal_image = self.target_animals[self.animal][0]
            possible_choices = [f"{v}-1" for v in self.options_animals if v != self.animal]
            distractors = random.sample(possible_choices, 2)
        elif "animals" in self.task:
            possible_animals = list(self.target_animals[self.animal] + self.generalisation_objects[self.animal])
            self.animal_image = random.choice(possible_animals)
            distractors = random.sample(self.distractor_animals, 2)

        self.activity_parameters.correct_image = [self.animal_image]
        self.activity_parameters.correct_image_highlighted = [f"{self.animal_image}-highlighted"]
        #geralisation with random images (no animals)
        list_of_images = [self.animal_image, distractors[0], distractors[1]]            
        self.activity_parameters.images = random.sample(list_of_images, len(list_of_images))

        if self.round_count == 0:
            look_at_tablet = random.choice(self.initial_phrase)
            self.say_text(f"{look_at_tablet} Tippe auf {self.ak_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!")
        else:
            self.say_text(f"Tippe auf {self.ak_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- animal: {self.animal}, image: {self.animal_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)

    def evaluate_answer(self):
        if self.wrong_answer_count > 0 or self.has_performed_coping():
            self.possitive_feedback = random.choice(["Gut gemacht", "Gut", "Prima"])
        else:
            self.possitive_feedback = random.choice(["Wunderbar", "Klasse", "Spitzenmäßig", "Sehr gut", "Toll", "Super"])

        feedback_emotions = {
            "right": "kiss",
            "right_1": "kiss",
            "right_2": "kiss",
            "wrong": "",
            "wrong_1": "",
            "wrong_2": ""
        }
        right_texts = {
            "cat": fr"\emph\ {self.possitive_feedback}! \emph\ die Katze!",
            "cat_vs_objects": fr"\emph\ {self.possitive_feedback}! \emph\ die Katze!",
            "cat_vs_animals": fr"\emph\ {self.possitive_feedback}! \emph\ die Katze!",

            "dog": fr"\emph\ {self.possitive_feedback}! \emph\ der Hund!",
            "dog_vs_objects": fr"\emph\ {self.possitive_feedback}! \emph\ der Hund!",
            "dog_vs_animals": fr"\emph\ {self.possitive_feedback}! \emph\ der Hund!",

            "cow": fr"\emph\ {self.possitive_feedback}! \emph\ die Kuh!",
            "cow_vs_objects": fr"\emph\ {self.possitive_feedback}! \emph\ die Kuh!",
            "cow_vs_animals": fr"\emph\ {self.possitive_feedback}! \emph\ die Kuh!",

            "pig": fr"\emph\ {self.possitive_feedback}! \emph\ das Schwein!",
            "pig_vs_objects": fr"\emph\ {self.possitive_feedback}! \emph\ das Schwein!",
            "pig_vs_animals": fr"\emph\ {self.possitive_feedback}! \emph\ das Schwein!",

            "mouse": fr"\emph\ {self.possitive_feedback}! \emph\ die Maus!",
            "mouse_vs_objects": fr"\emph\ {self.possitive_feedback}! \emph\ die Maus!",
            "mouse_vs_animals": fr"\emph\ {self.possitive_feedback}! \emph\ die Maus!",
            
            "animal_vs_objects": fr"\emph\ {self.possitive_feedback}! \emph\ {self.no_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!",
            "animal_vs_others": fr"\emph\ {self.possitive_feedback}! \emph\ {self.no_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!",
            "animal_vs_animals": fr"\emph\ {self.possitive_feedback}! \emph\ {self.no_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!"
        }

        right_audios = {
            "cat": "aleksandar.mitrevski/animals/Katze",
            "cat_vs_objects": "aleksandar.mitrevski/animals/Katze",
            "cat_vs_animals": "aleksandar.mitrevski/animals/Katze",
            
            "dog": "aleksandar.mitrevski/animals/Hund",
            "dog_vs_objects": "aleksandar.mitrevski/animals/Hund",
            "dog_vs_animals": "aleksandar.mitrevski/animals/Hund",
            
            "cow": "aleksandar.mitrevski/animals/Kuh",
            "cow_vs_objects": "aleksandar.mitrevski/animals/Kuh",
            "cow_vs_animals": "aleksandar.mitrevski/animals/Kuh",
            
            "pig": "aleksandar.mitrevski/animals/Schwein",
            "pig_vs_objects": "aleksandar.mitrevski/animals/Schwein",
            "pig_vs_animals": "aleksandar.mitrevski/animals/Schwein",

            "mouse": "aleksandar.mitrevski/animals/Maus",
            "mouse_vs_objects": "aleksandar.mitrevski/animals/Maus",
            "mouse_vs_animals": "aleksandar.mitrevski/animals/Maus",

            "animal_vs_objects": "aleksandar.mitrevski/animals/"+ str(self.en_to_de_animal_map[self.animal]),
            "animal_vs_others": "aleksandar.mitrevski/animals/"+ str(self.en_to_de_animal_map[self.animal]),
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
            self.activity_parameters.images = self.activity_parameters.correct_image_highlighted
            self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted

        self.say_text(f"Tippe auf {self.ak_article_animal_map[self.animal]} {self.en_to_de_animal_map[self.animal]}!")
        rospy.sleep(2)

        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- animal: {self.animal}, image: {self.animal_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)