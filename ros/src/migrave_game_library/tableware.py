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
        self.options_tableware = self.game_config["game_specific_params"]["options"]
        self.target_tableware = self.game_config["game_specific_params"]["target_tableware"]
        self.distractor_objects = self.game_config["game_specific_params"]["distractor_objects"]
        self.generalisation_objects = self.game_config["game_specific_params"]["generalisation_objects"]
        self.kid_action_feedback = self.game_config["media_params"]["answer_feedback_kid"]
        self.object_action_feedback = self.game_config["media_params"]["answer_feedback_object"]
        
        # during a generalisation task, objects should not be repeated in multiple rounds;
        # we thus keep a list of selected objects so that they can be avoided in subsequent rounds
        
        self.used_generalisation_objects = []
        self.activity_parameters = UIActivityParameters()
        self.activity_parameters_pub = rospy.Publisher("/migrave_game_tableware/activity_parameters",
                                                       UIActivityParameters, queue_size=1)
        self.crockery = "Waiting"
        self.crockery_image = "Kein"
        self.initial_phrase = ["Schau auf das Tablet!", "Guck auf das Tablet!", "Schau mal auf das Tablet!", 
                               "Guck mal auf das Tablet!", "Sieh mal auf das Tablet!"]
        self.en_to_de_crockery_map = {"fork": "Gabel", "spoon": "Löffel", "knife": "Messer", "glass": "Becher", "bowl": "Schüssel"}
        self.en_article_crockery_map = {"fork": "die", "spoon": "den", "knife": "das", "glass": "der", "bowl": "die"}
        
    def game_start(self):
        super().game_start()

        rospy.loginfo("Tableware game starts")
        self.say_text("Heute lernen wir verschiedene Gegenstände kennen, die wir zum Essen brauche, Fangen wir an!")
        self.show_emotion("happy")
        self.say_text("Hände auf den Tisch. Schau mich an.")
        self.show_emotion("happy")
        self.say_text("Ich nenne dir ein Wort und du tippst auf das passende Bild.")

    def task_start(self):
        super().task_start()
        if self.task_status == "done":
            return

        if self.task in ["fork", "spoon", "knife", "glass", "bowl", "fork_resume", 
                         "spoon_resume", "knife_resume", "glass_resume", "bowl_resume"]:
            rospy.loginfo(f"Starting simple task '{self.task}'")
            self.start_new_simple_round()
        
        elif self.task in ["fork_vs_objects", "spoon_vs_objects", "knife_vs_objects","glass_vs_objects", 
                           "bowl_vs_objects", "fork_vs_tableware", "spoon_vs_tableware",
                           "knife_vs_tableware", "glass_vs_tableware", "bowl_vs_tableware"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_differentiation_round()
        
        elif self.task in ["object_vs_others", "object_vs_others_resume", "kid_vs_kids", "kid_vs_kids_resume",
                           "object_vs_random", "object_vs_random_resume"]:
            rospy.loginfo(f"Starting differentiation task '{self.task}'")
            self.start_new_generalisation_round()

    def start_new_round_and_grade(self):

        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo("[start_new_round_and_grade] Publishing task status 'running'")
            self.task_status_pub.publish("running")
            rospy.sleep(2)

        if self.task in ["fork", "spoon", "knife", "glass", "bowl", "fork_resume", 
                         "spoon_resume", "knife_resume", "glass_resume", "bowl_resume"]:
            self.start_new_simple_round()
      
        elif self.task in ["fork_vs_objects", "spoon_vs_objects", "knife_vs_objects","glass_vs_objects", 
                           "bowl_vs_objects", "fork_vs_tableware", "spoon_vs_tableware",
                           "knife_vs_tableware", "glass_vs_tableware", "bowl_vs_tableware"]:
            self.start_new_differentiation_round()
      
        elif self.task in ["object_vs_others", "object_vs_others_resume", "kid_vs_kids", "kid_vs_kids_resume",
                           "object_vs_random", "object_vs_random_resume"]:
            self.start_new_generalisation_round()

    def start_new_simple_round(self):
        
        self.crockery = self.task
        self.crockery_image = f"{self.target_tableware[self.task][0]}"
        
        look_at_tablet = random.choice(self.initial_phrase)  
        self.say_text(f"{look_at_tablet} Tippe auf {self.en_article_crockery_map[self.crockery]} {self.en_to_de_crockery_map[self.crockery]}!")
        rospy.sleep(0.5)
        
        self.activity_parameters.images = [self.crockery_image]
        self.activity_parameters.correct_image = [self.crockery_image]
        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_simple_round] Publishing task parameters " +\
                          f"-- crockery: {self.crockery}, image: {self.crockery_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)

    def start_new_differentiation_round(self):

        self.crockery = self.task.split("_")[0]
        self.crockery_image = random.choice(self.target_tableware[self.crockery])

        look_at_tablet = random.choice(self.initial_phrase)
        self.say_text(f"{look_at_tablet} Tippe auf {self.en_article_crockery_map[self.crockery]} {self.en_to_de_crockery_map[self.crockery]}!")
        rospy.sleep(0.5)

        if "objects" in self.task:
            distractors = random.sample(self.distractor_objects, 2)

        elif "tableware" in self.task:
            possible_choices = [f"{v}-{random.randint(1, 5)}" for v in self.options_tableware if v != self.crockery]
            distractors = random.sample(possible_choices, 2)
    
        distractor_image = f"{distractors}"
        self.activity_parameters.correct_image = [self.crockery_image]
        self.activity_parameters.correct_image_highlighted = [f"{self.crockery_image}-highlighted"]

        # we randomise the position of the correct image
        list_of_images = [self.crockery_image, distractors[0], distractors[1]]
        self.activity_parameters.images = random.sample(list_of_images, len(list_of_images))
        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_differentiation_round] Publishing task parameters " +\
                          f"-- crockery: {self.crockery}, image: {self.crockery_image}, " +\
                          f"distractor image: {distractor_image}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)        
       
    def start_new_generalisation_round(self):
        
        self.crockery = random.choice(self.options_tableware)
        if "others" in self.task: 
            self.crockery_image = self.target_tableware[self.crockery][0]
            possible_choices = [f"{v}-1" for v in self.options_tableware if v != self.crockery]
            distractors = random.sample(possible_choices, 2)

        elif "kids" in self.task:
            self.crockery_image = f"{self.crockery}-kid"
            if self.crockery == "spoon":
                possible_choices = [v for v in self.generalisation_objects if (v != self.crockery_image and v != "bowl-kid")]
            elif self.crockery == "bowl":
                possible_choices = [v for v in self.generalisation_objects if (v != self.crockery_image and v != "spoon-kid")]
            else: 
                possible_choices = [v for v in self.generalisation_objects if v != self.crockery_image]
            distractors = random.sample(possible_choices, 2)

        elif "random" in self.task: 
            self.crockery_image = f"{self.crockery}-{random.randint(1, 5)}"
            possible_choices = [f"{v}-{random.randint(1, 5)}" for v in self.options_tableware if v != self.crockery]
            distractors = [random.choice(possible_choices), random.choice(self.distractor_objects)]

        self.activity_parameters.correct_image = [self.crockery_image]
        self.activity_parameters.correct_image_highlighted = [f"{self.crockery_image}-highlighted"]
           
        list_of_images = [self.crockery_image, distractors[0], distractors[1]]            
        self.activity_parameters.images = random.sample(list_of_images, len(list_of_images))
                
        look_at_tablet = random.choice(self.initial_phrase)
        if "kids" in self.task:
            self.say_text(f"{look_at_tablet} {self.kid_action_feedback[self.crockery][0]}!")
        elif "random" in self.task:
            self.say_text(f"{look_at_tablet} {self.object_action_feedback[self.crockery][0]}!")
        else:
            self.say_text(f"{look_at_tablet} Tippe auf {self.en_article_crockery_map[self.crockery]} {self.en_to_de_crockery_map[self.crockery]}!")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- crockery: {self.crockery}, image: {self.crockery_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)

    def evaluate_answer(self):
        self.possitive_feedback = []
        if self.wrong_answer_count > 0:
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
            "fork": fr"\emph\ Richtig! \emph\ die Gabel! \emph\ {self.possitive_feedback}!",
            "fork_vs_objects": fr"\emph\ Richtig! \emph\ die Gabel! \emph\ {self.possitive_feedback}!",
            "fork_vs_tableware": fr"\emph\ Richtig! \emph\ die Gabel! \emph\ {self.possitive_feedback}!",
            
            "spoon": fr"\emph\ Richtig! \emph\ der Löffel! \emph\ Richtig! \emph\ {self.possitive_feedback}!",
            "spoon_vs_objects": fr"\emph\ Richtig! \emph\ der Löffel! \emph\ {self.possitive_feedback}!",
            "spoon_vs_tableware": fr"\emph\ Richtig! \emph\ der Löffel! \emph\ {self.possitive_feedback}!",
            
            "knife": fr"\emph\ Richtig! \emph\ das Messer! \emph\ Richtig! \emph\ {self.possitive_feedback}!",
            "knife_vs_objects": fr"\emph\ Richtig! \emph\ das Messer! \emph\ {self.possitive_feedback}!",
            "knife_vs_tableware": fr"\emph\ Richtig! \emph\ das Messer! \emph\ {self.possitive_feedback}!",
            
            "glass": fr"\emph\ Richtig! \emph\ der Becher! \emph\ {self.possitive_feedback}!",
            "glass_vs_objects": fr"\emph\ Richtig! \emph\ der Becher! \emph\ {self.possitive_feedback}!",
            "glass_vs_tableware": fr"\emph\ Richtig! \emph\ der Becher! \emph\ {self.possitive_feedback}!",
            
            "bowl": fr"\emph\ Richtig! \emph\ die Schüssel! \emph\ {self.possitive_feedback}!",
            "bowl_vs_objects": fr"\emph\ Richtig! \emph\ die Schüssel! \emph\ {self.possitive_feedback}!",
            "bowl_vs_tableware": fr"\emph\ Richtig! \emph\ die Schüssel! \emph\ {self.possitive_feedback}!",
            
            "object_vs_others": fr"\emph\ Richtig! \emph\ {self.en_article_crockery_map[self.crockery]} {self.en_to_de_crockery_map[self.crockery]}! \emph\ {self.possitive_feedback}!",
            "kid_vs_kids": fr"\emph\ Richtig! \emph\ {self.kid_action_feedback[self.crockery][1]}! \emph\ {self.possitive_feedback}!",
            "object_vs_random": fr"\emph\ Richtig! \emph\ {self.object_action_feedback[self.crockery][1]}! \emph\ {self.possitive_feedback}!"
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
            self.activity_parameters.images = [self.activity_parameters.correct_image_highlighted[0]]
            self.activity_parameters.correct_image = self.activity_parameters.correct_image_highlighted

        look_at_tablet = random.choice(self.initial_phrase)
        if "kids" in self.task:
            self.say_text(f"{look_at_tablet} {self.kid_action_feedback[self.crockery][0]}!")
        elif "random" in self.task:
            self.say_text(f"{look_at_tablet} {self.object_action_feedback[self.crockery][0]}!")
        else:
            self.say_text(f"{look_at_tablet} Tippe auf {self.en_article_crockery_map[self.crockery]} {self.en_to_de_crockery_map[self.crockery]}!")

        rospy.sleep(2)
        self.msg_acknowledged = False
        while not self.msg_acknowledged:
            rospy.loginfo(f"[start_new_generalisation_round] Publishing task parameters " +\
                          f"-- crockery: {self.crockery}, image: {self.crockery_image}, " +\
                          f"all images: {self.activity_parameters.images}")
            self.activity_parameters_pub.publish(self.activity_parameters)
            rospy.sleep(0.5)
        rospy.sleep(2)
        
       