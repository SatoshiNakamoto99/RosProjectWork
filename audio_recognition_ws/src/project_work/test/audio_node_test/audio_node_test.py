#!/usr/bin/python3
import os
import json
TEST_PATH = os.path.dirname(os.path.abspath(__file__))

from base_test import BaseTest
from std_msgs.msg import Bool, String
import rospy
from config import *

class AudioNodeTest(BaseTest):
    def __init__(self):
        super().__init__()
        self._text = ""
        self._PEPPER_TALK = False
        self._HUMAN_PRESENCE = False
        self._pub_human_presence = rospy.Publisher(HUMAN_PRESENCE_TOPIC, Bool, queue_size=0)
        self._pub_pepper_talk = rospy.Publisher(PEPPER_TALK_TOPIC, Bool, queue_size=0)
        self._pub_text = rospy.Publisher(CHATBOT_OUTPUT_TOPIC, String, queue_size=0)
    def __read_config(self, file_path):
        try:
            with open(file_path, 'r') as json_file:
                data = json.load(json_file)
                # Imposta le variabili in base ai valori nel file JSON
                self._PEPPER_TALK = data.get("TEST_PEPPER_TALK", False)
                self._HUMAN_PRESENCE = data.get("TEST_HUMAN_PRESENCE", False)
        except FileNotFoundError:
            print(f"File '{file_path}' non trovato. Impostazione di valori predefiniti.")


     
    def __get_text(self):
        return self._text
    
    def __set_text(self, text):
        self._text = text
        
    def __get_text_data(self, text_path):
        
        # Read text from file named text_data.txt
        try:
            with open(os.path.join(text_path,"text_data.txt"),"r") as f:
                text = f.read()
        except:
            print("Error: file text_data.txt not found")
            exit()
        return text
    
    def __test_case(self, test_case_folder):
        print(test_case_folder.upper()+":",end="\t")
        test_case_path = os.path.join(TEST_PATH,test_case_folder)
        # get groundtruth
        self._setup(test_case_path)
 
        # test text
        text_path = test_case_path
        self.__set_text(self.__get_text_data(text_path))
        self.__read_config(os.path.join(text_path, "config_test.json"))
        
        if self._PEPPER_TALK:
            self._pub_pepper_talk.publish(Bool(True))
        else:
            self._pub_pepper_talk.publish(Bool(False))
            
        if self._HUMAN_PRESENCE:
            self._pub_human_presence.publish(Bool(True))
        else:
            self._pub_human_presence.publish(Bool(False))
        self._pub_text.publish(String(self.__get_text()))
        
        
    def __handle_user_input(self, data):
        self._set_output(True)
    
    def start(self):
        rospy.init_node('audio_node_test', anonymous=True)
        rospy.Subscriber(USER_INPUT_TOPIC, String, self.__handle_user_input)

        test_cases = os.listdir(TEST_PATH)
        test_cases.sort()
        for test in test_cases:
            if not os.path.isfile(os.path.join(TEST_PATH,test)):
                self.__test_case(test)
                self._test()
                self._cleanup()
    
if __name__ == "__main__":
    try:
        d = AudioNodeTest()
        d.start()
    except rospy.ROSInterruptException:
        pass