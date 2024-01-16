#!/usr/bin/python3
import os
import json
TEST_PATH = os.path.dirname(os.path.abspath(__file__))
import time
from std_msgs.msg import Bool, String
import rospy
from config import *

class AudioNodeTest:
    def __init__(self):
        super().__init__()
        self._text = ""
        self._PEPPER_TALK = False
        self._HUMAN_PRESENCE = False
        self._output = None
        self._groundtruth = None
        self._pub_human_presence = rospy.Publisher(HUMAN_PRESENCE_TOPIC, Bool, queue_size=0)
        #self._pub_pepper_talk = rospy.Publisher(PEPPER_TALK_TOPIC, Bool, queue_size=0)
        self._pub_output_chatbot = rospy.Publisher(CHATBOT_OUTPUT_TOPIC, String, queue_size=0)
    
    def _setup(self, test_case_path):
        """Load groundtruth from file stored in specific test_case_path, with name groundtruth.txt.
        for each file groundtruth.txt there is a single value True or False.
        Set the value of output to False.
        Args:
            test_case_path (_type_): _description_
        """
        self._output = False
        # check if the file exists
        if not os.path.isfile(os.path.join(test_case_path, "groundtruth.txt")):
            raise FileNotFoundError("File groundtruth.txt not found in {}".format(test_case_path))
        with open(os.path.join(test_case_path, "groundtruth.txt"), "r") as f:
            g = f.read()
        self._groundtruth = g=="True"
    
    def _cleanup(self):
        """Reset groundtruth and detector output"""
        self._groundtruth = None
        self._output = False
    
    def _test(self):
        """Test the correctness of the detector output"""
        # check if the sequece 
        if self._groundtruth == self._output:
            print("Passed")
        else:
            print("Failed")
     
    def __read_config(self, file_path):
        try:
            with open(file_path, 'r') as json_file:
                data = json.load(json_file)
                # Imposta le variabili in base ai valori nel file JSON
                self._PEPPER_TALK = data.get("TEST_PEPPER_TALK", False)
                self._HUMAN_PRESENCE = data.get("TEST_HUMAN_PRESENCE", False)
        except FileNotFoundError:
            print(f"File '{file_path}' non trovato. Impostazione di valori predefiniti.")

    def __get_output_chatbot_from_file(self, file_path, file_name):
        #Leggi il file txt e ritorna il contenuto
        
        file_path = os.path.join(file_path, file_name)
        with open(file_path, 'r') as file:
            output = file.read()
        return output

    
    
    
    def __test_case(self, test_case_folder):
        print(test_case_folder.upper()+":",end="\t")
        test_case_path = os.path.join(TEST_PATH,test_case_folder)
        # get groundtruth
        self._setup(test_case_path)
        #print("Groundtruth: {}".format(self._groundtruth))
        #print("Init Output: {}".format(self._output))
        # test text
        text_path = test_case_path
        self.__read_config(os.path.join(text_path, "config_test.json"))
        
        if self._HUMAN_PRESENCE:
            self._pub_human_presence.publish(True)
        else:
            self._pub_human_presence.publish(False)
        
        if self._PEPPER_TALK:
            #leggi l'output che pepper deve dire dal file txt text_data.txt
            output = self.__get_output_chatbot_from_file(text_path, "text_data.txt")
            #pubblica su CHATBOT_OUTPUT_TOPIC
            self._pub_output_chatbot.publish(output)
            #print("Pepper says: {}".format(output))
        
        time.sleep(5)    
        
        #self._pub_text.publish(String(self.__get_text()))
        print("Output: {}".format(self._output))
        
        self._test()
        self._cleanup()
        
    def __handle_user_input(self, data):
        self._set_output(True)
        print("User input: {}".format(data.data))
    
    def start(self):
        rospy.init_node('audio_node_test', anonymous=True)
        rospy.Subscriber(USER_INPUT_TOPIC, String, self.__handle_user_input)
        test_cases = os.listdir(TEST_PATH)
        test_cases.sort()
        for test in test_cases:
            if test!="__pycache__" and test!="base_test.py" and test!="audio_node_test.py" and test!=".gitignore" and test!=".git":
                if not os.path.isfile(os.path.join(TEST_PATH,test)):
                    self.__test_case(test)
                    
        print("TEST FINISHED")
        print("Type CTRL+C to exit")
        while not rospy.is_shutdown():
            pass
if __name__ == "__main__":
    try:
        d = AudioNodeTest()
        d.start()
    except rospy.ROSInterruptException:
        pass