#!/usr/bin/python3
import os
from config import *
import rospy
import time
import numpy as np
from std_msgs.msg import Bool, String
import time
import json


TEST_PATH = os.path.dirname(os.path.abspath(__file__))

class Text2SpeechNodeTest(object):
    def __init__(self):
        """
        Initializes the Text-to-Speech (T2S) node testing module."""
        self._output = None
        self._groundtruth = None
        self._pub_human_presence=rospy.Publisher(HUMAN_PRESENCE_TOPIC,Bool,queue_size=0)
        self._pub_output_chatbot=rospy.Publisher(CHATBOT_OUTPUT_TOPIC, String, queue_size=0)
        self._HUMAN_PRESENCE=False
        self._OUTPUT_CHATBOT=False
        self._pepper_talk_curr=False
        self._pepper_talk_past=False

    def _setup(self,test_case_path):
        """
        Load groundtruth from file stored in specific test_case_path.
        Args:
            test_case_path (String)
        """
        with open(os.path.join(test_case_path,"groundtruth.txt"),"r") as f:
            g = f.read()
        if g=="False":
            self._groundtruth=False
        if g=="":
            self._groundtruth="Groundtruth empty"
        if g=="True":
            self._groundtruth=True
        return

    def _cleanup(self):
        """
        Reset groundtruth and detector output
        """
        self._groundtruth = None
        self._output = None

    def _test(self):
        """
        Test the correctness of the detector output
        """
        # check if the sequece 
        if self._get_groundtruth()=="Groundtruth empty":
            print("Groundtruth empty")
        else:
            if self._output == self._groundtruth:
                print("Passed")
            else:
                print("gt: ")
                print(self._get_groundtruth())
                print("output: ")
                print(self._get_output())
                print("Failed")
    
    def _get_output(self):
        """
        Return detector output
        """
        return self._output
    
    def _set_output(self,output):
        """
        Set detector output
        """
        self._output=output
    
    def _get_groundtruth(self):
        """
        Return groundtruth
        """
        return self._groundtruth

    def __read_config(self, file_path):
        """
        Read configuration values from a JSON file.

        Args:
            file_path (String): The path to the JSON configuration file.
        """
        try:
            with open(file_path, 'r') as json_file:
                data = json.load(json_file)
                # Imposta le variabili in base ai valori nel file JSON
                self._OUTPUT_CHATBOT = data.get("TEST_OUTPUT_CHATBOT", False)
                self._HUMAN_PRESENCE = data.get("TEST_HUMAN_PRESENCE", False)
        except FileNotFoundError:
            print(f"File '{file_path}' non trovato. Impostazione di valori predefiniti.")


    def _test_case(self,test_case_folder):
        """
        Execute a test case based on data stored in the specified folder.

        Args:
            test_case_folder (String): Folder where test data is stored.
        """
        print(test_case_folder.upper()+":",end="\t")
        test_case_path = os.path.join(TEST_PATH,test_case_folder)
        # get groundtruth
        self._setup(test_case_path)

        text_path = test_case_path
        self.__read_config(os.path.join(text_path, "config_test.json"))
        #pubblico sui topic, il nodo di t2s leggerà questi valori
        self._pub_human_presence.publish(Bool(self._HUMAN_PRESENCE))
        if(self._OUTPUT_CHATBOT!=""): #diverso da stringa vuota
            self._pub_output_chatbot.publish(String(self._OUTPUT_CHATBOT))

        time.sleep(5)#in questo modo il nodo di ts2_node ha il tempo di pubblicare (se deve)
        #sul topic PEPPER_TALK_TOPIC

        #print("##################_test_case")
        #print("self._pepper_talk_curr: ")
        #print(self._pepper_talk_curr)
        #print("self._pepper_talk_past")
        #print(self._pepper_talk_past)

        if (self._pepper_talk_curr==False and self._pepper_talk_past==True):
            print("####if in test_case è true")
            self._set_output(True)
        else: 
            self._set_output(False)

        self._test()
        self._cleanup()   
        
        #reset di _pepper_talk_past e _pepper_talk_curr
        self._pepper_talk_curr=False
        self._pepper_talk_past=False

    def _pepper_talk(self, flag):
        """
        Callback function for the PEPPER_TALK_TOPIC.

        Args:
            flag (std_msgs/Bool): Flag indicating whether Pepper is talking.
        """
        self._pepper_talk_past=self._pepper_talk_curr
        self._pepper_talk_curr=flag.data

    def start(self):
        """
        Start the testing process. Initialize the ROS node, subscribe to the T2S node topic,
        publish flags and chatbot output messages, and wait for the ROS node to terminate.
        """
        rospy.init_node('text2speech_node_test', anonymous=True)
        rospy.Subscriber(PEPPER_TALK_TOPIC, Bool,self._pepper_talk)
   
        test_cases = os.listdir(TEST_PATH)
        test_cases.sort()
        
        for test in test_cases:
            if test!="__pycache__" and test!="t2s_node_test.py" and test!="config.py":
                if not os.path.isfile(os.path.join(TEST_PATH,test)):
                    self._test_case(test)
        print("TEST FINISHED")
        print("Type CTRL+C to exit")
        while not rospy.is_shutdown():
            pass
        

if __name__ == "__main__":
    try:
        d = Text2SpeechNodeTest()
        d.start()
    except rospy.ROSInterruptException:
        pass